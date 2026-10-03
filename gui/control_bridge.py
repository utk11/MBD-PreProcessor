"""Opt-in local IPC; every document and viewer command executes on Qt's GUI thread."""

from __future__ import annotations

from collections import OrderedDict
import json
from pathlib import Path
import tempfile
import uuid

import numpy as np
from PySide6.QtCore import QObject
from PySide6.QtNetwork import QLocalServer

from core.project_store import read_project, resolve_step_file, save_project
from core.transforms import body_world_pose


def json_safe(value):
    """Never put NaN or Infinity into a JSON response."""
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


class ApplicationControlBridge(QObject):
    def __init__(self, window, socket_name="mbd-preprocessor"):
        super().__init__(window)
        self.window = window
        self.operations = OrderedDict()
        self.connections = {}
        self.server = QLocalServer(self)
        self.server.setSocketOptions(QLocalServer.UserAccessOption)
        self.server.newConnection.connect(self._connect)
        if not self.server.listen(socket_name):
            raise RuntimeError(f"Control socket '{socket_name}' unavailable: {self.server.errorString()}")
        window.controller.scheduler.report_processed.connect(self._solve_finished)
        window.load_finished.connect(self._load_finished)

    def close(self):
        self.server.close()
        for socket in list(self.connections):
            socket.abort()

    def _connect(self):
        while self.server.hasPendingConnections():
            socket = self.server.nextPendingConnection()
            self.connections[socket] = bytearray()
            socket.readyRead.connect(lambda s=socket: self._read(s))
            socket.disconnected.connect(lambda s=socket: self._disconnect(s))

    def _disconnect(self, socket):
        self.connections.pop(socket, None)
        socket.deleteLater()

    def _read(self, socket):
        buffer = self.connections[socket]
        buffer.extend(bytes(socket.readAll()))
        if len(buffer) > 65536:
            socket.abort()
            return
        if b"\n" not in buffer:
            return
        try:
            request = json.loads(bytes(buffer).split(b"\n", 1)[0])
            result = self.dispatch(request["command"], request.get("arguments", {}))
            response = {"ok": True, "result": json_safe(result)}
        except Exception as exc:
            response = {"ok": False, "error": str(exc)}
        socket.write(json.dumps(response, allow_nan=False).encode("utf-8") + b"\n")
        socket.disconnectFromServer()

    def _idle(self):
        w = self.window
        queue = w.controller.scheduler.queue
        if w._load_workers or queue.active is not None or queue.pending is not None or w._dragging_body_ref is not None:
            raise ValueError("Application is busy. Check get_state/get_operation before retrying.")

    def _body(self, body_id):
        body = self.window.document.body_by_id(int(body_id))
        if body is None:
            raise ValueError(f"Body {body_id} does not exist.")
        return body

    def _operation(self, kind, identity):
        # Bounded history; completed operations can be evicted, active ones cannot.
        for key in list(self.operations):
            if len(self.operations) < 64:
                break
            if self.operations[key]["status"] != "running":
                del self.operations[key]
        key = uuid.uuid4().hex
        result = {"operation_id": key, "kind": kind, "identity": identity, "status": "running"}
        self.operations[key] = result
        return dict(result)

    def _solve_finished(self, report, accepted):
        for operation in self.operations.values():
            if operation["kind"] != "load" and operation["status"] == "running" and operation["identity"] == report.request_id:
                operation.update(
                    status="completed" if accepted else "failed" if not report.finite else "superseded",
                    result=json_safe({
                        "converged": report.converged, "joint_feasible": report.joint_feasible,
                        "target_error_meters": report.target_error, "max_residual": report.max_residual,
                        "iterations": report.iterations, "dof": report.dof,
                        "message": report.message, "per_joint_residual": report.per_joint_residual,
                    }),
                )

    def _load_finished(self, generation, success, message):
        for operation in self.operations.values():
            if operation["kind"] == "load" and operation["identity"] == generation:
                operation.update(status="completed" if success else "failed", message=message)

    def dispatch(self, command, arguments):
        w = self.window
        if w._close_requested:
            raise ValueError("Application is closing.")
        if not isinstance(arguments, dict):
            raise ValueError("Arguments must be an object.")
        if command == "get_state":
            queue = w.controller.scheduler.queue
            return json_safe({
                "generation": w.document.generation, "unit_scale": w.document.unit_scale,
                "source_cad_path": w.document.source_cad_path,
                "loading": bool(w._load_workers), "solver_active": None if queue.active is None else queue.active.kind,
                "solver_pending": None if queue.pending is None else queue.pending.kind,
                "bodies": [{
                    "id": b.id, "name": b.name, "visible": b.visible,
                    "origin_meters": body_world_pose(b)[0], "rotation_matrix": body_world_pose(b)[1],
                } for b in w.document.bodies],
                "joints": [{"name": j.name, "type": j.joint_type.name,
                    "body1_id": j.body1_id, "body2_id": j.body2_id, "axis": j.axis,
                    "motorized": j.is_motorized} for j in w.document.joints.values()],
                "frames": [{"name": name, "parent_body_id": w.document.frame_to_body.get(name),
                    "coordinates": w.document.frame_coordinates.get(name, "world"),
                    "origin": f.origin, "rotation_matrix": f.rotation_matrix} for name, f in w.document.frames.items()],
                "forces": list(w.document.forces), "torques": list(w.document.torques),
            })
        if command == "get_operation":
            operation = self.operations[arguments["operation_id"]]
            if operation["kind"] == "load" and operation["status"] == "running" and operation["identity"] < w._load_generation:
                operation["status"] = "superseded"
            return dict(operation)
        if command == "capture_view":
            with tempfile.NamedTemporaryFile(suffix=".png", prefix="mbd-view-", delete=False) as output:
                path = output.name
            if not w.display.View.Dump(path):
                Path(path).unlink(missing_ok=True)
                raise RuntimeError("Viewer capture failed.")
            return {"path": path}
        self._idle()
        if command in ("load_step", "load_project"):
            path = Path(arguments["path"]).expanduser().resolve(strict=True)
            project = read_project(str(path)) if command == "load_project" else None
            cad = resolve_step_file(str(path), project) if project else str(path)
            if cad is None:
                raise ValueError("Project's STEP file is missing. Restore the file or correct its saved path.")
            generation = w._start_import(cad, project, interactive=False)
            return self._operation("load", generation)
        if command == "save_project":
            if not w.document.source_cad_path:
                raise ValueError("Load CAD before saving a project.")
            path = Path(arguments["path"]).expanduser().resolve()
            if path.suffix.lower() != ".mbdp":
                raise ValueError("Project filename must end in .mbdp.")
            if path.exists() and not arguments.get("overwrite", False):
                raise ValueError("Destination exists. Pass overwrite=true to replace it.")
            save_project(str(path), w.document, w.document.source_cad_path)
            return {"path": str(path), "saved": True}
        if command == "solve_assembly":
            if not w.document.joints or w.document.state is None:
                raise ValueError("Load bodies and create joints before solving.")
            identity = w.controller.solve_assembly(notify=False)
            if identity is None:
                raise ValueError("Solver did not accept the request.")
            return self._operation("assembly", identity)
        if command == "move_body":
            body = self._body(arguments["body_id"])
            position = np.asarray(arguments["origin_meters"], dtype=float)
            if position.shape != (3,) or not np.isfinite(position).all():
                raise ValueError("origin_meters must contain three finite numbers.")
            if body.state is None:
                raise ValueError("Body has no live pose.")
            w.controller.on_drag_start(body.id)
            w.controller.submit_drag_target(body.id, position)
            if body.id not in w.document.constrained_body_ids():
                return {"status": "completed", "origin_meters": body_world_pose(body)[0].tolist()}
            identity = w.controller.scheduler.queue.next_request_id - 1
            w.controller.finish_drag(body.id)
            return self._operation("drag", identity)
        if command == "select_body":
            body = self._body(arguments["body_id"])
            w.on_body_selected(body.id)
            return {"selected_body_id": body.id}
        if command == "set_body_visibility":
            body = self._body(arguments["body_id"])
            if not isinstance(arguments["visible"], bool):
                raise ValueError("visible must be a boolean.")
            w.on_body_visibility_changed(body.id, arguments["visible"])
            return {"body_id": body.id, "visible": body.visible}
        if command == "set_camera":
            views = {name: getattr(w, f"set_view_{name}") for name in (
                "top", "bottom", "front", "back", "left", "right", "isometric",
            )}
            preset = arguments["preset"]
            if preset not in views:
                raise ValueError(f"Unknown camera preset {preset!r}.")
            views[preset]()
            if arguments.get("fit_all", True):
                w.display.FitAll()
            return {"preset": preset}
        raise ValueError(f"Unknown control command {command!r}.")
