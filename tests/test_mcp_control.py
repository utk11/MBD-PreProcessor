"""Headless control commands, real Qt IPC, and a real MCP stdio client."""

import asyncio
import json
from pathlib import Path
import sys
import subprocess
import tempfile
from threading import Event, Thread, get_ident
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import uuid

import numpy as np
from PySide6.QtCore import QCoreApplication, QObject, Signal

from core.assembly_document import AssemblyDocument
from core.kinematics.reports import SolveReport
from gui.application_controller import ApplicationController
from gui.control_bridge import ApplicationControlBridge, json_safe
from control_client import call_application
from tests.kinematics_fixtures import build_pendulum, GROUND_POSE
from tests.test_refactor_regressions import spin_until

ROOT = Path(__file__).resolve().parents[1]


class ControlWindow(QObject):
    load_finished = Signal(int, bool, str)

    def __init__(self):
        super().__init__()
        bodies, joints, state = build_pendulum(False)
        self.document = AssemblyDocument()
        self.document.replace_bodies(bodies)
        self.document.state = state
        self.document.add_joint(joints[0])
        self._load_workers = set()
        self._close_requested = False
        self._dragging_body_ref = None
        self._load_generation = 0
        self.display = Mock()
        self.controller = ApplicationController(self)
        self.selected = None
        self.status = Mock()
        self._on_solver_settled = Mock()
        for name in ("top", "bottom", "front", "back", "left", "right", "isometric"):
            setattr(self, f"set_view_{name}", Mock())

    def _body_pose_tuple(self, body_id):
        return GROUND_POSE

    def statusBar(self):
        return self.status

    def on_body_selected(self, body_id):
        self.selected = body_id

    def on_body_visibility_changed(self, body_id, visible):
        self.document.body_by_id(body_id).visible = visible

    def _start_import(self, path, project, interactive=True):
        self._load_generation += 1
        self.import_arguments = (path, project, interactive)
        return self._load_generation


class ControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.window = ControlWindow()
        self.name = "mbd-test-" + uuid.uuid4().hex
        self.bridge = ApplicationControlBridge(self.window, self.name)

    def tearDown(self):
        self.bridge.close()
        self.window.controller.shutdown()
        self.assertTrue(spin_until(lambda: self.window.controller.scheduler.is_stopped))
        self.window.deleteLater()

    def test_state_selection_visibility_and_validation(self):
        state = self.bridge.dispatch("get_state", {})
        self.assertEqual(state["bodies"][0]["origin_meters"], [1.0, 0.0, -1.0])
        self.bridge.dispatch("select_body", {"body_id": 1})
        self.assertEqual(self.window.selected, 1)
        self.bridge.dispatch("set_body_visibility", {"body_id": 1, "visible": False})
        self.assertFalse(self.window.document.bodies[0].visible)
        for arguments in ({"body_id": 99, "origin_meters": [0, 0, 0]},
                          {"body_id": 1, "origin_meters": [0, 0]},
                          {"body_id": 1, "origin_meters": [np.nan, 0, 0]}):
            with self.assertRaises(ValueError):
                self.bridge.dispatch("move_body", arguments)
        self.assertEqual(json_safe({"bad": float("inf")}), {"bad": None})

    def test_real_jax_move_and_assembly_return_operations_without_dialogs(self):
        with patch("gui.application_controller.QMessageBox.information") as dialog:
            move = self.bridge.dispatch("move_body", {"body_id": 1, "origin_meters": [0.995, 0.1, -1.0]})
            operation = self.bridge.operations[move["operation_id"]]
            self.assertTrue(spin_until(lambda: operation["status"] != "running", 15000))
            self.assertEqual(operation["status"], "completed")
            self.assertLess(operation["result"]["target_error_meters"], 0.001)
            self.assertTrue(spin_until(lambda: self.window.controller.scheduler.queue.active is None))
            assembly = self.bridge.dispatch("solve_assembly", {})
            operation = self.bridge.operations[assembly["operation_id"]]
            self.assertTrue(spin_until(lambda: operation["status"] != "running"))
            self.assertEqual(operation["status"], "completed")
            self.assertTrue(operation["result"]["converged"])
            dialog.assert_not_called()

    def test_busy_commands_rejected_but_inspection_remains_available(self):
        self.window._load_workers.add(object())
        with self.assertRaisesRegex(ValueError, "busy"):
            self.bridge.dispatch("select_body", {"body_id": 1})
        self.assertTrue(self.bridge.dispatch("get_state", {})["loading"])

    def test_save_overwrite_and_load_failure_tracking(self):
        with tempfile.TemporaryDirectory() as directory:
            cad = Path(directory) / "part.step"
            cad.write_bytes(b"test CAD")
            self.window.document.source_cad_path = str(cad)
            path = str(Path(directory) / "test.mbdp")
            self.bridge.dispatch("save_project", {"path": path})
            with self.assertRaisesRegex(ValueError, "exists"):
                self.bridge.dispatch("save_project", {"path": path})
            self.bridge.dispatch("save_project", {"path": path, "overwrite": True})
            load = self.bridge.dispatch("load_project", {"path": path})
            self.assertFalse(self.window.import_arguments[2])
            self.window.load_finished.emit(load["identity"], False, "CAD import failed")
            result = self.bridge.dispatch("get_operation", {"operation_id": load["operation_id"]})
            self.assertEqual(result["status"], "failed")
            self.assertEqual(result["message"], "CAD import failed")
            self.assertEqual(len(self.window.document.bodies), 1)

    def test_real_local_socket_runs_commands_on_gui_thread(self):
        threads = []
        dispatch = self.bridge.dispatch

        def record(command, arguments):
            threads.append(get_ident())
            return dispatch(command, arguments)

        self.bridge.dispatch = record

        code = (
            "import json,sys; from PySide6.QtCore import QCoreApplication; "
            "from control_client import call_application; app=QCoreApplication([]); "
            "print(json.dumps(call_application('get_state',socket_name=sys.argv[1])))"
        )
        # The real deployment uses separate processes. Avoid blocking Qt
        # socket waits in a Python thread sharing the GUI interpreter's GIL.
        worker = subprocess.Popen(
            [sys.executable, "-B", "-c", code, self.name], cwd=str(ROOT),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        try:
            self.assertTrue(spin_until(lambda: worker.poll() is not None, 15000))
            stdout, stderr = worker.communicate(timeout=1)
            self.assertEqual(worker.returncode, 0, stderr)
            self.assertEqual(json.loads(stdout)["bodies"][0]["id"], 1)
        finally:
            if worker.poll() is None:
                worker.kill()
                worker.communicate()
        self.assertEqual(threads, [get_ident()])

    def test_mcp_stdio_initialization_discovery_and_live_tool_calls(self):
        try:
            from mcp import ClientSession, StdioServerParameters
            from mcp.client.stdio import stdio_client
        except ImportError:
            self.skipTest("Install requirements-mcp.txt to test the stdio adapter")
        done = Event()
        result = {}

        async def client():
            parameters = StdioServerParameters(
                command=sys.executable, args=["-B", str(ROOT / "mcp_server.py"), "--control-socket", self.name],
                cwd=str(ROOT),
            )
            async with stdio_client(parameters) as (read, write):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    tools = await session.list_tools()
                    self.assertEqual(len(tools.tools), 11)
                    state = await session.call_tool("get_state", {})
                    self.assertFalse(state.isError)
                    self.assertEqual(json.loads(state.content[0].text)["bodies"][0]["id"], 1)
                    selected = await session.call_tool("select_body", {"body_id": 1})
                    self.assertFalse(selected.isError)
                    invalid = await session.call_tool("move_body", {"body_id": 99, "origin_meters": [0, 0, 0]})
                    self.assertTrue(invalid.isError)

        def run():
            try:
                asyncio.run(client())
            except BaseException as exc:
                result["error"] = exc
            finally:
                done.set()

        worker = Thread(target=run)
        worker.start()
        self.assertTrue(spin_until(done.is_set, 20000))
        worker.join(timeout=5)
        if "error" in result:
            raise result["error"]
        self.assertEqual(self.window.selected, 1)


if __name__ == "__main__":
    unittest.main()
