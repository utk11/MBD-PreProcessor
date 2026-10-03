"""MCP stdio adapter. Does not launch the GUI or write logs to stdout."""

import argparse
from pathlib import Path

from PySide6.QtCore import QCoreApplication
from mcp.server.fastmcp import FastMCP, Image
from mcp.types import ToolAnnotations

from control_client import call_application


def build_server(socket_name="mbd-preprocessor"):
    server = FastMCP("MBD PreProcessor")
    read = ToolAnnotations(readOnlyHint=True, destructiveHint=False, openWorldHint=False)
    edit = ToolAnnotations(readOnlyHint=False, destructiveHint=False, openWorldHint=False)
    replace = ToolAnnotations(readOnlyHint=False, destructiveHint=True, openWorldHint=False)

    def call(command, **arguments):
        return call_application(command, arguments, socket_name)

    @server.tool(annotations=read)
    def get_state() -> dict:
        """Inspect live body poses in meters, joints, frames, loads and solver/import activity."""
        return call("get_state")

    @server.tool(annotations=read)
    def get_operation(operation_id: str) -> dict:
        """Read a load/solve/move result. completed does not imply converged; inspect result fields."""
        return call("get_operation", operation_id=operation_id)

    @server.tool(annotations=replace)
    def load_step(path: str) -> dict:
        """Replace the current document with a local STEP file. Returns an operation ID."""
        return call("load_step", path=path)

    @server.tool(annotations=replace)
    def load_project(path: str) -> dict:
        """Restore a local .mbdp project. Current document survives failure. Poll the operation ID."""
        return call("load_project", path=path)

    @server.tool(annotations=replace)
    def save_project(path: str, overwrite: bool = False) -> dict:
        """Save the current assembly to a local .mbdp file; replacing a file needs overwrite=true."""
        return call("save_project", path=path, overwrite=overwrite)

    @server.tool(annotations=edit)
    def solve_assembly() -> dict:
        """Run the existing JAX assembly solver. Poll the operation ID; no GUI dialog is opened."""
        return call("solve_assembly")

    @server.tool(annotations=edit)
    def move_body(body_id: int, origin_meters: list[float]) -> dict:
        """Move toward an absolute world position through the normal drag solver. Joints limit reachable motion."""
        return call("move_body", body_id=body_id, origin_meters=origin_meters)

    @server.tool(annotations=edit)
    def select_body(body_id: int) -> dict:
        """Select and highlight a body in the GUI."""
        return call("select_body", body_id=body_id)

    @server.tool(annotations=edit)
    def set_body_visibility(body_id: int, visible: bool) -> dict:
        """Show or hide a body in the current document and viewer."""
        return call("set_body_visibility", body_id=body_id, visible=visible)

    @server.tool(annotations=edit)
    def set_camera(preset: str, fit_all: bool = True) -> dict:
        """Use top, bottom, front, back, left, right, or isometric view; optionally fit the assembly."""
        return call("set_camera", preset=preset, fit_all=fit_all)

    @server.tool(annotations=read)
    def capture_view() -> Image:
        """Return a PNG image of the live CAD viewport."""
        result = call("capture_view")
        path = Path(result["path"])
        try:
            return Image(data=path.read_bytes(), format="png")
        finally:
            path.unlink(missing_ok=True)

    return server


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--control-socket", default="mbd-preprocessor")
    options = parser.parse_args()
    app = QCoreApplication.instance() or QCoreApplication([])
    server = build_server(options.control_socket)
    server.run(transport="stdio")
    return app


if __name__ == "__main__":
    main()
