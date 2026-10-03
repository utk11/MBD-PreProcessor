# Application control through MCP

This server controls the running MBD PreProcessor through its application
commands. It does not automate mouse clicks or execute arbitrary Python.

## Start the application

From the repository directory in PowerShell:

```powershell
& 'C:\miniconda\envs\mbd_preproc\python.exe' -B main.py --enable-mcp
```

Control is disabled during ordinary `main.py` startup. The extra flag opens
an OS-local socket named `mbd-preprocessor`, restricted to the current user.
There is no network port. A second application cannot take over that socket.
For multiple instances, use distinct `--control-socket` names in both the GUI
launch command and the MCP server arguments.

The installed SDK lives in `.venv-mcp`, an isolated virtual environment that
inherits the app environment's Qt packages. To recreate it on another machine:

```powershell
& 'C:\miniconda\envs\mbd_preproc\python.exe' -m venv --system-site-packages .venv-mcp
& '.\.venv-mcp\Scripts\python.exe' -m pip install -r requirements-mcp.txt
```

## Connect Codex

The installed project-scoped `.codex/config.toml` is local and Git-ignored
because it contains machine-specific paths. It uses these values. Paths are absolute so
the server also works when the client starts in another directory:

```toml
[mcp_servers.mbd_preprocessor]
command = 'C:/Users/Utkarsh Kulkarni/Documents/Pre_processor/MBD-PreProcessor/.venv-mcp/Scripts/python.exe'
args = ['-B', 'C:/Users/Utkarsh Kulkarni/Documents/Pre_processor/MBD-PreProcessor/mcp_server.py']
cwd = 'C:/Users/Utkarsh Kulkarni/Documents/Pre_processor/MBD-PreProcessor'
startup_timeout_sec = 20
tool_timeout_sec = 20
```

Codex supports project-scoped `.codex/config.toml` MCP configuration, as
described in the [official configuration documentation](https://learn.chatgpt.com/docs/extend/mcp?surface=cli).
Reopen the project or restart the client to discover the new tools. The GUI
must be started with control enabled before calling a tool. The MCP process
can initialize and list its tools even while the GUI is closed; tool calls
then return an application-unavailable error.

Other local MCP clients can start the same executable and script over stdio.
The implementation uses the [official Python SDK](https://github.com/modelcontextprotocol/python-sdk/tree/v1.x)
and the standard [MCP stdio transport](https://modelcontextprotocol.io/specification/2025-11-25/basic/transports).
Application logs remain in the GUI process. MCP stdout contains protocol
messages only.

## Tools

| Tool | Behavior |
| --- | --- |
| `get_state` | Body poses, body settings, joints, stored frame coordinates, load names, and activity |
| `get_operation` | Completion, failure, supersession, and numerical results for a returned operation ID |
| `load_step` | Import a STEP file and replace the document after successful import |
| `load_project` | Restore a `.mbdp` project; preserve the open document on restoration failure |
| `save_project` | Save `.mbdp`; existing destinations require explicit `overwrite=true` |
| `solve_assembly` | Solve using the existing JAX worker and commit a current, finite result |
| `move_body` | Request an absolute origin in world meters through the normal constrained drag path |
| `select_body` | Select and highlight a body |
| `set_body_visibility` | Show or hide a body |
| `set_camera` | Set a named view, optionally fitting the assembly |
| `capture_view` | Return a PNG of the CAD viewport as MCP image content |

Example sequence:

1. Call `get_state` to identify body IDs and check activity.
2. Call `move_body(body_id=2, origin_meters=[-0.01, -0.025, 0.0075])`.
3. Poll the returned `operation_id` using `get_operation` until terminal.
4. Inspect `joint_feasible` and `target_error_meters`, then call `capture_view`.

`completed` means the result was applied; it does not promise the requested
point was reachable or the assembly converged. Inspect the numerical fields.
The same joint constraints and solver limits apply to mouse and MCP movement.
Mutation commands reject busy applications instead of interrupting a drag,
import, or solve. Inspection remains available. A completed drag can be
followed briefly by automatic diagnostics; check `solver_active` before the
next mutation. Commands execute once: after a timeout, inspect the state
before retrying a mutation.

Project load and assembly solve commands report through operation results
instead of showing modal success dialogs. Normal GUI commands retain their
existing dialogs. Tool annotations distinguish read-only actions and actions
that replace documents or files; the MCP client's approval policy still applies.
The initial tools cover the actions above. Joint creation/deletion, motors,
loads, and export are not exposed by this first version.

## Architecture and validation

`mcp_server.py` declares the tools and runs MCP stdio. `control_client.py`
opens a short-lived local socket for each request. `gui/control_bridge.py`
receives the command using Qt's event loop and executes document/viewer work
on the GUI thread. Numerical requests reuse `ApplicationController` and its
existing JAX worker, including revision checks. Operation history is bounded
to 64 entries. Closing the GUI closes the listener and its connections.

Run the MCP-specific checks in the optional environment:

```powershell
& '.\.venv-mcp\Scripts\python.exe' -B -m unittest tests.test_mcp_control -v
```

They exercise a real MCP client and stdio subprocess, local IPC dispatched
on the GUI thread, real JAX movement/assembly, busy-state rejection, validation,
overwrite protection, and asynchronous failure reporting. The CAD viewport
capture itself requires a running graphical viewer; headless protocol tests
do not establish the visual appearance of that capture.
