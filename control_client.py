"""Blocking local IPC client used only by the separate MCP process."""

import json
import time

from PySide6.QtNetwork import QLocalSocket


def call_application(command, arguments=None, socket_name="mbd-preprocessor", timeout_ms=10000):
    socket = QLocalSocket()
    try:
        socket.connectToServer(socket_name)
        if not socket.waitForConnected(1000):
            raise RuntimeError("Application control is unavailable. Start main.py --enable-mcp and check the socket name.")
        payload = json.dumps({"command": command, "arguments": arguments or {}}, allow_nan=False).encode() + b"\n"
        socket.write(payload)
        if socket.bytesToWrite() and not socket.waitForBytesWritten(1000):
            raise RuntimeError("Could not send the control request.")
        response = bytearray()
        deadline = time.monotonic() + timeout_ms / 1000.0
        while b"\n" not in response:
            response.extend(bytes(socket.readAll()))
            if b"\n" in response:
                break
            remaining = int((deadline - time.monotonic()) * 1000)
            if remaining <= 0 or not socket.waitForReadyRead(remaining):
                response.extend(bytes(socket.readAll()))
                if b"\n" not in response:
                    raise RuntimeError("No reply from the app. A mutation may still be running; inspect state before retrying.")
            if len(response) > 8 * 1024 * 1024:
                raise RuntimeError("Application reply exceeds 8 MiB.")
        result = json.loads(bytes(response).split(b"\n", 1)[0])
        if not result.get("ok"):
            raise ValueError(result.get("error", "Application command failed."))
        return result["result"]
    finally:
        socket.abort()
