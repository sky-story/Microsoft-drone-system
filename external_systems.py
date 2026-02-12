#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Modular external system controllers (WebSocket).

Provides a scalable base and concrete controllers:
- BaseExternalSystem: shared WebSocket connect/send/disconnect
- WinchController: winch/lower/pull/stop (lower:mm, pull:mm, stop)
- GripperController: hold | release | grip | status
- ExternalSystemsManager: register and manage multiple systems by name
"""

import time
from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional, Dict, Any

import websocket

# Require websocket-client (has create_connection). The "websocket" package does not.
if not getattr(websocket, "create_connection", None):
    raise ImportError(
        "Wrong WebSocket package installed. Use the client library:\n"
        "  pip uninstall websocket\n"
        "  pip install websocket-client\n"
        "Then run again."
    )


class GrabState(Enum):
    """Winch/grab sequence state."""
    IDLE = "idle"
    TRACKING = "tracking"
    TRIGGERED = "triggered"
    LOWERING = "lowering"
    WAITING = "waiting"
    PULLING = "pulling"
    COMPLETED = "completed"
    ERROR = "error"


# ---------------------------------------------------------------------------
# Base
# ---------------------------------------------------------------------------


class BaseExternalSystem(ABC):
    """Base for WebSocket-based external systems. Subclass and implement command protocol."""

    def __init__(self, ws_url: str, timeout: float = 5.0, log_prefix: str = "EXTERNAL"):
        self.ws_url = ws_url
        self.timeout = timeout
        self.log_prefix = log_prefix
        self.ws = None
        self.connected = False

    def connect(self) -> bool:
        """Connect to WebSocket server."""
        try:
            print(f"[{self.log_prefix}] Connecting to WebSocket: {self.ws_url}")
            self.ws = websocket.create_connection(self.ws_url, timeout=self.timeout)
            self.connected = True
            print(f"[{self.log_prefix}] ✓ WebSocket connected")
            return True
        except Exception as e:
            print(f"[{self.log_prefix}] ✗ WebSocket connection failed: {e}")
            self.connected = False
            return False

    def disconnect(self) -> None:
        """Disconnect WebSocket."""
        if self.ws:
            try:
                self.ws.close()
                print(f"[{self.log_prefix}] WebSocket disconnected")
            except Exception as e:
                print(f"[{self.log_prefix}] Disconnect error: {e}")
            finally:
                self.ws = None
                self.connected = False

    def send_raw(self, cmd_str: str, expect_response: bool = True) -> Optional[str]:
        """
        Send a raw command string and optionally wait for one response line.

        Returns:
            Response string if expect_response and recv succeeded, else None.
            On send failure returns None.
        """
        if not self.connected or not self.ws:
            if not self.connect():
                return None
        try:
            print(f"[{self.log_prefix}] Sending: {cmd_str} -> {self.ws_url}")
            self.ws.send(cmd_str)
            if expect_response:
                try:
                    # Grip (and other ops) can take several seconds; use long recv timeout so we don't get None
                    recv_timeout = getattr(self, "recv_timeout", 30.0)
                    old_timeout = None
                    if hasattr(self.ws, "sock") and self.ws.sock is not None:
                        old_timeout = self.ws.sock.gettimeout()
                        self.ws.sock.settimeout(recv_timeout)
                    try:
                        response = self.ws.recv()
                    finally:
                        if old_timeout is not None and hasattr(self.ws, "sock") and self.ws.sock is not None:
                            self.ws.sock.settimeout(old_timeout)
                    # Skip welcome/help messages (ESP may send "connected" then "cmds: hold | release | ..." on connect).
                    # Keep recv until we get a real command response (contains "ok" or "err").
                    while response is not None and "ok" not in response.lower() and "err" not in response.lower():
                        r = response.strip().lower()
                        if r.startswith("connected") or "cmds:" in r:
                            print(f"[{self.log_prefix}] (skip: {response.strip()[:60]})")
                            response = self.ws.recv()
                        else:
                            break
                    print(f"[{self.log_prefix}] ✓ Response: {response}")
                    return response
                except Exception as e:
                    print(f"[{self.log_prefix}] ✓ Sent (no response / timeout: {e})")
                    return None
            return ""  # sent, no response expected
        except websocket.WebSocketConnectionClosedException:
            print(f"[{self.log_prefix}] ✗ Connection closed, reconnecting...")
            self.connected = False
            if self.connect():
                return self.send_raw(cmd_str, expect_response)
            return None
        except Exception as e:
            print(f"[{self.log_prefix}] ✗ Error: {e}")
            return None

    def recv_with_timeout(self, timeout_sec: float) -> Optional[str]:
        """
        Wait for one message from the WebSocket with the given timeout.
        Use after a command that will send a follow-up (e.g. winch "ok: done").
        """
        if not self.connected or not self.ws:
            return None
        try:
            old_timeout = None
            if hasattr(self.ws, "sock") and self.ws.sock is not None:
                old_timeout = self.ws.sock.gettimeout()
                self.ws.sock.settimeout(timeout_sec)
            try:
                response = self.ws.recv()
                return response
            finally:
                if old_timeout is not None and hasattr(self.ws, "sock") and self.ws.sock is not None:
                    self.ws.sock.settimeout(old_timeout)
        except Exception as e:
            print(f"[{self.log_prefix}] recv timeout or error: {e}")
            return None


# ---------------------------------------------------------------------------
# Winch (lower / pull / stop)
# ---------------------------------------------------------------------------


class WinchController(BaseExternalSystem):
    """Winch/lowering device: lower:mm, pull:mm, stop."""

    def __init__(
        self,
        ws_url: str = "ws://192.168.42.15",
        timeout: float = 5.0,
        lower_length: float = 100.0,
        pull_length: float = 50.0,
        log_prefix: str = "WINCH",
    ):
        super().__init__(ws_url, timeout, log_prefix=log_prefix)
        self.lower_length = lower_length
        self.pull_length = pull_length
        self.state = GrabState.IDLE

    def send_command(self, command: str, length: float = 0) -> bool:
        """
        Send command: "lower", "pull", or "stop".
        length in mm for lower/pull.
        """
        if command == "stop":
            cmd_str = "stop"
        elif command == "lower":
            cmd_str = f"lower:{int(length)}"
        elif command == "pull":
            cmd_str = f"pull:{int(length)}"
        else:
            print(f"[{self.log_prefix}] ✗ Unknown command: {command}")
            return False
        return self.send_raw(cmd_str) is not None

    def execute_grab_sequence(
        self,
        gripper: Optional["GripperController"] = None,
        wait_time: float = 0.0,
        pull_time: float = 0.0,
        lower_done_timeout: float = 120.0,
        pull_done_timeout: float = 120.0,
    ) -> bool:
        """
        External workflow (each step waits for ok, winch steps also wait for ok: done):
          1. Gripper release → received ok
          2. Winch lower length_mm → received ok → wait for ok: done
          3. Gripper grip → received ok
          4. Winch pull length_mm → received ok → wait for ok: done
        """
        def received_ok(resp: Optional[str]) -> bool:
            return resp is not None and "ok" in (resp or "").lower()

        self.state = GrabState.TRIGGERED

        if gripper:
            print(f"[{self.log_prefix}] Gripper release...")
            resp = gripper.send_raw("release")
            if not received_ok(resp):
                print(f"[{self.log_prefix}] ✗ No ok after gripper release: {resp}")
                self.state = GrabState.ERROR
                return False
            print(f"[{self.log_prefix}] ✓ received ok (release)")

        self.state = GrabState.LOWERING
        print(f"[{self.log_prefix}] Winch lower {self.lower_length}mm...")
        resp = self.send_raw(f"lower:{int(self.lower_length)}")
        if not received_ok(resp):
            print(f"[{self.log_prefix}] ✗ No ok after lower: {resp}")
            self.state = GrabState.ERROR
            return False
        print(f"[{self.log_prefix}] ✓ received ok (lower), waiting for ok: done...")
        if not self.wait_for_done(timeout_sec=lower_done_timeout):
            self.state = GrabState.ERROR
            return False

        if gripper:
            self.state = GrabState.WAITING
            print(f"[{self.log_prefix}] Gripper grip...")
            resp = gripper.send_raw("grip")
            if not received_ok(resp):
                print(f"[{self.log_prefix}] ✗ No ok after gripper grip: {resp}")
                self.state = GrabState.ERROR
                return False
            print(f"[{self.log_prefix}] ✓ received ok (grip)")

        self.state = GrabState.PULLING
        print(f"[{self.log_prefix}] Winch pull {self.pull_length}mm...")
        resp = self.send_raw(f"pull:{int(self.pull_length)}")
        if not received_ok(resp):
            print(f"[{self.log_prefix}] ✗ No ok after pull: {resp}")
            self.state = GrabState.ERROR
            return False
        print(f"[{self.log_prefix}] ✓ received ok (pull), waiting for ok: done...")
        if not self.wait_for_done(timeout_sec=pull_done_timeout):
            self.state = GrabState.ERROR
            return False

        self.state = GrabState.COMPLETED
        print(f"[{self.log_prefix}] ✓ Grab sequence completed!")
        return True

    def wait_for_done(self, timeout_sec: float = 120.0, poll_interval: float = 1.0) -> bool:
        """
        After a lower or pull command, wait for "ok: done" from the winch (move completed).
        Returns True if "ok: done" received, False on timeout.
        """
        if not self.connected or not self.ws:
            return False
        elapsed = 0.0
        while elapsed < timeout_sec:
            resp = self.recv_with_timeout(poll_interval)
            elapsed += poll_interval
            if resp is not None and "ok: done" in resp.lower():
                print(f"[{self.log_prefix}] ✓ received ok: done")
                return True
        print(f"[{self.log_prefix}] ✗ Timeout waiting for ok: done ({timeout_sec}s)")
        return False

    def stop(self) -> None:
        """Send stop command."""
        self.send_command("stop")


# ---------------------------------------------------------------------------
# Gripper (hold | release | grip | status)
# ---------------------------------------------------------------------------


class GripperController(BaseExternalSystem):
    """Gripper ESP: commands hold, release, grip, status."""

    VALID_CMDS = ("hold", "release", "grip", "status")

    def __init__(
        self,
        ws_url: str = "ws://192.168.42.39:81",
        timeout: float = 5.0,
        log_prefix: str = "GRIPPER",
        recv_timeout: float = 30.0,
    ):
        super().__init__(ws_url, timeout, log_prefix=log_prefix)
        # grip does a servo sweep that can take several seconds; wait long for response
        self.recv_timeout = recv_timeout

    def send_command(self, command: str) -> bool:
        """Send one of: hold, release, grip, status. Returns True if send succeeded."""
        cmd = command.strip().lower()
        if cmd not in self.VALID_CMDS:
            print(f"[{self.log_prefix}] ✗ Unknown command: {command!r}, allowed: {self.VALID_CMDS}")
            return False
        return self.send_raw(cmd) is not None

    def hold(self) -> bool:
        return self.send_command("hold")

    def release(self) -> bool:
        return self.send_command("release")

    def grip(self) -> bool:
        return self.send_command("grip")

    def status(self) -> Optional[str]:
        """Send status command and return response string, or None on failure."""
        return self.send_raw("status")


# ---------------------------------------------------------------------------
# Manager (scalable: add more systems by name)
# ---------------------------------------------------------------------------


class ExternalSystemsManager:
    """Registry of named external systems. Connect/disconnect all, get by name."""

    def __init__(self):
        self._systems: Dict[str, BaseExternalSystem] = {}

    def register(self, name: str, system: BaseExternalSystem) -> None:
        """Register a system under a name. Overwrites if name exists."""
        self._systems[name] = system

    def get(self, name: str) -> Optional[BaseExternalSystem]:
        """Get system by name."""
        return self._systems.get(name)

    def __getitem__(self, name: str) -> BaseExternalSystem:
        """Get system by name; KeyError if missing."""
        if name not in self._systems:
            raise KeyError(f"Unknown external system: {name!r}")
        return self._systems[name]

    def connect_all(self) -> bool:
        """Connect all registered systems. Returns True only if all connected."""
        ok = True
        for name, sys in self._systems.items():
            if not sys.connect():
                ok = False
        return ok

    def disconnect_all(self) -> None:
        """Disconnect all registered systems."""
        for sys in self._systems.values():
            sys.disconnect()

    def names(self):
        """Return registered system names."""
        return list(self._systems.keys())
