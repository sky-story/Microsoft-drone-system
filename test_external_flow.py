#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test the full external system flow: release → lower → grip → pull.

Runs: gripper release (wait ok) → winch lower 400mm (ok, then ok: done) →
      gripper grip (wait ok) → winch pull 400mm (ok, then ok: done).

Usage:
  python test_external_flow.py

  # Custom URLs (e.g. same host, different ports)
  python test_external_flow.py --winch-url ws://172.20.10.3:81 --gripper-url ws://172.20.10.3:82

  # Custom lengths (default 400mm each)
  python test_external_flow.py --lower-mm 300 --pull-mm 200

  # Shorter timeout for ok: done (default 120s)
  python test_external_flow.py --done-timeout 60
"""

import argparse
import sys

from external_systems import GripperController, WinchController


def main():
    p = argparse.ArgumentParser(
        description="Test external flow: release → lower → grip → pull (wait ok / ok: done each step)"
    )
    p.add_argument(
        "--winch-url",
        default="ws://192.168.42.15",
        help="Winch WebSocket URL",
    )
    p.add_argument(
        "--gripper-url",
        default="ws://192.168.42.39:81",
        help="Gripper WebSocket URL",
    )
    p.add_argument(
        "--lower-mm",
        type=float,
        default=400.0,
        help="Lower distance in mm (default: 400)",
    )
    p.add_argument(
        "--pull-mm",
        type=float,
        default=400.0,
        help="Pull distance in mm (default: 400)",
    )
    p.add_argument(
        "--done-timeout",
        type=float,
        default=120.0,
        help="Seconds to wait for ok: done from winch (default: 120)",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="WebSocket connection timeout in seconds",
    )
    args = p.parse_args()

    gripper = GripperController(ws_url=args.gripper_url, timeout=args.timeout)
    winch = WinchController(
        ws_url=args.winch_url,
        timeout=args.timeout,
        lower_length=args.lower_mm,
        pull_length=args.pull_mm,
    )

    print("=" * 60)
    print("  External system flow test")
    print("  release → lower {}mm → grip → pull {}mm".format(args.lower_mm, args.pull_mm))
    print("=" * 60)
    print("  Gripper: {}".format(args.gripper_url))
    print("  Winch:   {}".format(args.winch_url))
    print("  Done timeout: {}s".format(args.done_timeout))
    print("=" * 60)
    print()

    ok = winch.execute_grab_sequence(
        gripper=gripper,
        lower_done_timeout=args.done_timeout,
        pull_done_timeout=args.done_timeout,
    )

    winch.disconnect()
    gripper.disconnect()

    if ok:
        print()
        print("Flow test PASSED.")
        return 0
    print()
    print("Flow test FAILED.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
