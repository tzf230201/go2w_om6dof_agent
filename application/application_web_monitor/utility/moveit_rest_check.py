#!/usr/bin/env python3
"""Read a /joint_states YAML message from stdin and evaluate MoveIt home pose."""
import math
import re
import sys

HOME_POSE = {
    "joint1": 0.0,
    "joint2": -0.6806,
    "joint3": 1.3613,
    "joint4": 0.0,
    "joint5": 0.8901,
    "joint6": 0.0,
}
TOLERANCE_RAD = 0.25


def yaml_list(text: str, key: str) -> list[str]:
    values: list[str] = []
    reading = False
    for line in text.splitlines():
        if line == f"{key}:":
            reading = True
            continue
        if not reading:
            continue
        match = re.fullmatch(r"\s*-\s*(.+)", line)
        if match:
            values.append(match.group(1))
        elif line and not line.startswith((" ", "-")):
            break
    return values


text = sys.stdin.read()
names = yaml_list(text, "name")
raw_positions = yaml_list(text, "position")
try:
    positions = [float(value) for value in raw_positions]
except ValueError:
    positions = []

actual = dict(zip(names, positions))
if any(name not in actual or not math.isfinite(actual[name]) for name in HOME_POSE):
    print("UNKNOWN: no complete /joint_states feedback")
    raise SystemExit(0)

largest_error = max(abs(actual[name] - target) for name, target in HOME_POSE.items())
if largest_error <= TOLERANCE_RAD:
    print(f"YES: max error {largest_error:.3f} rad (limit {TOLERANCE_RAD:.2f} rad)")
else:
    print(f"NO: max error {largest_error:.3f} rad (limit {TOLERANCE_RAD:.2f} rad)")
