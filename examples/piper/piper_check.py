#!/usr/bin/env python

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""PiPER bring-up / diagnostics, independent of LeRobot's robot classes.

    # 1) Is feedback alive? Are control (command) frames visible on this bus?  How far does feedback lag?
    python examples/piper/piper_check.py monitor --port can0 --seconds 10

    # 2) One-time role setup (power-cycle the arm after master -> slave)
    python examples/piper/piper_check.py set-role --port can_leader --role master
    python examples/piper/piper_check.py set-role --port can_follower --role slave

    # 3) Enable, move to the zero/home pose, disable
    python examples/piper/piper_check.py home --port can_follower

`monitor` only listens; it never sends motion commands. Move the leader arm by hand while it runs.
"""

import argparse
import time

import numpy as np

from lerobot.motors.piper import JOINT_NAMES, MOTOR_NAMES, PiperArm


def _fmt(values: dict[str, float]) -> str:
    return " ".join(f"{values[m]:8.2f}" for m in MOTOR_NAMES)


def _estimate_lag(t: np.ndarray, cmd: np.ndarray, fb: np.ndarray, max_lag_s: float = 0.5) -> float | None:
    """Lag (s) that best aligns feedback to command, by minimising mean squared error over joint trajectories."""
    if len(t) < 20 or np.ptp(cmd) < 1.0:
        return None
    dt = float(np.median(np.diff(t)))
    best, best_k = np.inf, 0
    for k in range(0, int(max_lag_s / dt) + 1):
        err = np.mean((cmd[: len(cmd) - k] - fb[k:]) ** 2)
        if err < best:
            best, best_k = err, k
    return best_k * dt


def monitor(port: str, seconds: float, hz: float) -> None:
    arm = PiperArm(port)
    arm.connect(wait_for=None)
    print(f"listening on {port} for {seconds}s  (columns: {' '.join(f'{m:>8}' for m in MOTOR_NAMES)})")

    ts, cmds, fbs = [], [], []
    fb_seen = cmd_seen = False
    t0 = time.monotonic()
    next_print = t0
    try:
        while time.monotonic() - t0 < seconds:
            now = time.monotonic()
            fb = arm.read("feedback")
            cmd = arm.read("command")
            fb_seen |= fb.received
            cmd_seen |= cmd.received
            if fb.received and cmd.received:
                ts.append(now - t0)
                cmds.append([cmd.values[j] for j in JOINT_NAMES])
                fbs.append([fb.values[j] for j in JOINT_NAMES])
            if now >= next_print:
                fb_s = _fmt(fb.values) if fb.received else "   (no feedback frames)"
                cmd_s = _fmt(cmd.values) if cmd.received else "   (no command frames)"
                print(f"t={now - t0:5.1f}s  fb  {fb_s}  age={fb.age_s():.3f}s")
                print(f"           cmd {cmd_s}  age={cmd.age_s():.3f}s")
                next_print = now + 1.0
            time.sleep(1.0 / hz)
    finally:
        arm.disconnect()

    print("\n==== summary ====")
    print(f"feedback frames : {'yes' if fb_seen else 'NO'}")
    print(f"command frames  : {'yes' if cmd_seen else 'NO'}")
    if not cmds:
        print(
            "-> leader commands are NOT visible on this bus. Single-port recording must use "
            "--action_source=follower_state (or record dual-port)."
        )
        return
    t, c, f = np.asarray(ts), np.asarray(cmds), np.asarray(fbs)
    print("per-joint |cmd - fb| mean / max [deg]:")
    for i, j in enumerate(JOINT_NAMES):
        d = np.abs(c[:, i] - f[:, i])
        lag = _estimate_lag(t, c[:, i], f[:, i])
        lag_s = f"{lag * 1000:6.0f} ms" if lag is not None else "   (not moved)"
        print(f"  {j}: {d.mean():6.2f} / {d.max():6.2f}   est. follow lag {lag_s}")


def set_role(port: str, role: str) -> None:
    arm = PiperArm(port)
    arm.connect(wait_for=None)
    arm.set_role(role)
    time.sleep(0.2)
    arm.disconnect()
    print(f"{port}: role set to {role}. Power-cycle the arm if it changed from master to slave.")


def home(port: str, speed: int) -> None:
    arm = PiperArm(port)
    arm.connect(wait_for="feedback")
    try:
        if not arm.enable():
            raise SystemExit("enable failed")
        ok = arm.move_to(dict.fromkeys(MOTOR_NAMES, 0.0), speed_rate=speed)
        print(f"home reached: {ok}  status: {arm.status()}")
        if ok:
            arm.disable()
    finally:
        arm.disconnect()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("monitor")
    m.add_argument("--port", required=True)
    m.add_argument("--seconds", type=float, default=10.0)
    m.add_argument("--hz", type=float, default=200.0)
    r = sub.add_parser("set-role")
    r.add_argument("--port", required=True)
    r.add_argument("--role", choices=["master", "slave"], required=True)
    h = sub.add_parser("home")
    h.add_argument("--port", required=True)
    h.add_argument("--speed", type=int, default=20)
    a = p.parse_args()

    if a.cmd == "monitor":
        monitor(a.port, a.seconds, a.hz)
    elif a.cmd == "set-role":
        set_role(a.port, a.role)
    else:
        home(a.port, a.speed)


if __name__ == "__main__":
    main()
