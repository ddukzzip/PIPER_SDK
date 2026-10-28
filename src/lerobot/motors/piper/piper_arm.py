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

"""Thin helper around ``piper_sdk.C_PiperInterface_V2``.

This is NOT a LeRobot ``MotorsBus``. The PiPER controller already exposes an arm-level API (6 joints +
gripper in one call), so the only things this module adds are:

* explicit unit conversion (SDK integers <-> degrees / millimetres),
* joint / gripper range clipping before a command is sent,
* a connect step that waits until real CAN feedback arrives (the SDK returns all-zero messages until the
  first frame is received, which would otherwise look like a valid "all joints at 0" reading),
* small blocking helpers (enable, disable, move_to) with timeouts.

Units used everywhere outside this file:
    joints  -> degrees (float)
    gripper -> millimetres of opening (float)
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Literal

logger = logging.getLogger(__name__)

JOINT_NAMES: tuple[str, ...] = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_NAME = "gripper"
MOTOR_NAMES: tuple[str, ...] = (*JOINT_NAMES, GRIPPER_NAME)

# Defaults from piper_sdk (C_PiperParamManager / JointCtrl docstring), in degrees.
JOINT_LIMITS_DEG: dict[str, tuple[float, float]] = {
    "joint1": (-150.0, 150.0),
    "joint2": (0.0, 180.0),
    "joint3": (-170.0, 0.0),
    "joint4": (-100.0, 100.0),
    "joint5": (-70.0, 70.0),
    "joint6": (-120.0, 120.0),
}
# piper_sdk default gripper range is 0.0 - 0.07 m.
GRIPPER_RANGE_MM: tuple[float, float] = (0.0, 70.0)

# SDK integer units: joints in 0.001 deg, gripper in 0.001 mm.
_SDK_PER_DEG = 1000.0
_SDK_PER_MM = 1000.0

# ModeCtrl constants
_CTRL_MODE_CAN = 0x01
_MOVE_J = 0x01
_POS_VEL_MODE = 0x00
# GripperCtrl status codes
_GRIPPER_ENABLE = 0x01
# MasterSlaveConfig linkage codes
_ROLE_CODES = {"master": 0xFA, "slave": 0xFC}

Source = Literal["feedback", "command"]


@dataclass
class ArmReading:
    """One reading of the 6 joints + gripper, already converted to degrees / mm."""

    values: dict[str, float]
    joint_stamp: float  # SDK receive time (time.time() based) of the joint message, 0.0 if never received
    gripper_stamp: float

    @property
    def received(self) -> bool:
        return self.joint_stamp > 0.0 and self.gripper_stamp > 0.0

    def age_s(self, now: float | None = None) -> float:
        now = time.time() if now is None else now
        return now - min(self.joint_stamp, self.gripper_stamp)


class PiperArm:
    def __init__(
        self,
        port: str,
        *,
        judge_flag: bool = True,
        sdk_joint_limit: bool = True,
        sdk_gripper_limit: bool = True,
    ):
        self.port = port
        self.judge_flag = judge_flag
        self.sdk_joint_limit = sdk_joint_limit
        self.sdk_gripper_limit = sdk_gripper_limit
        self.sdk = None
        self._connected = False

    def __repr__(self) -> str:
        return f"PiperArm(port={self.port!r}, connected={self._connected})"

    # ------------------------------------------------------------------ connection
    @property
    def is_connected(self) -> bool:
        return self._connected and self.sdk is not None and bool(self.sdk.get_connect_status())

    def connect(self, wait_for: Source | None = "feedback", timeout_s: float = 2.0) -> None:
        """Open the CAN port and start the SDK read thread.

        Args:
            wait_for: block until this kind of message has been received at least once.
                "feedback" for an arm we read joint states from, "command" for a master arm whose
                pose is only published as control frames, None to skip the check.
            timeout_s: how long to wait for that first message.
        """
        from piper_sdk import C_PiperInterface_V2

        self.sdk = C_PiperInterface_V2(
            can_name=self.port,
            judge_flag=self.judge_flag,
            can_auto_init=True,
            start_sdk_joint_limit=self.sdk_joint_limit,
            start_sdk_gripper_limit=self.sdk_gripper_limit,
        )
        self.sdk.ConnectPort()
        self._connected = True

        if wait_for is None:
            return
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.read(wait_for).received:
                return
            time.sleep(0.01)

        self.disconnect()
        hint = (
            "Check `ip -br link show type can` (interface up, bitrate 1000000) and the arm power."
            if wait_for == "feedback"
            else "No control frames on this bus. Is the arm configured as master (teaching input arm)?"
        )
        raise ConnectionError(f"No {wait_for} message received on '{self.port}' within {timeout_s}s. {hint}")

    def disconnect(self) -> None:
        if self.sdk is not None and self._connected:
            self.sdk.DisconnectPort()
        self._connected = False

    # ------------------------------------------------------------------ torque
    def enable(self, timeout_s: float = 5.0) -> bool:
        """Enable all motors (joints + gripper). Returns True once every joint driver reports enabled."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.sdk.EnablePiper():  # True only when all 6 drivers already report enabled
                return True
            time.sleep(0.05)
        logger.warning(f"{self.port}: enable timed out, status={self.sdk.GetArmEnableStatus()}")
        return False

    def disable(self, timeout_s: float = 2.0) -> bool:
        """Disable all motors. The arm will fall under gravity unless it is resting."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if not self.sdk.DisablePiper():  # returns True while any driver is still enabled
                return True
            time.sleep(0.05)
        logger.warning(f"{self.port}: disable timed out, status={self.sdk.GetArmEnableStatus()}")
        return False

    # ------------------------------------------------------------------ read
    def read(self, source: Source = "feedback") -> ArmReading:
        """Read joints + gripper.

        source="feedback": measured state (0x2A5-0x2A8 feedback frames).
        source="command":  joint / gripper control frames seen on the bus (0x155-0x157, 0x159). On a master
                           arm these carry the pose the human is dragging the arm to.
        """
        if source == "feedback":
            jm = self.sdk.GetArmJointMsgs()
            gm = self.sdk.GetArmGripperMsgs()
            j, g = jm.joint_state, gm.gripper_state
        elif source == "command":
            jm = self.sdk.GetArmJointCtrl()
            gm = self.sdk.GetArmGripperCtrl()
            j, g = jm.joint_ctrl, gm.gripper_ctrl
        else:
            raise ValueError(f"Unknown source {source!r}")

        raw = (j.joint_1, j.joint_2, j.joint_3, j.joint_4, j.joint_5, j.joint_6)
        values = {name: v / _SDK_PER_DEG for name, v in zip(JOINT_NAMES, raw, strict=True)}
        values[GRIPPER_NAME] = g.grippers_angle / _SDK_PER_MM
        return ArmReading(values=values, joint_stamp=float(jm.time_stamp), gripper_stamp=float(gm.time_stamp))

    def status(self):
        """Raw SDK arm status (ctrl_mode, arm_status, motion_status, ...)."""
        return self.sdk.GetArmStatus().arm_status

    # ------------------------------------------------------------------ write
    @staticmethod
    def clip(values: dict[str, float]) -> dict[str, float]:
        out = {}
        for name, v in values.items():
            lo, hi = JOINT_LIMITS_DEG.get(name, GRIPPER_RANGE_MM)
            out[name] = min(max(float(v), lo), hi)
        return out

    def send(
        self, targets: dict[str, float], speed_rate: int = 30, gripper_effort: int = 1000
    ) -> dict[str, float]:
        """Send a joint-space target (degrees / mm). Joints missing from `targets` keep their current
        measured position; the gripper is only commanded if present. Returns the clipped values sent."""
        unknown = set(targets) - set(MOTOR_NAMES)
        if unknown:
            raise KeyError(f"Unknown motor names: {sorted(unknown)}")

        if not all(name in targets for name in JOINT_NAMES):
            current = self.read("feedback").values
            targets = {**{n: current[n] for n in JOINT_NAMES}, **targets}
        sent = self.clip(targets)

        self.sdk.ModeCtrl(_CTRL_MODE_CAN, _MOVE_J, int(speed_rate), _POS_VEL_MODE)
        self.sdk.JointCtrl(*(round(sent[n] * _SDK_PER_DEG) for n in JOINT_NAMES))
        if GRIPPER_NAME in sent:
            self.sdk.GripperCtrl(
                round(sent[GRIPPER_NAME] * _SDK_PER_MM), int(gripper_effort), _GRIPPER_ENABLE, 0
            )
        return sent

    def move_to(
        self,
        targets: dict[str, float],
        speed_rate: int = 30,
        gripper_effort: int = 1000,
        timeout_s: float = 10.0,
        tol_deg: float = 1.0,
        tol_mm: float = 2.0,
    ) -> bool:
        """Blocking move. Re-sends the target every 20 ms until every motor is within tolerance
        (judged from feedback, not from a motion flag) or the timeout expires."""
        deadline = time.monotonic() + timeout_s
        sent = self.clip(targets)
        while time.monotonic() < deadline:
            self.send(sent, speed_rate, gripper_effort)
            time.sleep(0.02)
            cur = self.read("feedback").values
            if all(abs(cur[n] - v) <= (tol_mm if n == GRIPPER_NAME else tol_deg) for n, v in sent.items()):
                return True
        logger.warning(f"{self.port}: move_to timed out after {timeout_s}s")
        return False

    # ------------------------------------------------------------------ config
    def set_role(self, role: Literal["master", "slave"]) -> None:
        """Configure master (teaching input) / slave (motion output) linkage.
        The arm must be power-cycled for a master -> slave change to take effect (piper_sdk docs)."""
        self.sdk.MasterSlaveConfig(_ROLE_CODES[role], 0, 0, 0)

    def fk(self, source: Source = "feedback") -> list:
        """Forward kinematics per link (mm / deg). Requires `sdk.EnableFkCal()` beforehand."""
        return self.sdk.GetFK("feedback" if source == "feedback" else "control")
