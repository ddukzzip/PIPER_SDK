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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


def _default_home() -> dict[str, float]:
    # PiPER zero pose (folded rest position): all joints 0 deg, gripper closed.
    return {
        "joint1": 0.0,
        "joint2": 0.0,
        "joint3": 0.0,
        "joint4": 0.0,
        "joint5": 0.0,
        "joint6": 0.0,
        "gripper": 0.0,
    }


@RobotConfig.register_subclass("piper_follower")
@dataclass(kw_only=True)
class PiperFollowerConfig(RobotConfig):
    # SocketCAN interface name, e.g. "can_follower" or "can0"
    port: str
    # piper_sdk: check that the interface is an official AgileX CAN module
    judge_flag: bool = True

    # Passive mode: the PC never sends motion / enable / disable commands. Use it when the follower is driven
    # by a leader arm through a direct CAN link (single-port recording) and the PC only listens.
    passive: bool = False

    # ModeCtrl move_spd_rate_ctrl (0-100 %). Limits joint velocity of every command.
    speed_rate: int = 30
    # GripperCtrl effort in 0.001 N*m (0-5000)
    gripper_effort: int = 1000

    # Max change per send_action call, in degrees (joints) / mm (gripper), measured from the current feedback.
    # Scalar for all motors or a dict keyed by "joint1"... "gripper". None disables the check.
    max_relative_target: float | dict[str, float] | None = None

    enable_on_connect: bool = True
    # Move to `home_position` when connect(calibrate=True) is called.
    home_on_connect: bool = True
    # On disconnect: move to `home_position`, then disable torque.
    home_on_disconnect: bool = True
    disable_torque_on_disconnect: bool = True
    home_position: dict[str, float] = field(default_factory=_default_home)
    home_timeout_s: float = 10.0

    # Feedback older than this (seconds) makes get_observation raise instead of returning stale values.
    max_feedback_age_s: float = 0.5

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
