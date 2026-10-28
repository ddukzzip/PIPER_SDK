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

"""AgileX PiPER leader arm (master / teaching input arm) as a LeRobot teleoperator.

Actions use the same keys and units as `PiperFollower`: joint1.pos ... joint6.pos [deg], gripper.pos [mm],
so a leader action can be sent to the follower 1:1.
"""

import logging
import time
from typing import Any

from lerobot.motors.piper import MOTOR_NAMES, PiperArm
from lerobot.processor import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_piper_leader import PiperLeaderConfig

logger = logging.getLogger(__name__)


class PiperLeader(Teleoperator):
    config_class = PiperLeaderConfig
    name = "piper_leader"

    def __init__(self, config: PiperLeaderConfig):
        super().__init__(config)
        self.config = config
        self.arm = PiperArm(config.port, judge_flag=config.judge_flag)

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in MOTOR_NAMES}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.arm.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        # Fails loudly if the leader never publishes the selected message type, instead of silently
        # returning an all-zero pose that would drive the follower to joint zero.
        self.arm.connect(wait_for=self.config.source, timeout_s=self.config.connect_timeout_s)
        if self.config.enable_on_connect:
            self.arm.enable()
        logger.info(f"{self} connected (source={self.config.source}).")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def setup_motors(self) -> None:
        """One-time: configure this arm as master (teaching input arm)."""
        was_connected = self.arm.is_connected
        if not was_connected:
            self.arm.connect(wait_for=None)
        self.arm.set_role("master")
        logger.info(f"{self}: set as master.")
        if not was_connected:
            self.arm.disconnect()

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        start = time.perf_counter()
        reading = self.arm.read(self.config.source)
        age = reading.age_s()
        limit = self.config.max_message_age_s
        if not reading.received or (limit is not None and age > limit):
            raise TimeoutError(f"{self}: leader {self.config.source} is stale ({age:.2f}s old).")
        logger.debug(f"{self} read action: {(time.perf_counter() - start) * 1e3:.1f}ms")
        return {f"{m}.pos": v for m, v in reading.values.items()}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    @check_if_not_connected
    def disconnect(self) -> None:
        if self.config.enable_on_connect:
            self.arm.disable()
        self.arm.disconnect()
        logger.info(f"{self} disconnected.")
