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

"""AgileX PiPER follower arm.

Implements the LeRobot `Robot` interface directly on top of piper_sdk (through the thin `PiperArm`
helper). There is no MotorsBus layer and no -100..100 normalisation:

    observation / action keys : joint1.pos ... joint6.pos  [deg],  gripper.pos  [mm]

Policies normalise with dataset statistics, so physical units can be fed to them directly.
"""

import logging
import time
from functools import cached_property

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors.piper import MOTOR_NAMES, ArmReading, PiperArm
from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_piper_follower import PiperFollowerConfig

logger = logging.getLogger(__name__)


class PiperFollower(Robot):
    config_class = PiperFollowerConfig
    name = "piper_follower"

    def __init__(self, config: PiperFollowerConfig):
        super().__init__(config)
        self.config = config
        self.arm = PiperArm(config.port, judge_flag=config.judge_flag)
        self.cameras = make_cameras_from_configs(config.cameras)

    # ------------------------------------------------------------------ features
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in MOTOR_NAMES}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    # ------------------------------------------------------------------ connection
    @property
    def is_connected(self) -> bool:
        return self.arm.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.arm.connect(wait_for="feedback")

        if not self.config.passive:
            if self.config.enable_on_connect and not self.arm.enable():
                self.arm.disconnect()
                raise ConnectionError(f"{self}: failed to enable motors.")
            if calibrate and self.config.home_on_connect:
                logger.info(f"{self}: moving to home position.")
                self.arm.move_to(
                    self.config.home_position,
                    self.config.speed_rate,
                    self.config.gripper_effort,
                    timeout_s=self.config.home_timeout_s,
                )

        for cam in self.cameras.values():
            cam.connect()
        logger.info(f"{self} connected (passive={self.config.passive}).")

    @property
    def is_calibrated(self) -> bool:
        # Joint zero is stored in the PiPER controller; nothing to calibrate from LeRobot.
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def setup_motors(self) -> None:
        """One-time: configure this arm as slave (motion output arm)."""
        was_connected = self.arm.is_connected
        if not was_connected:
            self.arm.connect(wait_for=None)
        self.arm.set_role("slave")
        logger.info(f"{self}: set as slave. Power-cycle the arm if it was previously a master.")
        if not was_connected:
            self.arm.disconnect()

    # ------------------------------------------------------------------ io
    def _fresh(self, reading: ArmReading, what: str) -> ArmReading:
        age = reading.age_s()
        if not reading.received or age > self.config.max_feedback_age_s:
            raise TimeoutError(f"{self}: {what} is stale ({age:.2f}s old). Is the CAN link alive?")
        return reading

    def read_state(self) -> dict[str, float]:
        """Measured joint/gripper state in deg/mm, without `.pos` suffix."""
        return self._fresh(self.arm.read("feedback"), "feedback").values

    def read_command(self) -> ArmReading:
        """Control frames seen on this CAN bus (deg/mm). In single-port wired mode these are the leader's
        commands to the follower; check `.received` / `.age_s()` before using them."""
        return self.arm.read("command")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()
        obs: RobotObservation = {f"{m}.pos": v for m, v in self.read_state().items()}
        logger.debug(f"{self} read state: {(time.perf_counter() - start) * 1e3:.1f}ms")

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs[cam_key] = cam.async_read()
            logger.debug(f"{self} read {cam_key}: {(time.perf_counter() - start) * 1e3:.1f}ms")
        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        if self.config.passive:
            raise RuntimeError(f"{self} is passive (driven by a wired leader); it does not accept actions.")

        goal = {
            k.removesuffix(".pos"): float(v)
            for k, v in action.items()
            if k.removesuffix(".pos") in MOTOR_NAMES
        }

        if self.config.max_relative_target is not None:
            present = self.read_state()
            goal = ensure_safe_goal_position(
                {k: (g, present[k]) for k, g in goal.items()}, self.config.max_relative_target
            )

        sent = self.arm.send(goal, self.config.speed_rate, self.config.gripper_effort)
        return {f"{m}.pos": v for m, v in sent.items()}

    def move_home(self) -> bool:
        return self.arm.move_to(
            self.config.home_position,
            self.config.speed_rate,
            self.config.gripper_effort,
            timeout_s=self.config.home_timeout_s,
        )

    @check_if_not_connected
    def disconnect(self) -> None:
        try:
            if not self.config.passive:
                if self.config.home_on_disconnect and not self.move_home():
                    logger.warning(
                        f"{self}: did not reach home; keeping torque ON to avoid dropping the arm."
                    )
                elif self.config.disable_torque_on_disconnect:
                    self.arm.disable()
        finally:
            for cam in self.cameras.values():
                if cam.is_connected:
                    cam.disconnect()
            self.arm.disconnect()
            logger.info(f"{self} disconnected.")
