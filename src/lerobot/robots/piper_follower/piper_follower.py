#!/usr/bin/env python

# Copyright 2025 WeGo-Robotics Inc. EDU team. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.piper import PiperMotorsBus
from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_piper_follower import PiperFollowerConfig

logger = logging.getLogger(__name__)

class PiperFollower(Robot):

    # Set these in ALL subclasses

    config_class: PiperFollowerConfig
    name = "piper_follower"

    # cameras : dict[str, Camera] = {}


    def __init__(self, config: PiperFollowerConfig):
        super().__init__(config)
        self.config = config
        self.id = config.id
        self.port = config.port
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = PiperMotorsBus(
            id=config.id,
            port=self.config.port,
            motors={
                "joint1": Motor(1, "AGILEX-M", norm_mode_body),
                "joint2": Motor(2, "AGILEX-M", norm_mode_body),
                "joint3": Motor(3, "AGILEX-M", norm_mode_body),
                "joint4": Motor(4, "AGILEX-S", norm_mode_body),
                "joint5": Motor(5, "AGILEX-S", norm_mode_body),
                "joint6": Motor(6, "AGILEX-S", norm_mode_body),
                "gripper": Motor(7, "AGILEX-S", MotorNormMode.RANGE_0_100),
            },
            calibration={
                "joint1": MotorCalibration(1, 0, 0, -150000, 150000),
                "joint2": MotorCalibration(2, 0, 0,       0, 180000),
                "joint3": MotorCalibration(3, 0, 0, -170000, 0     ),
                "joint4": MotorCalibration(4, 0, 0, -100000, 100000),
                "joint5": MotorCalibration(5, 0, 0,  -65000, 65000 ),
                "joint6": MotorCalibration(6, 0, 0, -100000, 130000),
                "gripper": MotorCalibration(7, 0, 0, 0, 100000),
            }
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    def __str__(self) -> str:
        return f"{self.id} {self.__class__.__name__}"

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
       return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())
        # return self.bus.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """

        self.bus.connect()

        # if not self.bus.enable_torque():
        #     logger.info(f"{self} retry torque on.")
        
        if calibrate:
            logger.info(f"{self} go to origin.")
            self.bus.parking()

        for cam in self.cameras.values():
            cam.connect()

        logger.info(f"{self} connected.")
        print('follower connected, enabling torque...')

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        self.bus.clear_gripper()
        return True

    def _load_calibration(self, fpath: Path | None = None) -> None:
        pass

    def _save_calibration(self, fpath: Path | None = None) -> None:
        pass

    def configure(self) -> None:
        pass

    def setup_motors(self) -> None:
        self.bus.connect()
        self.bus.set_slave()

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:

        obs_dict = {} 

        # Read arm position
        start = time.perf_counter()
        obs_dict = self.bus.get_action()
        obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:

        goal_pos = {}
        for key, val in action.items():
            if key.endswith(".pos"):
                goal_pos[key.removesuffix(".pos")] = val
            else:
                goal_pos[key] = val

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        self.bus.set_action(goal_pos, is_conv=True)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def parking(self):
        self.bus.parking()

    @check_if_not_connected
    def disconnect(self, disable_torque: bool = False) -> None:
        self.bus.disconnect(disable_torque)


    def get_status(self):
        rlt = {
            print(self.bus.piper.GetArmGripperMsgs()),
            print(self.bus.piper.GetArmGripperCtrl()),
        }
        return rlt

    def get_fk(self, mode: str = "feedback"):
        return self.bus.piper.GetFK(mode)