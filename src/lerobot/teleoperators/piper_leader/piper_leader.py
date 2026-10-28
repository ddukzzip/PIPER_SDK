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
from pathlib import Path
from typing import Any

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from ...motors.piper import PiperMotorsBus

from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from ..teleoperator import Teleoperator
from .config_piper_leader import PiperLeaderConfig


_PIPER_CALIBRATION = {
    "joint1.pos": (-150000.0, 150000.0),
    "joint2.pos": (0.0, 180000.0),
    "joint3.pos": (-170000.0, 0.0),
    "joint4.pos": (-100000.0, 100000.0),
    "joint5.pos": (-65000.0, 65000.0),
    "joint6.pos": (-100000.0, 130000.0),
    "gripper.pos": (0.0, 100000.0),
}


logger = logging.getLogger(__name__)

class PiperLeader(Teleoperator):
    config_class = PiperLeaderConfig
    name = "piper_leader"

    def __init__(self, config: PiperLeaderConfig):
        super().__init__(config)
        self.config = config
        self.id = config.id
        self.port = config.port
        self.bus = PiperMotorsBus(
            id=config.id,
            port=config.port,
            motors={
                "joint1": Motor(1, "AGILEX-M", MotorNormMode.RANGE_M100_100),
                "joint2": Motor(2, "AGILEX-M", MotorNormMode.RANGE_M100_100),
                "joint3": Motor(3, "AGILEX-M", MotorNormMode.RANGE_M100_100),
                "joint4": Motor(4, "AGILEX-S", MotorNormMode.RANGE_M100_100),
                "joint5": Motor(5, "AGILEX-S", MotorNormMode.RANGE_M100_100),
                "joint6": Motor(6, "AGILEX-S", MotorNormMode.RANGE_M100_100),
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

    def __str__(self) -> str:
        return f"{self.id} {self.__class__.__name__}"

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()

        # self.configure()
        # logger.info(f"{self} connected.")
        self.bus.piper.EnableFkCal()   
        self.bus.enable_torque()
        logger.info(f"{self} torque on.")
        print('leader connected, enabling torque...')
            
        
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def _load_calibration(self, fpath: Path | None = None) -> None:
        pass

    def _save_calibration(self, fpath: Path | None = None) -> None:
        pass

    def configure(self) -> None:
        pass

    def setup_motors(self) -> None:
        self.bus.connect()
        self.bus.set_master()

    @check_if_not_connected
    def get_action(self) -> dict[str, Any]:
        start = time.perf_counter()
        action = self.bus.get_control()
        action = {f"{motor}.pos": val for motor, val in action.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disable_torque()
        self.bus.disconnect()
        logger.info(f"{self} disconnected.")

    def get_status(self):
        rlt = {
            print(self.bus.piper.GetArmGripperMsgs()),
            print(self.bus.piper.GetArmGripperCtrl()),
        }
        return rlt
    
    def get_fk(self, mode: str = "control"):
        return self.bus.piper.GetFK(mode)
