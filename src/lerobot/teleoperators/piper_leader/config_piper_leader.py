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

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("piper_leader")
@dataclass(kw_only=True)
class PiperLeaderConfig(TeleoperatorConfig):
    # SocketCAN interface name, e.g. "can_leader"
    port: str
    judge_flag: bool = True

    # Which CAN messages carry the human's pose.
    #   "command":  control frames (0x155-0x159) a master (teaching input) arm publishes while being dragged.
    #   "feedback": measured joint feedback, if your firmware publishes it on the master bus instead.
    source: str = "command"

    # Previous WeGo code enabled the leader on connect. Kept as an option: set False if drag teaching feels
    # stiff, True if the leader stops publishing without it.
    enable_on_connect: bool = True

    # Seconds without a new message before get_action raises instead of returning a stale pose.
    # None disables the check (e.g. if your master arm only publishes while it is moving).
    max_message_age_s: float | None = 0.5
    connect_timeout_s: float = 2.0

    def __post_init__(self):
        if self.source not in ("command", "feedback"):
            raise ValueError(f"source must be 'command' or 'feedback', got {self.source!r}")
