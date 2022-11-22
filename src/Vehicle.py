#!/usr/bin/env python
#
# Copyright (c) 2022 Abhishek Bharambe <abhishek.bharambe@sjsu.edu>
# Copyright (c) 2022 Eugene Clewlow <eugene.clewlow@sjsu.edu>
# Copyright (c) 2022 Kanak Kshirsagar <kanak.kshirsagar@sjsu.edu>
# Copyright (c) 2022 Spoorthi Devanand <spoorthi.devanand@sjsu.edu>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

from utils import add_vehicle, set_par, change_lane, communicate, \
    get_distance, get_par, start_sumo, running
from Direction import Direction
from enum import Enum, auto
import traci
from V2V import v2v


def is_platoon_vehicle(vid):
    return vid.startswith(VehicleCounter.ID_PRE_PLATOON)


class Vehicle:
    # vehicle length
    LENGTH = 4
    # inter-vehicle distance
    DISTANCE = 5

    DEFAULT_SLOW_SPEED = 30
    DEFAULT_SLOW_LANE = 0

    CMD_CHANGE_LANE_LEFT = auto()
    CMD_CHANGE_LANE_RIGHT = auto()

    def __init__(self, vid, commands=dict(), v2v=False):
        self.vid = vid
        self.commands = commands
        self.v2v = v2v
        self.vehicle_length = traci.vehicletype.getLength('V2V_Car')
        self.min_gap = traci.vehicletype.getMinGap('V2V_Car')

    def get_lane(self):
        return traci.vehicle.getLaneIndex(self.vid)

    def change_lane(self, direction):
        lane = self.get_lane()
        destination_lane = lane + direction

        change_lane(self.vid, destination_lane)

    def tick(self, step):
        if step in self.commands:
            command = self.commands.get(step)
            if command == self.CMD_CHANGE_LANE_LEFT:
                self.change_lane(Direction.LEFT)
            elif command == self.CMD_CHANGE_LANE_RIGHT:
                self.change_lane(Direction.RIGHT)

    def could_lane_change(self, direction):
        edge_id = traci.vehicle.getRoadID(self.vid)
        lane_count = traci.edge.getLaneNumber(edge_id)
        lane_index = traci.vehicle.getLaneIndex(self.vid)

        if direction == Direction.LEFT and lane_index == lane_count - 1:
            return False
        if direction == Direction.RIGHT and lane_index == 0:
            return False

        if direction == Direction.LEFT:
            leaders = traci.vehicle.getLeftLeaders(self.vid)
            followers = traci.vehicle.getLeftFollowers(self.vid)
        if direction == Direction.RIGHT:
            leaders = traci.vehicle.getRightLeaders(self.vid)
            followers = traci.vehicle.getRightFollowers(self.vid)

        for l in leaders:
            _, dist = l
            if dist <= self.vehicle_length:
                return False
        for f in followers:
            _, dist = f
            if dist <= self.vehicle_length:
                return False
        return True

    def receive_v2v_request(self, sender_id, request_type):
        if request_type == v2v.V2V_LANE_CHANGE_MANEUVER_REQUEST:
            if self.could_lane_change(Direction.LEFT):
                self.change_lane(Direction.LEFT)
            elif self.could_lane_change(Direction.RIGHT):
                self.change_lane(Direction.RIGHT)


class VehicleCounter:
    i = 0
    ID_PRE = 'v.'
    ID_PRE_PLATOON = 'platoon.'

    def __init__(self):
        i = 0

    def get_next_vehicle_id(self):
        vid = f"{self.ID_PRE}{self.i}"
        self.i += 1
        return vid

    def get_next_platoon_vehicle_id(self):
        vid = f"{self.ID_PRE_PLATOON}{self.i}"
        self.i += 1
        return vid

    def reset(self):
        self.i = 0


vehicle_counter = VehicleCounter()
