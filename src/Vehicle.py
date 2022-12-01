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

from enum import auto

import traci

from Direction import Direction
from V2V import v2v
from utils import change_lane


def is_platoon_vehicle(vid):
    """
    Return whether a vehicle is a platoon vehicle or not

    :param vid: the target vehicle's traci vehicle id
    """
    return vid.startswith(VehicleCounter.ID_PRE_PLATOON)


class Vehicle:
    """
    Vehicle class encapsulating vehicle functionality
    """
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
        """
        Get the current traveling lane of this vehicle
        """
        return traci.vehicle.getLaneIndex(self.vid)

    def change_lane(self, direction):
        """
        Change the traveling lane of this vehicle

        :param direction: the direction in which to change lanes
        """
        lane = self.get_lane()
        destination_lane = lane + direction

        change_lane(self.vid, destination_lane)

    def tick(self, step):
        """
        The simulation step function for this vehicle

        :param step: current simulation step
        """
        if step in self.commands:
            command = self.commands.get(step)
            if command == self.CMD_CHANGE_LANE_LEFT:
                self.change_lane(Direction.LEFT)
            elif command == self.CMD_CHANGE_LANE_RIGHT:
                self.change_lane(Direction.RIGHT)

    def could_lane_change(self, direction):
        """
        Returns whether there is adequate space to make a lane change in the given direction

        :param direction: the direction to check for lane change availability
        """
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
        """
        Process a V2V request

        :param sender_id: the origin of the V2V message
        :param request_type: the type of the V2V message
        """
        if request_type == v2v.V2V_LANE_CHANGE_MANEUVER_REQUEST:
            if self.could_lane_change(Direction.LEFT):
                self.change_lane(Direction.LEFT)
            elif self.could_lane_change(Direction.RIGHT):
                self.change_lane(Direction.RIGHT)


class VehicleCounter:
    """
    A class which provides a unique name creation system for traci vehicles
    """
    i = 0
    ID_PRE = 'v.'
    ID_PRE_PLATOON = 'platoon.'

    def __init__(self):
        i = 0

    def get_next_vehicle_id(self):
        """
        Generate the next unique id for a vehicle
        """
        vid = f"{self.ID_PRE}{self.i}"
        self.i += 1
        return vid

    def get_next_platoon_vehicle_id(self):
        """
        Generate the next unique id for a platoon vehicle
        """
        vid = f"{self.ID_PRE_PLATOON}{self.i}"
        self.i += 1
        return vid

    def reset(self):
        """
        Reset the vehicle counter
        """
        self.i = 0


vehicle_counter = VehicleCounter()
