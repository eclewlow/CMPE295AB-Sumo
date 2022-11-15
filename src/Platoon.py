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

import ccparams as cc
from utils import add_vehicle, set_par, change_lane, communicate, \
    get_distance, get_par, start_sumo, running

from Vehicle import vehicle_counter, is_platoon_vehicle, Vehicle
from Direction import Direction

import traci
from PlatoonManager import platoon_manager
from enum import Enum, auto


class Platoon:
    DEFAULT_LANE = 2
    # minimum platoon length for split - lane change maneuver
    M = 3
    # cruising speed
    SPEED = 130 / 3.6

    STATE_CRUISING = auto()
    STATE_OVERTAKING = auto()

    def get_length(self):
        return len(self.vehicles)

    def change_lane(self, direction):
        lane = self.get_lane()
        destination_lane = lane + direction

        for vid in self.vehicles:
            change_lane(vid, destination_lane)

    def get_lane(self):
        return traci.vehicle.getLaneIndex(self.vehicles[0])

    def set_desired_speed(self, speed):
        self.desired_speed = speed
        set_par(self.vehicles[0], cc.PAR_CC_DESIRED_SPEED, speed)
        set_par(self.vehicles[0], cc.PAR_ACTIVE_CONTROLLER, cc.ACC)

    def get_leader(self, radar_front_distance=160):
        vehicle = traci.vehicle.getLeader(self.vehicles[0], radar_front_distance)
        if vehicle is not None:
            # simulate real radar distance
            if vehicle[1] <= radar_front_distance:
                return vehicle[0], vehicle[1]
        return None, None

    def set_leader(self, leader):
        self.leader = leader
        if leader is None:
            self.set_desired_speed(self.desired_speed)
        else:
            set_par(self.vehicles[0], cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)

    def communicate(self):
        for i, vid in enumerate(self.vehicles):
            if i == 0:
                if self.leader is None:
                    continue
                leader = self.leader
                front = self.leader
            else:
                leader = self.vehicles[0]
                front = self.vehicles[i - 1]

            # get data about platoon leader
            leader_data = get_par(leader, cc.PAR_SPEED_AND_ACCELERATION)
            (l_v, l_a, l_u, l_x, l_y, l_t, _, _, _) = cc.unpack(leader_data)
            leader_data = cc.pack(l_v, l_u, l_x, l_y, l_t)
            # get data about front vehicle
            front_data = get_par(front, cc.PAR_SPEED_AND_ACCELERATION)
            (f_v, f_a, f_u, f_x, f_y, f_t, _, _, _) = cc.unpack(front_data)
            front_data = cc.pack(f_v, f_u, f_x, f_y, f_t)
            # pass leader and front vehicle data to CACC
            set_par(vid, cc.PAR_LEADER_SPEED_AND_ACCELERATION, leader_data)
            set_par(vid, cc.PAR_PRECEDING_SPEED_AND_ACCELERATION, front_data)
            # compute GPS distance and pass it to the fake CACC
            f_d = get_distance(vid, front)
            set_par(vid, cc.PAR_LEADER_FAKE_DATA, cc.pack(l_v, l_u))
            set_par(vid, cc.PAR_FRONT_FAKE_DATA, cc.pack(f_v, f_u, f_d))

    def set_state(self, state):
        self.state = state

    def tick(self):
        # update cacc values
        self.communicate()

        # check for leader vehicles
        leader, distance = self.get_leader()

        if leader is None:
            self.set_leader(None)

        if self.state == self.STATE_CRUISING:
            # get lane position and move towards default lane
            pass

        if self.state == self.STATE_OVERTAKING:
            # keep checking left lane for chance to change back
            index = self.get_lane_change_split_index(Direction.LEFT)
            if self.get_length() == index and not self.vehicle_to_overtake_exists(Direction.LEFT):
                # the left lane can fit the whole platoon, so make the left lane change
                self.change_lane(Direction.LEFT)
                self.set_state(self.STATE_CRUISING)

        if leader is not None and not is_platoon_vehicle(leader):
            # approach vehicle
            self.set_leader(leader)

            # if we have approached the leading vehicle
            if distance < Vehicle.LENGTH + Vehicle.DISTANCE:

                # check lane change availability
                index = self.get_lane_change_split_index(Direction.RIGHT)

                if self.get_length() == index:
                    # no split required, just do lane change
                    self.change_lane(Direction.RIGHT)
                    self.set_state(self.STATE_OVERTAKING)
                    pass
                else:
                    # check if the split index
                    if index >= self.M:
                        # front platoon after split meets M requirement, so split
                        rear_platoon = self.split(index)
                        platoon_manager.add_platoon(rear_platoon)

                        self.change_lane(Direction.RIGHT)
                        self.set_state(self.STATE_OVERTAKING)

    def could_lane_change(self, vid, direction):
        if direction == Direction.LEFT:
            leaders = traci.vehicle.getLeftLeaders(vid)
            followers = traci.vehicle.getLeftFollowers(vid)
        if direction == Direction.RIGHT:
            leaders = traci.vehicle.getRightLeaders(vid)
            followers = traci.vehicle.getRightFollowers(vid)
        for l in leaders:
            _, dist = l
            if dist <= Vehicle.LENGTH:
                return False
        for f in followers:
            _, dist = f
            if dist <= Vehicle.LENGTH:
                return False
        return True

    def vehicle_to_overtake_exists(self, direction):
        if direction == Direction.LEFT:
            leaders = traci.vehicle.getLeftLeaders(self.vehicles[0])
        if direction == Direction.RIGHT:
            leaders = traci.vehicle.getRightLeaders(self.vehicles[0])
        for l in leaders:
            _, dist = l
            if dist <= Vehicle.LENGTH + Vehicle.DISTANCE:
                return True
        return False

    def get_lane_change_split_index(self, direction):
        for i, vid in enumerate(self.vehicles):
            if not self.could_lane_change(vid, direction):
                return i
        return len(self.vehicles)

    def split(self, i):
        rear_vehicles = self.vehicles[i:]
        front_vehicles = self.vehicles[:i]

        self.vehicles = front_vehicles

        return Platoon(speed=self.desired_speed, vehicles=rear_vehicles)

    def build(self, n=6, pos=0, speed=SPEED, lane=DEFAULT_LANE):
        for i in range(n):
            vid = vehicle_counter.get_next_platoon_vehicle_id()
            self.vehicles.append(vid)
            add_vehicle(vid, pos - i * (Vehicle.DISTANCE + Vehicle.LENGTH), 0, speed, Vehicle.DISTANCE)
            change_lane(vid, lane)
            if i == 0:
                set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                set_par(vid, cc.PAR_CC_DESIRED_SPEED, speed)
            else:
                set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                set_par(vid, cc.PAR_CC_DESIRED_SPEED, speed)

    def __init__(self, *args, **kwargs):
        self.leader = None
        self.vehicles = kwargs.get("vehicles", list())
        self.desired_speed = kwargs.get("speed", 0)
        self.state = self.STATE_CRUISING

        # this is not a split platoon. it is a new platoon from scratch
        if "vehicles" not in kwargs:
            self.build(*args, **kwargs)
