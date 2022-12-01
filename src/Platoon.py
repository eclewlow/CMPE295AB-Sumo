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

import math
from enum import Enum, auto

import traci

import ccparams as cc
from Direction import Direction
from PlatoonManager import platoon_manager
from V2V import v2v
from Vehicle import vehicle_counter, is_platoon_vehicle
from utils import add_vehicle, set_par, change_lane, get_distance, get_par


class PlatoonState(Enum):
    """
    Class for Platoon state
    """
    STATE_CRUISING = auto()
    STATE_OVERTAKING_RIGHT = auto()
    STATE_OVERTAKING_LEFT = auto()
    STATE_REQUEST_LEADER_LANE_CHANGE = auto()
    STATE_REQUEST_LEFT_VEHICLES_LANE_CHANGE = auto()
    STATE_REQUEST_RIGHT_VEHICLES_LANE_CHANGE = auto()


class Platoon():
    """
    Class which encapsulates the functionality of a Platoon
    """
    DEFAULT_LANE = 2
    # minimum platoon length for split - lane change maneuver
    M = 1
    # cruising speed
    SPEED = 130 / 3.6

    def get_length(self):
        """
        Get the number of vehicles in the platoon - the platoon length
        """
        return len(self.vehicles)

    def change_lane(self, direction):
        """
        Changes the lane of the entire platoon in the given direction

        :param direction: the direction to change lanes
        """
        lane = self.get_lane()
        destination_lane = lane + direction

        for vid in self.vehicles:
            change_lane(vid, destination_lane)

    def get_lane(self):
        """
        Return the current lane index that the platoon is driving in
        """
        return traci.vehicle.getLaneIndex(self.vehicles[0])

    def set_desired_speed(self, speed):
        """
        Set the desired speed of the platoon leader, and thus the entire platoon

        :param speed: the desired speed to set
        """
        self.desired_speed = speed
        set_par(self.vehicles[0], cc.PAR_CC_DESIRED_SPEED, speed)
        set_par(self.vehicles[0], cc.PAR_ACTIVE_CONTROLLER, cc.ACC)

    def get_leader(self, radar_front_distance=160):
        """
        Get the vehicle that is driving in front of the platoon within the radar distance

        :param radar_front_distance: the front radar distance of the platoon
        """
        vehicle = traci.vehicle.getLeader(self.vehicles[0], radar_front_distance)
        if vehicle is not None:
            # simulate real radar distance
            if vehicle[1] <= radar_front_distance:
                return vehicle[0], vehicle[1]
        return None, None

    def set_leader(self, leader):
        """
        Set the leader of the platoon which this platoon should adjust its ACC to

        :param leader: the vehicle id of the leading vehicle to set to
        """
        self.leader = leader
        if leader is None:
            self.set_desired_speed(self.desired_speed)
        else:
            set_par(self.vehicles[0], cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)

    def communicate(self):
        """
        Update inter vehicular data for cooperative adaptive cruise control settings for making platooning possible
        """
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
        """
        Update state of the platoon

        :param state: the new state to be changed to
        """
        self.last_state_change_step = self.step
        self.state = state

    def tick(self):
        """
        Run these commands every simulation step
        """
        # update cacc values
        self.communicate()

        # check for leader vehicles
        leader, distance = self.get_leader()

        if leader is None:
            self.set_leader(None)
            if self.state == PlatoonState.STATE_REQUEST_LEADER_LANE_CHANGE:
                self.set_state(PlatoonState.STATE_CRUISING)

        if self.state == PlatoonState.STATE_OVERTAKING_LEFT or self.state == PlatoonState.STATE_OVERTAKING_RIGHT:
            # keep checking left lane for chance to change back
            index_right = self.get_lane_change_split_index(Direction.RIGHT)
            index_left = self.get_lane_change_split_index(Direction.LEFT)

            if self.state == PlatoonState.STATE_OVERTAKING_RIGHT and self.get_length() == index_left \
                    and not self.vehicle_to_overtake_exists(Direction.LEFT):
                # the left lane can fit the whole platoon, so make the left lane change
                self.change_lane(Direction.LEFT)
                self.set_state(PlatoonState.STATE_CRUISING)
            elif self.state == PlatoonState.STATE_OVERTAKING_LEFT and self.get_length() == index_right \
                    and not self.vehicle_to_overtake_exists(Direction.RIGHT):
                # the left lane can fit the whole platoon, so make the left lane change
                self.change_lane(Direction.RIGHT)
                self.set_state(PlatoonState.STATE_CRUISING)

        if leader is not None and not is_platoon_vehicle(leader):

            if leader != self.leader:
                # a new leader, so reset to cruising state
                self.set_state(PlatoonState.STATE_CRUISING)

            # approach vehicle
            self.set_leader(leader)

            # if we have approached the leading vehicle
            if distance < self.min_gap and self.get_speed() < self.desired_speed:

                # we cannot lane change, so check front vehicle has v2v
                v2v_response = v2v.request_coordinates()
                if self.is_target_vehicle_gps_match(leader, v2v_response):
                    # leader is v2v enabled. so send request to change lanes.
                    v2v.request_lane_change_maneuver(self.vehicles[0], leader)
                    self.set_state(PlatoonState.STATE_REQUEST_LEADER_LANE_CHANGE)
                else:
                    # check lane change availability
                    index_right = self.get_lane_change_split_index(Direction.RIGHT)
                    index_left = self.get_lane_change_split_index(Direction.LEFT)

                    # left_lane_vehicles = self.get_left_lane_vehicles()
                    # right_lane_vehicles = self.get_right_lane_vehicles()

                    # check lane change availability
                    right_lane_vehicles_index, right_lane_vehicles = self.get_v2v_vehicles_up_to_index(Direction.RIGHT,
                                                                                                       v2v_response)
                    left_lane_vehicles_index, left_lane_vehicles = self.get_v2v_vehicles_up_to_index(Direction.LEFT,
                                                                                                     v2v_response)

                    if self.state == PlatoonState.STATE_REQUEST_RIGHT_VEHICLES_LANE_CHANGE or \
                            self.state == PlatoonState.STATE_REQUEST_LEFT_VEHICLES_LANE_CHANGE:
                        if self.state == PlatoonState.STATE_REQUEST_RIGHT_VEHICLES_LANE_CHANGE:
                            index = index_right
                            direction = Direction.RIGHT
                            next_state = PlatoonState.STATE_OVERTAKING_RIGHT
                            vehicle_index = right_lane_vehicles_index
                            vehicles = right_lane_vehicles
                            request_vehicle_move_state = PlatoonState.STATE_REQUEST_RIGHT_VEHICLES_LANE_CHANGE
                        else:
                            index = index_left
                            direction = Direction.LEFT
                            next_state = PlatoonState.STATE_OVERTAKING_LEFT
                            vehicle_index = left_lane_vehicles_index
                            vehicles = left_lane_vehicles
                            request_vehicle_move_state = PlatoonState.STATE_REQUEST_LEFT_VEHICLES_LANE_CHANGE

                        if self.get_length() == index:
                            # clear to change lanes
                            if self.step - self.last_state_change_step > 100:
                                self.change_lane(direction)
                                self.set_state(next_state)
                        elif index >= self.M:
                            # clear to change lanes
                            if self.step - self.last_state_change_step > 100:
                                # front platoon after split meets M requirement, so split
                                rear_platoon = self.split(index)
                                platoon_manager.add_platoon(rear_platoon)

                                self.change_lane(direction)
                                self.set_state(next_state)
                        elif vehicle_index >= self.M and len(vehicles) > 0:
                            # need to request more vehicles to change lanes
                            for vid in vehicles:
                                v2v.request_lane_change_maneuver(self.vehicles[0], vid)
                            self.set_state(request_vehicle_move_state)
                        else:
                            self.set_state(PlatoonState.STATE_CRUISING)
                    elif self.state == PlatoonState.STATE_CRUISING:
                        # if we can change lanes, just change lanes
                        if self.get_length() == index_right:
                            # clear to change lanes
                            self.change_lane(Direction.RIGHT)
                            self.set_state(PlatoonState.STATE_OVERTAKING_RIGHT)
                        elif self.get_length() == index_left:
                            # clear to change lanes
                            self.change_lane(Direction.LEFT)
                            self.set_state(PlatoonState.STATE_OVERTAKING_LEFT)
                        # check if we can signal other cars to move
                        elif left_lane_vehicles_index >= self.M and len(left_lane_vehicles) > 0:
                            just_ids = list(t[0] for t in v2v_response)
                            for vid in left_lane_vehicles:
                                v2v.request_lane_change_maneuver(self.vehicles[0], vid)
                            self.set_state(PlatoonState.STATE_REQUEST_LEFT_VEHICLES_LANE_CHANGE)
                        elif right_lane_vehicles_index >= self.M and len(right_lane_vehicles) > 0:
                            for vid in right_lane_vehicles:
                                v2v.request_lane_change_maneuver(self.vehicles[0], vid)
                            self.set_state(PlatoonState.STATE_REQUEST_RIGHT_VEHICLES_LANE_CHANGE)
                        elif index_right >= self.M:
                            # last resort / split
                            rear_platoon = self.split(index_right)
                            platoon_manager.add_platoon(rear_platoon)

                            self.change_lane(Direction.RIGHT)
                            self.set_state(PlatoonState.STATE_OVERTAKING_RIGHT)
                        elif index_left >= self.M:
                            # last resort / split
                            rear_platoon = self.split(index_left)
                            platoon_manager.add_platoon(rear_platoon)

                            self.change_lane(Direction.LEFT)
                            self.set_state(PlatoonState.STATE_OVERTAKING_LEFT)
                        else:
                            self.set_state(PlatoonState.STATE_CRUISING)

        self.step += 1

    def is_target_vehicle_gps_match(self, vid, v2v_response):
        """
        Check against the v2v response if the target vehicle id is within our v2v response - which means that
        the target vehicle is v2v enabled.

        :param vid: the target vehicle
        :param v2v_response: the response package of v2v equipped vehicle's GPS data
        """
        v_data = get_par(vid, cc.PAR_SPEED_AND_ACCELERATION)
        (target_v, target_a, target_u, target_x, target_y, target_t, _, _, _) = cc.unpack(v_data)
        for vehicle_data in v2v_response:
            (vid2, v, a, u, x, y, t) = vehicle_data
            if math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2) <= 0.1:
                print(vid + " = " + vid2)
                return True

        return False

    def are_target_vehicles_gps_match(self, vlist, v2v_response):
        """
        Check if all target vehicles are v2v equipped.

        :param vlist: a list of target vehicles
        :param v2v_response: the response package of v2v equipped vehicle's GPS data
        """
        result = True
        if len(vlist) == 0:
            return False
        for vid in vlist:
            result = result and self.is_target_vehicle_gps_match(vid, v2v_response)
        return result

    def get_left_lane_vehicles(self):
        """
        Returns a list of all vehicles in the left lane relative to this platoon's traveling lane
        """
        edge_id = traci.vehicle.getRoadID(self.vehicles[0])
        lane_index = traci.vehicle.getLaneIndex(self.vehicles[0])
        lane_count = traci.edge.getLaneNumber(edge_id)

        vehicles = set()

        if lane_index == lane_count - 1:
            return vehicles

        for pvid in self.vehicles:
            leaders = traci.vehicle.getLeftLeaders(pvid)
            followers = traci.vehicle.getLeftFollowers(pvid)

            for v in leaders:
                vid, dist = v
                if dist <= self.vehicle_length:
                    vehicles.add(vid)
            for v in followers:
                vid, dist = v
                if dist <= self.vehicle_length:
                    vehicles.add(vid)
        return vehicles

    def get_right_lane_vehicles(self):
        """
        Returns a list of all vehicles in the left lane relative to this platoon's traveling lane
        """
        edge_id = traci.vehicle.getRoadID(self.vehicles[0])
        lane_index = traci.vehicle.getLaneIndex(self.vehicles[0])

        vehicles = set()

        if lane_index == 0:
            return vehicles

        for pvid in self.vehicles:
            leaders = traci.vehicle.getRightLeaders(pvid)
            followers = traci.vehicle.getRightFollowers(pvid)

            for v in leaders:
                vid, dist = v
                leader_lane_index = traci.vehicle.getLaneIndex(vid)
                if leader_lane_index - lane_index == -1:
                    if dist <= self.vehicle_length:
                        vehicles.add(vid)
            for v in followers:
                vid, dist = v
                follower_lane_index = traci.vehicle.getLaneIndex(vid)
                if follower_lane_index - lane_index == -1:
                    if dist <= self.vehicle_length:
                        vehicles.add(vid)
        return vehicles

    def could_lane_change(self, vid, direction):
        """
        Returns whether a there is sufficient room for a given platoon member to change lanes in the given direction

        :param vid: the traci vehicle id of the platoon member
        :param direction: the direction to change lanes in
        """
        edge_id = traci.vehicle.getRoadID(vid)
        lane_count = traci.edge.getLaneNumber(edge_id)
        lane_index = traci.vehicle.getLaneIndex(vid)

        if direction == Direction.LEFT and lane_index == lane_count - 1:
            return False
        if direction == Direction.RIGHT and lane_index == 0:
            return False

        if direction == Direction.LEFT:
            leaders = traci.vehicle.getLeftLeaders(vid)
            followers = traci.vehicle.getLeftFollowers(vid)
        if direction == Direction.RIGHT:
            leaders = traci.vehicle.getRightLeaders(vid)
            followers = traci.vehicle.getRightFollowers(vid)

        for l in leaders:
            _, dist = l
            if dist <= self.vehicle_length:
                return False
        for f in followers:
            _, dist = f
            if dist <= self.vehicle_length:
                return False
        return True

    def get_speed(self):
        """
        Returns the speed of the platoon leader
        """
        return traci.vehicle.getSpeed(self.vehicles[0])

    def get_total_length(self):
        """
        Returns the length in meters of the platoon
        """
        return self.get_length() * (self.vehicle_length + self.min_gap) - self.min_gap

    def vehicle_to_overtake_exists(self, direction):
        """
        Returns true if there is a vehicle diagonally ahead of the platoon in the given direction

        :param direction: the direction in which to check diagonally for a vehicle
        """
        edge_id = traci.vehicle.getRoadID(self.vehicles[0])
        lane_count = traci.edge.getLaneNumber(edge_id)
        lane_index = traci.vehicle.getLaneIndex(self.vehicles[0])

        if direction == Direction.LEFT:
            leaders = traci.vehicle.getLeftLeaders(self.vehicles[0])
        if direction == Direction.RIGHT:
            leaders = traci.vehicle.getRightLeaders(self.vehicles[0])
        for l in leaders:
            lid, dist = l
            leader_lane_index = traci.vehicle.getLaneIndex(lid)
            if leader_lane_index - lane_index == direction:
                if dist <= self.min_gap + self.vehicle_length:
                    return True
        return False

    def get_v2v_vehicles_up_to_index(self, direction, v2v_response):
        """
        Returns the index within the platoon in which there are only v2v enabled vehicles in the given direction

        :param direction: the direction to which to check for v2v enabled vehicles
        :param v2v_response: the v2v response with vehicle GPS information

        :return: a tuple containing (1) the maximum index into the platoon for which there appear only v2v enabled
        vehicles in the given direction and (2) a list of traci vehicle ids for those adjacent v2v enabled vehicles
        """
        edge_id = traci.vehicle.getRoadID(self.vehicles[0])
        lane_count = traci.edge.getLaneNumber(edge_id)
        lane_index = traci.vehicle.getLaneIndex(self.vehicles[0])

        vehicles = set()

        if direction == Direction.LEFT and lane_index == lane_count - 1:
            return 0, vehicles
        if direction == Direction.RIGHT and lane_index == 0:
            return 0, vehicles

        for i, vid in enumerate(self.vehicles):
            vehicles_frame = set()
            if direction == Direction.LEFT:
                leaders = traci.vehicle.getLeftLeaders(vid)
                followers = traci.vehicle.getLeftFollowers(vid)
            if direction == Direction.RIGHT:
                leaders = traci.vehicle.getRightLeaders(vid)
                followers = traci.vehicle.getRightFollowers(vid)

            for v in leaders:
                lid, dist = v
                if dist <= self.vehicle_length:
                    if not self.is_target_vehicle_gps_match(lid, v2v_response):
                        return i, vehicles
                    if self.is_target_vehicle_gps_match(lid, v2v_response):
                        vehicles_frame.add(lid)
            for v in followers:
                fid, dist = v
                if dist <= self.vehicle_length:
                    if not self.is_target_vehicle_gps_match(fid, v2v_response):
                        return i, vehicles
                    if self.is_target_vehicle_gps_match(fid, v2v_response):
                        vehicles_frame.add(fid)
            vehicles.update(vehicles_frame)
        return len(self.vehicles), vehicles

    def get_lane_change_split_index(self, direction):
        """
        Returns the index within the platoon in which the platoon can split and change lanes.

        :param direction: the direction in which to check for lane change availability
        :return: the maximum index into the platoon for which there is space for a conditional split and lane change
        maneuver
        """
        for i, vid in enumerate(self.vehicles):
            if not self.could_lane_change(vid, direction):
                return i
        return len(self.vehicles)

    def split(self, i):
        """
        Splits the platoon at index i into two platoons: i.e. (0, i-1) and (i, last_platoon_member_index)

        :return: the Platoon behind that was created after the split
        """
        rear_vehicles = self.vehicles[i:]
        front_vehicles = self.vehicles[:i]

        self.vehicles = front_vehicles

        return Platoon(speed=self.desired_speed, vehicles=rear_vehicles)

    def build(self, n=6, pos=0, speed=SPEED, lane=DEFAULT_LANE):
        """
        Builds a platoon of (n) vehicles at a given speed, position, and lane.

        :param n: the number of vehicles in the platoon
        :param pos: the starting position of the platoon in meters
        :param speed: the starting speed of the platoon in meters/second
        :param lane: the starting lane of the platoon
        """
        for i in range(n):
            vid = vehicle_counter.get_next_platoon_vehicle_id()
            self.vehicles.append(vid)

            add_vehicle(vid, pos - i * (self.min_gap + self.vehicle_length), lane, speed, self.min_gap)

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
        self.state = PlatoonState.STATE_CRUISING
        self.vehicle_length = traci.vehicletype.getLength('PlatoonCar')
        self.min_gap = traci.vehicletype.getMinGap('PlatoonCar')
        self.last_state_change_step = 0
        self.step = 0

        # this is not a split platoon. it is a new platoon from scratch
        if "vehicles" not in kwargs:
            self.build(*args, **kwargs)
