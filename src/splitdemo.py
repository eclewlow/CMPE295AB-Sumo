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

import os
import sys
import time
import matplotlib.pyplot as plt
from collections import defaultdict

import ccparams as cc
import random
from utils import add_vehicle, set_par, change_lane, communicate, \
    get_distance, get_par, start_sumo, running

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib

import traci

SUMO_PARAMS = ["--lateral-resolution", "0.8",
               "--quit-on-end"]
# vehicle length
LENGTH = 4
# inter-vehicle distance
DISTANCE = 5
# inter-vehicle distance when leaving space for joining
JOIN_DISTANCE = DISTANCE * 2
# cruising speed
SPEED = 130 / 3.6

# maneuver states:
FIRST_STATE = -1
GOING_TO_POSITION = 0
OPENING_GAP = 1
COMPLETED = 2
FRONT_CHANGING_LANES = 3
FRONT_SPEED_UP = 4
REAR_CHANGING_LANES = 5
MERGE_PLATOONS = 6

# maneuver actors
LEADER = "v.0"
JOIN_POSITION = 3
FRONT_JOIN = "v.%d" % (JOIN_POSITION - 1)
BEHIND_JOIN = "v.%d" % JOIN_POSITION
N_VEHICLES = 6
JOINER = "v.%d" % N_VEHICLES
LEFT_SLOW_VEHICLE = "v.%d" % (N_VEHICLES + 1)
RIGHT_SLOW_VEHICLE = "v.%d" % (N_VEHICLES + 2)

# sumo launch command
sumoBinary = sumolib.checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "-D", "-c", "cfg/freeway_test.sumocfg"]

# for plots / graphs
x = defaultdict(list)
y_accel = defaultdict(list)
y_vel = defaultdict(list)


def add_vehicles(n, real_engine=False):
    # add a platoon of n vehicles
    topology = {}
    for i in range(n):
        vid = "v.%d" % i
        add_vehicle(vid, (n - i + 1) * (DISTANCE + LENGTH) + 50, 1, SPEED, DISTANCE,
                    real_engine)
        change_lane(vid, 1)
        if i == 0:
            set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
            set_par(vid, cc.PAR_CC_DESIRED_SPEED, SPEED)
        else:
            set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
        if i > 0:
            topology[vid] = {"front": "v.%d" % (i - 1), "leader": LEADER}

    # slow vehicle Lane 0
    vid = "v.%d" % n
    add_vehicle(vid, (n + 9) * (DISTANCE + LENGTH) + 50, 0, SPEED - 20, DISTANCE, real_engine, type_id='PlatoonCar')
    time.sleep(1)
    change_lane(vid, 0)
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
    set_par(vid, cc.PAR_CACC_SPACING, DISTANCE)

    # slow vehicle Lane 1
    vid = "v.%d" % (n + 1)
    add_vehicle(vid, (n + 15) * (DISTANCE + LENGTH) + 50, 1, SPEED - 20, DISTANCE, real_engine, type_id='PlatoonCar')
    change_lane(vid, 1)
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
    set_par(vid, cc.PAR_CACC_SPACING, DISTANCE)

    return topology


def get_in_position(vid, lid, topology):
    topology[vid] = {"leader": lid, "front": lid}
    set_par(vid, cc.PAR_CC_DESIRED_SPEED, SPEED + 15)
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
    return topology


def split_platoon(vid, jid, topology, n):
    index = int(vid.split(".")[1])
    for i in range(index + 1, n):
        topology["v.%d" % i]["leader"] = vid

    topology[vid]["front"] = jid
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
    return topology


def set_leader(vid, lid, topology, n):
    index = int(vid.split(".")[1])
    for i in range(index, n):
        topology["v.%d" % i]["leader"] = vid
    topology[vid]["front"] = lid
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)

    return topology


def front_speed_up(topology):
    topology.pop(LEADER)
    set_par(LEADER, cc.PAR_CC_DESIRED_SPEED, SPEED - 15)
    set_par(LEADER, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)

    return topology


def merge_platoons(vid, fid, topology, n):
    leader_id = topology[fid]["leader"]

    index = int(vid.split(".")[1])
    for i in range(index, n):
        # temporarily change the leader
        topology["v.%d" % i]["leader"] = leader_id
    topology[vid]["front"] = fid
    return topology


def can_change_lane(leader_id, direction, n):
    result = True
    index = int(leader_id.split(".")[1])
    for i in range(index, n):
        vid = "v.%d" % i
        result = result and traci.vehicle.couldChangeLane(vid, direction)
    return result

def platoon_ahead(vid, topology):
    result = traci.vehicle.getLeader(vid)
    if result is None:
        return None
    lid, dist = result
    if lid in topology:
        return lid



def record_data_points():
    for i in range(N_VEHICLES):
        vid = "v.%d" % i
        vehicle_data = get_par(vid, cc.PAR_SPEED_AND_ACCELERATION)
        (vel, acc, u, xpos, ypos, t, _, _, _) = cc.unpack(vehicle_data)

        x[vid].append(t)
        y_accel[vid].append(acc)
        y_vel[vid].append(vel)


def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("cfg/freeway_test.sumocfg", False)
    step = 0
    state = FIRST_STATE
    while running(demo_mode, step, 4000):

        traci.simulationStep()

        # first step, add vehicles
        if step == 0:
            topology = add_vehicles(N_VEHICLES, real_engine)
            traci.gui.trackVehicle("View #0", LEFT_SLOW_VEHICLE)
            traci.gui.setZoom("View #0", 20000)
        # every 100ms, update CACC and record data points for plot/graph
        if step % 10 == 1:
            communicate(topology)
            record_data_points()
        # at one second, begin phase 'approaching vehicles'
        if step == 100:
            state = GOING_TO_POSITION
            topology = get_in_position('v.0', LEFT_SLOW_VEHICLE, topology)
        if state == GOING_TO_POSITION:
            if get_distance('v.0', LEFT_SLOW_VEHICLE) < JOIN_DISTANCE + 1:
                state = FRONT_CHANGING_LANES
                topology = split_platoon(BEHIND_JOIN, FRONT_JOIN, topology, N_VEHICLES)
        if state == FRONT_CHANGING_LANES:
            if can_change_lane(LEADER, -1, JOIN_POSITION):
                for i in range(JOIN_POSITION):
                    vid = f'v.{i}'
                    change_lane(vid, 0)
                state = FRONT_SPEED_UP
                topology = set_leader(BEHIND_JOIN, LEFT_SLOW_VEHICLE, topology, N_VEHICLES)
                topology = front_speed_up(topology)

        if state == FRONT_SPEED_UP:
            if get_distance(BEHIND_JOIN, LEFT_SLOW_VEHICLE) < JOIN_DISTANCE + 1 and can_change_lane(BEHIND_JOIN, -1,
                                                                                                    N_VEHICLES):
                state = REAR_CHANGING_LANES
        if state == REAR_CHANGING_LANES:
            for i in range(JOIN_POSITION, N_VEHICLES):
                vid = f'v.{i}'
                change_lane(vid, 0)
            state = MERGE_PLATOONS
        if state == MERGE_PLATOONS:
            fid = platoon_ahead(BEHIND_JOIN, topology)
            if fid is not None:
                merge_platoons(BEHIND_JOIN, fid, topology, N_VEHICLES)
                state = COMPLETED

        step += 1

    traci.close()


if __name__ == "__main__":
    main(False, False)
    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    plt.figure(0)
    for i in range(N_VEHICLES):
        vid = "v.%d" % i
        plt.plot(x[vid], y_accel[vid], label=f"Platoon Vehicle {i + 1}", linestyle="-")
    plt.title("Acceleration of Platoon Vehicles in Platoon Split Scenario")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.legend()
    plt.show()

    plt.figure(1)
    for i in range(N_VEHICLES):
        vid = "v.%d" % i
        plt.plot(x[vid], y_vel[vid], label=f"Platoon Vehicle {i + 1}", linestyle="-")
    plt.title("Speed of Platoon Vehicles in Platoon Split Scenario")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend()
    plt.show()
