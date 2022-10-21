#!/usr/bin/env python
#
# Copyright (c) 2017 Michele Segata <segata@ccs-labs.org>
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

# maneuver actors
LEADER = "v.0"
JOIN_POSITION = 3
FRONT_JOIN = "v.%d" % (JOIN_POSITION - 1)
BEHIND_JOIN = "v.%d" % JOIN_POSITION
N_VEHICLES = 6
JOINER = "v.%d" % N_VEHICLES
LEFT_SLOW_VEHICLE = "v.%d" % (N_VEHICLES+1)
RIGHT_SLOW_VEHICLE = "v.%d" % (N_VEHICLES+2)

# sumo launch command
sumoBinary = sumolib.checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "-D", "-c", "cfg/freeway_test.sumocfg"]


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
            pass
        else:
            set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
        if i > 0:
            topology[vid] = {"front": "v.%d" % (i - 1), "leader": LEADER}

    # slow vehicle Lane 0
    vid = "v.%d" % n
    add_vehicle(vid, (n + 10) * (DISTANCE + LENGTH) + 50, 0, SPEED - 20, DISTANCE, real_engine, type_id='PlatoonCar')
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


def open_gap(vid, jid, topology, n):
    index = int(vid.split(".")[1])
    for i in range(index + 1, n):
        topology["v.%d" % i]["leader"] = vid

    topology[vid]["front"] = jid
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
    return topology

def set_leader(vid, lid, topology, n):
    index = int(vid.split(".")[1])
    for i in range(index, n):
        # temporarily change the leader
        topology["v.%d" % i]["leader"] = vid
    # the front vehicle if the vehicle opening the gap is the joiner
    topology[vid]["front"] = lid
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)

    return topology

def front_speed_up(topology):
    topology.pop(LEADER)
    set_par(LEADER, cc.PAR_CC_DESIRED_SPEED, SPEED-15)
    set_par(LEADER, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)

    return topology


def reset_leader(vid, topology, n):
    index = int(vid.split(".")[1])
    for i in range(index, n):
        # temporarily change the leader
        topology["v.%d" % i]["leader"] = LEADER
    topology[vid]["front"] = FRONT_JOIN
    return topology


def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("cfg/freeway_test.sumocfg", False)
    step = 0
    state = FIRST_STATE
    while running(demo_mode, step, 6000):

        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 6000:
            start_sumo("cfg/freeway_test.sumocfg", True)
            step = 0
            state = GOING_TO_POSITION
            random.seed(1)

        traci.simulationStep()

        if step == 0:
            topology = add_vehicles(N_VEHICLES, real_engine)
            traci.gui.trackVehicle("View #0", LEFT_SLOW_VEHICLE)
            traci.gui.setZoom("View #0", 20000)
        if step % 10 == 1:
            communicate(topology)
        if step == 100:
            state = GOING_TO_POSITION
            topology = get_in_position('v.0', LEFT_SLOW_VEHICLE, topology)
        if state == GOING_TO_POSITION:
            if get_distance('v.0', LEFT_SLOW_VEHICLE) < JOIN_DISTANCE + 1:
                state = FRONT_CHANGING_LANES
                topology = open_gap(BEHIND_JOIN, FRONT_JOIN, topology, N_VEHICLES)
        if state == FRONT_CHANGING_LANES:
            for i in range(JOIN_POSITION):
                vid = f'v.{i}'
                change_lane(vid, 0)
            state = FRONT_SPEED_UP

            topology = set_leader(BEHIND_JOIN, LEFT_SLOW_VEHICLE, topology, N_VEHICLES)

            topology = front_speed_up(topology)

        if state == FRONT_SPEED_UP:
            if get_distance(BEHIND_JOIN, LEFT_SLOW_VEHICLE) < JOIN_DISTANCE + 1 and traci.vehicle.couldChangeLane(BEHIND_JOIN, -1):
                state = REAR_CHANGING_LANES
        if state == REAR_CHANGING_LANES:
            for i in range(JOIN_POSITION, N_VEHICLES):
                vid = f'v.{i}'
                change_lane(vid, 0)
            topology = reset_leader(BEHIND_JOIN, topology, N_VEHICLES)
            state = COMPLETED



        step += 1

    traci.close()


if __name__ == "__main__":
    main(True, False)
