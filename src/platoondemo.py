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

from Platoon import Platoon
from Vehicle import vehicle_counter
from PlatoonManager import platoon_manager

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
LEFT_SLOW_VEHICLE = vehicle_counter.get_next_vehicle_id()
RIGHT_SLOW_VEHICLE = vehicle_counter.get_next_vehicle_id()

# sumo launch command
sumoBinary = sumolib.checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "-D", "-c", "cfg/freeway_test.sumocfg"]

# for plots / graphs
x = defaultdict(list)
y_accel = defaultdict(list)
y_vel = defaultdict(list)


def add_vehicles(n, real_engine=False):
    # slow vehicle Lane 0
    vid = RIGHT_SLOW_VEHICLE
    add_vehicle(vid, (n + 9) * (DISTANCE + LENGTH) + 50, 0, SPEED - 20, DISTANCE, real_engine, type_id='PlatoonCar')
    time.sleep(1)
    change_lane(vid, 0)
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
    set_par(vid, cc.PAR_CACC_SPACING, DISTANCE)

    # slow vehicle Lane 1
    vid = LEFT_SLOW_VEHICLE
    add_vehicle(vid, (n + 15) * (DISTANCE + LENGTH) + 50, 1, SPEED - 20, DISTANCE, real_engine, type_id='PlatoonCar')
    change_lane(vid, 1)
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
    set_par(vid, cc.PAR_CACC_SPACING, DISTANCE)


def main(demo_mode, real_engine, setter=None):
    global platoon
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("cfg/freeway_test.sumocfg", False)
    step = 0
    while running(demo_mode, step, 6000):

        traci.simulationStep()

        # first step, add vehicles
        if step == 0:
            platoon = Platoon(n=N_VEHICLES, pos=DISTANCE, speed=SPEED)
            platoon_manager.add_platoon(platoon)

            add_vehicles(N_VEHICLES, real_engine)
            traci.gui.trackVehicle("View #0", LEFT_SLOW_VEHICLE)
            traci.gui.setZoom("View #0", 20000)

        platoon_manager.tick()

        step += 1

    traci.close()


if __name__ == "__main__":
    main(False, False)
