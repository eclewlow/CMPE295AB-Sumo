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
from Vehicle import vehicle_counter, Vehicle
from PlatoonManager import platoon_manager
from VehicleManager import vehicle_manager

SUMO_PARAMS = ["--lateral-resolution", "0.8",
               "--quit-on-end"]

# sumo launch command
sumoBinary = sumolib.checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "-D", "-c", "cfg/freeway_test.sumocfg"]


class Simulation:

    def __init__(self, run_time_seconds=60):
        self.run_time_seconds = run_time_seconds

        # used to randomly color the vehicles
        random.seed(1)
        start_sumo("cfg/freeway_test.sumocfg", False)

    def set_simulation_time_length(self, length):
        self.run_time_seconds = length

    def track_vehicle(self, vid):
        traci.gui.trackVehicle("View #0", vid)

    def set_zoom(self, zoom=20000):
        traci.gui.setZoom("View #0", zoom)

    def add_platoon(self, platoon_length=6, platoon_start_position=50, platoon_start_lane=Platoon.DEFAULT_LANE,
                    platoon_desired_speed=Platoon.SPEED):
        platoon = Platoon(n=platoon_length, pos=platoon_start_position, lane=platoon_start_lane,
                          speed=platoon_desired_speed)
        platoon_manager.add_platoon(platoon)

        return platoon.vehicles[0]

    def add_vehicle(self, vehicle_start_position=0, vehicle_start_lane=Vehicle.DEFAULT_SLOW_LANE,
                    vehicle_start_speed=Vehicle.DEFAULT_SLOW_SPEED, commands=dict()):
        vid = vehicle_counter.get_next_vehicle_id()
        add_vehicle(vid, vehicle_start_position, 0, vehicle_start_speed, Vehicle.DISTANCE)
        # time.sleep(1)
        change_lane(vid, vehicle_start_lane)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        set_par(vid, cc.PAR_CACC_SPACING, Vehicle.DISTANCE)

        vehicle_manager.add_vehicle(Vehicle(vid, commands))

        return vid

    def run(self):
        step = 0
        while running(step, self.run_time_seconds * 100):
            traci.simulationStep()

            platoon_manager.tick()
            vehicle_manager.tick(step)

            step += 1

        traci.close()
        time.sleep(5)