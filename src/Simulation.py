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

import random
import time

import traci

import ccparams as cc
from Platoon import Platoon
from PlatoonManager import platoon_manager
from Vehicle import vehicle_counter, Vehicle
from VehicleManager import vehicle_manager
from utils import add_vehicle, set_par, start_sumo, running, running_distance


class Simulation:
    """
    Simulation class for encapsulating a simulation and making it configurable
    """

    def __init__(self, run_time_seconds=None, platoon_run_distance=None):
        self.platoon_run_distance = platoon_run_distance
        self.run_time_seconds = run_time_seconds

        self.step = 0

        # used to randomly color the vehicles
        random.seed(1)
        start_sumo("cfg/map.sumocfg", False)

    def set_simulation_time_length(self, length):
        """
        Set the amount of time the simulation should run for

        :param length: the time length in seconds
        """
        self.run_time_seconds = length

    def set_simulation_platoon_run_distance(self, distance):
        """
        Set the distance the platoon should travel at which point the simulation will end

        :param distance: the distance in meters
        """
        self.platoon_run_distance = distance

    def track_vehicle(self, vid):
        """
        Track the given vehicle in the Sumo GUI

        :param vid: the target vehicle's traci vehicle id
        """
        traci.gui.trackVehicle("View #0", vid)

    def set_zoom(self, zoom=20000):
        """
        Set the zoom for the Sumo GUI

        :param zoom: the zoom value to set
        """
        traci.gui.setZoom("View #0", zoom)

    def add_platoon(self, platoon_length=6, platoon_start_position=50, platoon_start_lane=Platoon.DEFAULT_LANE,
                    platoon_desired_speed=Platoon.SPEED):
        """
        Function to add a platoon to the simulation

        :param platoon_length: the length of the platoon
        :param platoon_start_position: the start position of the platoon
        :param platoon_start_lane: the start_lane of the platoon
        :param platoon_desired_speed: the desired speed of the platoon
        """
        platoon = Platoon(n=platoon_length, pos=platoon_start_position, lane=platoon_start_lane,
                          speed=platoon_desired_speed)
        platoon_manager.add_platoon(platoon)

        return platoon

    def add_vehicle(self, vehicle_start_position=0, vehicle_start_lane=Vehicle.DEFAULT_SLOW_LANE,
                    vehicle_start_speed=Vehicle.DEFAULT_SLOW_SPEED, v2v=False, commands=dict()):
        """
        Function to add a vehicle to the simulation

        :param vehicle_start_position: the start position of the vehicle
        :param vehicle_start_lane: the start_lane of the vehicle
        :param vehicle_start_speed: the desired speed of the vehicle
        :param v2v: whether the vehicle is equipped with V2V
        :param commands: a dictionary of commands to execute during the simulation
        """
        vid = vehicle_counter.get_next_vehicle_id()

        min_gap = traci.vehicletype.getMinGap('V2V_Car')

        if v2v:
            color = (255, 0, 0)
        else:
            color = (0, 0, 255)

        add_vehicle(vid, vehicle_start_position, vehicle_start_lane, vehicle_start_speed, min_gap, type_id='V2V_Car',
                    color=color)

        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        set_par(vid, cc.PAR_CACC_SPACING, min_gap)

        vehicle_manager.add_vehicle(Vehicle(vid, commands=commands, v2v=v2v))

        return vid

    def run(self):
        """
        The main execution loop for the simulation
        """
        self.step = 0

        last_platoon_vehicle = platoon_manager.get_last_platoon_vehicle_id()
        total_simulation_time = 0

        while running(self.step, self.run_time_seconds) and running_distance(last_platoon_vehicle,
                                                                             self.platoon_run_distance):
            traci.simulationStep()

            platoon_manager.tick()
            vehicle_manager.tick(self.step)

            self.step += 1
            total_simulation_time = traci.simulation.getTime()

        traci.close()
        time.sleep(5)
        return total_simulation_time
