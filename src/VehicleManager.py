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
from utils import get_par


class VehicleManager:
    """
    Class to manage the creation of vehicles and simulation step for all vehicles
    """
    def add_vehicle(self, vehicle):
        """
        Add a vehicle for the VehicleManger to manage

        :param vehicle: the vehicle to be managed
        """
        self.vehicles[vehicle.vid] = vehicle

    def get_vehicle(self, vid):
        """
        Get a vehicle managed by the VehicleManager

        :param vid: the traci vehicle id of the vehicle to get
        """
        return self.vehicles.get(vid)

    def tick(self, step):
        """
        Run a single step for all the vehicles that the VehicleManager is managing

        :param step: the current simulation step
        """
        for v in self.vehicles.values():
            v.tick(step)

    def reset(self):
        """
        Clear the current list of vehicles managed by the VehicleManager
        """
        self.vehicles = dict()

    def __init__(self, *args, **kwargs):
        self.vehicles = dict()

    def v2v_request_coordinates(self):
        """
        This functions simulates a V2V request for GPS-coordinates being received and responds with a list of
        all vehicle's GPS data if the vehicle has V2V communication enabled.

        :return: a list of tuples - each tuple contains vehicular data such as speed, acceleration, and coordinates
        """
        v2v_response = list()
        for vid, vehicle in self.vehicles.items():
            if vehicle.v2v:
                vehicle_data = get_par(vid, cc.PAR_SPEED_AND_ACCELERATION)
                (v, a, u, x, y, t, _, _, _) = cc.unpack(vehicle_data)
                v2v_response.append((vid, v, a, u, x, y, t))
        return v2v_response


vehicle_manager = VehicleManager()
