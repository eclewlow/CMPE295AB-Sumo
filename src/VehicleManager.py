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
import traci
from utils import add_vehicle, set_par, change_lane, communicate, \
    get_distance, get_par, start_sumo, running
import ccparams as cc

class VehicleManager:
    def add_vehicle(self, vehicle):
        self.vehicles[vehicle.vid] = vehicle

    def get_vehicle(self, vid):
        return self.vehicles.get(vid)

    def tick(self, step):
        for v in self.vehicles.values():
            v.tick(step)

    def reset(self):
        self.vehicles = dict()

    def __init__(self, *args, **kwargs):
        self.vehicles = dict()

    def v2v_request_coordinates(self):
        v2v_response = list()
        for vid, vehicle in self.vehicles.items():
            if vehicle.v2v:
                vehicle_data = get_par(vid, cc.PAR_SPEED_AND_ACCELERATION)
                (v, a, u, x, y, t, _, _, _) = cc.unpack(vehicle_data)
                v2v_response.append((vid, v, a, u, x, y, t))
        return v2v_response

vehicle_manager = VehicleManager()
