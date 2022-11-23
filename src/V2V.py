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

from VehicleManager import vehicle_manager

from enum import Enum, auto


class V2V:
    def __init__(self, *args, **kwargs):
        pass

    V2V_LANE_CHANGE_MANEUVER_REQUEST = auto()

    def request_coordinates(self):
        response = list()
        for vid, vehicle in vehicle_manager.vehicles.items():
            if vehicle.v2v:
                vehicle_data = get_par(vid, cc.PAR_SPEED_AND_ACCELERATION)
                (v, a, u, x, y, t, _, _, _) = cc.unpack(vehicle_data)
                response.append((vid, v, a, u, x, y, t))
        return response

    def request_lane_change_maneuver(self, sender_id, recipient_id):
        recipient = vehicle_manager.get_vehicle(recipient_id)
        recipient.receive_v2v_request(sender_id, self.V2V_LANE_CHANGE_MANEUVER_REQUEST)


v2v = V2V()
