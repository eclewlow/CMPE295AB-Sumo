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

from enum import auto

import ccparams as cc
from VehicleManager import vehicle_manager
from utils import get_par


class V2V:
    """
    A class that facilitates V2V message transmission
    """

    def __init__(self, *args, **kwargs):
        pass

    V2V_LANE_CHANGE_MANEUVER_REQUEST = auto()

    def request_coordinates(self):
        """
        Simulates a V2V message broadcast sent to all vehicles requesting for GPS information

        :return: a list of vehicular information, including coordinates, speed, and acceleration
        """
        response = list()
        for vid, vehicle in vehicle_manager.vehicles.items():
            if vehicle.v2v:
                vehicle_data = get_par(vid, cc.PAR_SPEED_AND_ACCELERATION)
                (v, a, u, x, y, t, _, _, _) = cc.unpack(vehicle_data)
                response.append((vid, v, a, u, x, y, t))
        return response

    def request_lane_change_maneuver(self, sender_id, recipient_id):
        """
        Simulates a V2V message targeted at a given recipient requesting that the recipient changes lanes.

        :param sender_id: the originator of the request
        :param recipient_id: the target recipient of the request
        """
        recipient = vehicle_manager.get_vehicle(recipient_id)
        recipient.receive_v2v_request(sender_id, self.V2V_LANE_CHANGE_MANEUVER_REQUEST)


v2v = V2V()
