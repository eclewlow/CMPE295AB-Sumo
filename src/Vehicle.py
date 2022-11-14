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

def is_platoon_vehicle(vid):
    return vid.startswith(VehicleCounter.ID_PRE_PLATOON)


class VehicleCounter:
    i = 0
    ID_PRE = 'v.'
    ID_PRE_PLATOON = 'platoon.'

    def __init__(self):
        i = 0

    def get_next_vehicle_id(self):
        vid = f"{self.ID_PRE}{self.i}"
        self.i += 1
        return vid

    def get_next_platoon_vehicle_id(self):
        vid = f"{self.ID_PRE_PLATOON}{self.i}"
        self.i += 1
        return vid


vehicle_counter = VehicleCounter()
