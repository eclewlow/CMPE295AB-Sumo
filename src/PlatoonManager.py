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


class PlatoonManager:
    def add_platoon(self, platoon):
        self.platoons.append(platoon)

    def tick(self):
        for p in self.platoons:
            p.tick()

    def reset(self):
        self.platoons = list()

    def __init__(self, *args, **kwargs):
        self.platoons = list()

    def get_last_platoon_vehicle_id(self):
        last_vehicle = None
        for p in self.platoons:
            for vid in p.vehicles:
                if last_vehicle is None or traci.vehicle.getDistance(vid) < traci.vehicle.getDistance(last_vehicle):
                    last_vehicle = vid
        return last_vehicle


platoon_manager = PlatoonManager()
