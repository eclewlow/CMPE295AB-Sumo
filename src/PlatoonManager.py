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
    """
    Class to manage platoons and simulation step for all platoons
    """
    def add_platoon(self, platoon):
        """
        Add a platoon for the PlatoonManger to manage

        :param platoon: the platoon to be managed
        """
        self.platoons.append(platoon)

    def tick(self):
        """
        Run a single step for all the platoons that the PlatoonManager is managing
        """
        for p in self.platoons:
            p.tick()

    def reset(self):
        """
        Clear the current list of platoons managed by the PlatoonManager
        """
        self.platoons = list()

    def __init__(self, *args, **kwargs):
        self.platoons = list()

    def get_last_platoon_vehicle_id(self):
        """
        Returns the traci vehicle id of the last platoon member within the fleet of platoons managed by this
        PlatoonManager
        """
        last_vehicle = None
        for p in self.platoons:
            for vid in p.vehicles:
                if last_vehicle is None or traci.vehicle.getDistance(vid) < traci.vehicle.getDistance(last_vehicle):
                    last_vehicle = vid
        return last_vehicle


platoon_manager = PlatoonManager()
