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

import pytest
from Simulation import Simulation
from Vehicle import Vehicle, vehicle_counter
from Platoon import Platoon
from PlatoonManager import platoon_manager
from VehicleManager import vehicle_manager
import traci


# Arrange
@pytest.fixture(autouse=True, params=[4])
def before_after(request):
    request.config.sim = Simulation()
    yield
    platoon_manager.reset()
    vehicle_counter.reset()
    vehicle_manager.reset()
    assert True


@pytest.mark.skip(reason="uncomment this to skip this test")
def test_add_slow_vehicle_and_track_platoon(request):
    simulation = request.config.sim

    platoon = simulation.add_platoon(platoon_length=6, platoon_start_position=50,
                                     platoon_start_lane=Platoon.DEFAULT_LANE,
                                     platoon_desired_speed=50)

    slow_vehicle_1 = simulation.add_vehicle(vehicle_start_position=6 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=1, vehicle_start_speed=30)
    slow_vehicle_2 = simulation.add_vehicle(vehicle_start_position=15 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=0, vehicle_start_speed=30)

    simulation.set_simulation_time_length(10)  # end simulation after 10 seconds

    simulation.set_zoom(20000)
    simulation.track_vehicle(platoon)

    simulation.run()


@pytest.mark.skip(reason="uncomment this to skip this test")
def test_add_slow_vehicle_and_require_overtake(request):
    simulation = request.config.sim

    platoon = simulation.add_platoon(platoon_length=6, platoon_start_position=6 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                     platoon_start_lane=Platoon.DEFAULT_LANE,
                                     platoon_desired_speed=50)

    slow_vehicle_1 = simulation.add_vehicle(vehicle_start_position=6 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=1, vehicle_start_speed=30)
    slow_vehicle_2 = simulation.add_vehicle(vehicle_start_position=18 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=2, vehicle_start_speed=30)

    simulation.set_simulation_time_length(60)  # end simulation after 10 seconds

    simulation.set_zoom(20000)
    simulation.track_vehicle(platoon)

    simulation.run()


@pytest.mark.skip(reason="uncomment this to skip this test")
def test_unable_to_split(request):
    simulation = request.config.sim

    platoon = simulation.add_platoon(platoon_length=6, platoon_start_position=6 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                     platoon_start_lane=Platoon.DEFAULT_LANE,
                                     platoon_desired_speed=50)

    slow_vehicle_1 = simulation.add_vehicle(vehicle_start_position=6 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=1, vehicle_start_speed=30)
    slow_vehicle_2 = simulation.add_vehicle(vehicle_start_position=10 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=2, vehicle_start_speed=30)

    simulation.set_simulation_time_length(25)  # end simulation after 10 seconds

    simulation.set_zoom(20000)
    simulation.track_vehicle(platoon)

    simulation.run()


@pytest.mark.skip(reason="uncomment this to skip this test")
def test_four_vehicle_split(request):
    simulation = request.config.sim

    platoon = simulation.add_platoon(platoon_length=6, platoon_start_position=6 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                     platoon_start_lane=Platoon.DEFAULT_LANE,
                                     platoon_desired_speed=50)

    slow_vehicle_1 = simulation.add_vehicle(vehicle_start_position=6 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=1, vehicle_start_speed=30)
    slow_vehicle_2 = simulation.add_vehicle(vehicle_start_position=15 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=2, vehicle_start_speed=30)

    simulation.set_simulation_time_length(25)  # end simulation after 10 seconds

    simulation.set_zoom(20000)
    simulation.track_vehicle(platoon)

    simulation.run()


@pytest.mark.skip(reason="uncomment this to skip this test")
def test_three_vehicle_split(request):
    simulation = request.config.sim

    platoon = simulation.add_platoon(platoon_length=6, platoon_start_position=6 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                     platoon_start_lane=Platoon.DEFAULT_LANE,
                                     platoon_desired_speed=50)

    slow_vehicle_1 = simulation.add_vehicle(vehicle_start_position=6 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=1, vehicle_start_speed=30)
    slow_vehicle_2 = simulation.add_vehicle(vehicle_start_position=14 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=2, vehicle_start_speed=30)

    simulation.set_simulation_time_length(25)  # end simulation after 10 seconds

    simulation.set_zoom(20000)
    simulation.track_vehicle(platoon)

    simulation.run()


# @pytest.mark.skip(reason="uncomment this to skip this test")
def test_dangerous_situation(request):
    simulation = request.config.sim

    platoon = simulation.add_platoon(platoon_length=6, platoon_start_position=6 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                     platoon_start_lane=Platoon.DEFAULT_LANE,
                                     platoon_desired_speed=50)

    slow_vehicle_1 = simulation.add_vehicle(vehicle_start_position=6 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=1, vehicle_start_speed=30)
    slow_vehicle_2 = simulation.add_vehicle(vehicle_start_position=15 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                            vehicle_start_lane=2, vehicle_start_speed=30)
    merging_vehicle = simulation.add_vehicle(vehicle_start_position=12 * (Vehicle.DISTANCE + Vehicle.LENGTH),
                                             vehicle_start_lane=0, vehicle_start_speed=30,
                                             commands={
                                                 1000: Vehicle.CMD_CHANGE_LANE_LEFT})  # at 10 seconds change lanes

    simulation.set_simulation_time_length(25)  # end simulation after 10 seconds

    simulation.set_zoom(20000)
    simulation.track_vehicle(platoon)

    simulation.run()
