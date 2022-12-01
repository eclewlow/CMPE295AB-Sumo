# Platoon Overtaking on Freeways using V2V Communication

This repository includes all necessary files for simulating the cooperative overtaking algorithm for platoons on freeways as proposed in []

[SUMO](https://www.eclipse.org/sumo/) (Simulation of Urban MObility) is used as simulation environment.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

The following software has to be installed:

* [Python 3](https://www.python.org/) - Version 3.4 or later required
* [SUMO](https://www.eclipse.org/sumo/) - Simulation of Urban MObility - Version 1.7.0 or higher required
* [pytest](https://docs.pytest.org/en/stable/) - to execute the included test cases

### Installing

Just clone the repository.  Optionally, create a Python virtual environment.  Install the project's python dependencies with

```powershell
$ pip install -r requirements.txt
```

## Executing the Algorithm

### Running the test cases

* `tests/test_simulation.py` - Tests for overtaking maneuver

You can run the tests using `pytest` from the root directory of this repository:
```powershell
env PYTHONPATH=$(pwd)/src pytest -v tests/
```

## Using the Algorithm in Own Test Cases

Use the following guideline to build a platoon that utilizes the overtaking algorithm.

### Build a Platoon
```python
simulation = Simulation()
platoon = simulation.add_platoon(platoon_length=6, platoon_start_position=50,
                                     platoon_start_lane=Platoon.DEFAULT_LANE,
                                     platoon_desired_speed=50)
simulation.run()
```
This will build a platoon with `6` vehicles (including the leader) of vehicle type
`PlatoonCar` with desired speed `50` m/s with starting position `50m` in the default Platoon lane.
The vehicle type as well as the route have to be defined in the according `.rou.xml` file of SUMO.

PDF and Details can be found at [https://drive.google.com/file/d/1rSCgEsY8Ds0HoX8eFjzRPuqCV6rvLffn/view](https://drive.google.com/file/d/1rSCgEsY8Ds0HoX8eFjzRPuqCV6rvLffn/view).

## License
```
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
```
