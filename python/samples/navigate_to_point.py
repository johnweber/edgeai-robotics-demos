#!/usr/bin/env python3

"""
This file is part of the ddcontroller library (https://github.com/ansarid/ddcontroller).
Copyright (C) 2022  Daniyal Ansari

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import time
from ddcontroller import DDRobot

config = '/opt/robot/edgeai-robotics-demos/python/common/ddcontroller/config/rovybot_config.yaml'

# Create robot object
robot = DDRobot(config_path=config,debug=True)

try:

    # Set target location for navigation to (1,1)
    robot.go_to([.5, .5], tolerance=0.01, backwards=False)

    # Loop while robot is running and not at target location
    while robot.running and not robot.reached_target_position:

        # Get the robot's latest location
        x, y = robot.get_global_position()

        # Print the location of the robot
        print(f"Global Position: {round(x, 3)}, {round(y, 3)}", end="\r")

        # Run loop at 50Hz
        time.sleep(1/50)

    print('\nDone!')
    time.sleep(10)
    print('\nExit!')

except KeyboardInterrupt:
    print('Stopping...')

finally:
    # Clean up.
    robot.stop()
    print('Stopped.')