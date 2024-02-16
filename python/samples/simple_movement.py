import sys
import time
from ddcontroller import *

def print_motion(linvel, angvel):
    print(f'Motion: Linear={linvel:2f}, Angular={angvel:2f}')

config = '/opt/robot/edgeai-robotics-demos/python/common/ddcontroller/config/rovybot_config.yaml'

velfactors = [[1.0, 0.0],
            [0.5, 1.0],
            [0.0, 1.0],
            [0.0, 0.0],
            [0.0, -1.0],
            [-0.5, -1.0],
            [-1.0, 0.0]]

try:
    robot = DDRobot(config_path=config, debug=True)

    for velfactor in velfactors:
        linvel = velfactor[0]*robot.max_linear_velocity
        angvel = velfactor[1]*robot.max_angular_velocity
        print_motion(linvel, angvel)
        robot.set_motion([linvel, angvel])
        time.sleep(5)

    robot.stop()

except KeyboardInterrupt:
    print("Keyboard interrrupt")
    robot.stop()

del robot

print("exiting")
sys.exit(0)

