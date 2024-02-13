import sys
import time
from ddcontroller import *

config = '/opt/robot/edgeai-robotics-demos/python/common/ddcontroller/config/rovybot_config.yaml'

try:
    robot = DDRobot(config_path=config, debug=True)
except Exception as e:
    print("Exception: {0}".format(e))
    sys.exit(2)

robot.set_motion([1.0*robot.max_linear_velocity, 0.0*robot.max_angular_velocity])
time.sleep(5)
robot.set_motion([1.0*robot.max_linear_velocity, 1.0*robot.max_angular_velocity])
time.sleep(5)
robot.set_motion([0.0*robot.max_linear_velocity, 1.0*robot.max_angular_velocity])
time.sleep(5)
robot.set_motion([0.0*robot.max_linear_velocity, 0.0*robot.max_angular_velocity])
time.sleep(5)
robot.set_motion([-1.0*robot.max_linear_velocity, 0.0*robot.max_angular_velocity])
time.sleep(5)
robot.set_motion([-1.0*robot.max_linear_velocity, -1.0*robot.max_angular_velocity])
time.sleep(5)
robot.set_motion([0.0*robot.max_linear_velocity, -1.0*robot.max_angular_velocity])
time.sleep(5)
robot.stop()
del robot

