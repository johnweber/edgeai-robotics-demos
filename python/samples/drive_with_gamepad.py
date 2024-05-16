# A demonstration program: drive with gamepad control.

# Import External programs
import sys
import time
import VL53L1X

# Import Internal Programs
from common.gamepad_motor_control import *

def main():
    config = '/opt/robot/edgeai-robotics-demos/python/apps/rovybot_demo/rovybot_demo_config.yaml'

    try:
        # Get the global instance of the robot control object
        motor_control = GamepadMotorControl(config=config)
        print("main: start")
        status = motor_control.start()
        if status < 0:
            sys.exit(status)

        is_alive = True
        while(is_alive):
            is_alive = motor_control.wait_for_exit(1)

    except KeyboardInterrupt:
        print("Keyboard interrrupt")
        motor_control.stop()
    finally:
        pass
    
    del motor_control

    sys.exit(0)
        
if __name__ == "__main__":
    main()
