#  Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Gamepad controller for motor control

This script provides a class for interfacing with a gamepad controller
and mapping a few button events to generate robot motion commands. The
interface provided by the robot_motion_command is used to form the robot
control messages.
"""

import sys
import time
import threading

# Import Internal Programs
import common.gamepad as gp
from common.robot_motion_command import *
from common.robot_drive import *

BUTTON_Y     = 4
"""int: Button for enabling gamepad controller """

BUTTON_X     = 7
"""int: Button for disabling gamepad controller """

BUTTON_B     = 5
"""int: Button for setting emergency stop flag in the robot context """

BUTTON_A     = 6
"""int: Button for resetting emergency stop flag in the robot context """

BUTTON_START = 13
"""int: Button for setting the control by the application """

BUTTON_BACK  = 12
"""int: Button for relinquishing the by from the application """

class GamepadMotorControl:
    """
    A class for interfacing with a gamepad controller and mapping the controller
    button events to robot motion commands.

    Attributes:
        MAX_ANGULAR_VELOCITY (float): Maximum allowed angular velocity
        MAX_LINEAR_VELOCITY (float): Maximum allowed linear velocity
        config (str): File spwcifying the YAML configuration to setup the robot under control
        _stop_thread (bool): Flag to  control the control thread execution
        _control_on (bool): Flag to track if the gamepad controller has been enabled
        _robot (obj): A reference to the robot object context
        _rateHz (float): The rate at which the control thread samples the gamepad events
        thread (thread): COntrol thread handle

    """

    MAX_ANGULAR_VELOCITY = 2.7 # rad/s
    MAX_LINEAR_VELOCITY  = 1.0 # m/s

    def __init__(self, config=None, rateHz=25.0):
        """
        Initializes the internal state and acquires a reference to the robot object.
        Also resets the emergency flag within the robot object.

        Args:
            config (str): File spwcifying the YAML configuration to setup the robot under control
            rateHz (float): The rate at which the control thread samples the gamepad events

        Raises:
            ValueError: If 'config' is None.
            ValueError: If 'rateHz' is not a non-zero positive number

        """

        self.thread = None
        self._robot = False

        if config is None:
            raise ValueError("NULL configuration passed.")

        if rateHz <= 0:
            raise ValueError("Invalid rate specified.")

        try:
            self.gamepad = gp.Gamepad()
        except Exception as e:
            self.gamepad = None
            raise e

        self._rateHz = rateHz

        self._config      = config
        self._stop_thread = False
        self._control_on  = False
        self._robot       = get_robot_control_instance(self._config)
        if self._robot == None:
            print("Gamepad: failed to get robot control instance")
            return -1
        #self._robot.set_emergency_flag(True)
        self.thread = threading.Thread(target=self._control_thread)

        # Grab the max velocities from the robot instance
        self.max_linear_velocity = self._robot._robot.max_linear_velocity
        self.max_angular_velocity = self._robot._robot.max_angular_velocity
        print("Gamepad Control: Max LV: {0:.2f} Max AV: {1:.2f}".format(self.max_linear_velocity,self.max_angular_velocity))

    def is_alive(self):
        if (self.thread == None):
            return False

        return self.thread.is_alive()

    def _control_thread(self):
        """
        Thread body implementing the morot command generation logic.

        This thread gets the gamepad at the rate determined by '_rateHz'. It then processes
        the events as per the state machine design and send out a command to the robot motor
        control interface under the following conditions:
        - The gamepad control event (BUTTON_Y) is set
        - The emergency stop condition event (Button_B) is reset

        """
        user_task_registered = False

        print(f"starting gamepad control thread")

        while self._stop_thread == False:
            # COLLECT GAMEPAD COMMANDS
            # BUG: this blocks and inhibhits the app from exiting with CTRL-C
            gp_data = self.gamepad.getData()

            # Mutiply joystick x axis by scuttle max angular velocity to get angular velocity
            angVel = gp_data[0] * self.max_angular_velocity

            # Mutiply joystick y axis by scuttle max linear velocity to get linear velocity
            linVel  = gp_data[1] * self.max_linear_velocity
            
            # State machine logic
            if gp_data[BUTTON_Y] > 0:
                print(f"control on")
                self._control_on = True
                self._robot.set_joystick_control(True) 
                self._robot.register_control_task(ID_CONTROLLER_TASK)

            elif gp_data[BUTTON_X] > 0:
                print(f"control off")
                self._control_on = False
                self._robot.set_joystick_control(False)
                self._robot.unregister_control_task(ID_CONTROLLER_TASK)

            elif gp_data[BUTTON_B] > 0:
                # Set the emergency stop condition
                print(f"Gamepad Control: Emergency Stop Set")
                self._robot.set_emergency_flag(True)
            elif gp_data[BUTTON_A] > 0:
                # Reset the emergency stop condition
                print(f"Gamepad Control: Emergency Stop Reset (off)")
                self._robot.set_emergency_flag(False)
            elif gp_data[BUTTON_START] > 0:
                # Enable the user task control and register with the robot control
                user_task_registered = True
                self._robot.register_control_task(ID_USER_TASK)
            elif gp_data[BUTTON_BACK] > 0:
                # Disable the user task control
                if user_task_registered:
                    user_task_registered = False
                    print(f"Gamepad Control: User task unregistered")
                    self._robot.unregister_control_task(ID_USER_TASK)

            # Send the command only if the gamepad control has been enabled
            if self._control_on:
                cmd = MotionCommand([linVel, angVel], tskId=ID_CONTROLLER_TASK)
                self._robot.sendCommand(cmd)

            # Honor the rate
            time.sleep(1.0/self._rateHz)

        print(f"Stopping gamepad control thread")

    def start(self):
        """
        Starts a processing thread if one is already not running.

        Returns:
            0 if successful, a negative value otherwise.

        """
        self._stop_thread = False
        self.thread.start()

        return 0

    def wait_for_exit(self, timeout=1):
        """
        A blocking call for the processing thread to exit. 
        """
        self.thread.join(timeout)

        return self.thread.is_alive()
 
    def stop(self):
        """
        Stops the processing thread, if one is running. The call will wait for the thread to exit
        before returning so will block until the thread is exited.

        The call will return immediately if no processing thread is being executed.

        """
        self._stop_thread = True

    def __del__(self):
        # Release the instance of the robot control object
        if(self.gamepad):
            self.gamepad.close()  # will block
        if self._robot:
            release_robot_control_instance()
            self._robot = None

