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

import threading
import queue
import yaml
import logging
import math
import numpy as np

from common.robot_motion_command import *
from common.logger import *
from common.current_sensor import *
from common.payload_bucket import *
from common.data_store import *

class RobotIf:
    """
    An interface class for the robot motor control.

    ...

    Attributes:
        MAX_IN_QUEUE_SIZE: int
            Maximum size of the input queue. The sender will block if the queue is full.

        IN_QUEUE_TIMEOUT: int
            The timeout value of the queue. If the queue times out then the object will
            send a stop command to the robot.

        ID_MASTER_TASK: int
            Task Id asigned to this controlling class.

    """
    MAX_IN_QUEUE_SIZE = 10
    IN_QUEUE_TIMEOUT  = 1 # in seconds
    ID_MASTER_TASK    = 12

    STATE_START = 1
    STATE_INIT = 2
    STATE_JOYSTICK_CTRL = 3
    STATE_ESTOP = 4
    STATE_SEARCHING_FOR_PERSON = 5
    STATE_WAIT_FOR_PERSON = 6
    STATE_TURNING_TO_PERSON = 7
    STATE_TRAVELING_TO_PERSON = 8
    STATE_ROTATE_180_1 = 9  
    STATE_WAIT_FOR_BALL = 10
    STATE_ROTATE_180_2 = 11
    STATE_SEARCHING_FOR_PATCH = 12
    STATE_NAVIGATING_TO_PATCH = 13
    STATE_TURNING_TO_PATCH = 14
    STATE_DROPPING_BALL = 15
    STATE_PREP_FOR_START = 16
    STATE_EMERGENCY_STOP = 20
    STATE_WAIT_FOR_TIMER = 21
    STATE_SUBJECT_FOLLOWER = 30
    STATE_STOP = 40

    LOOP_RATE_HZ = 100

    def __init__(self, config, queue_time_out=None):
        """
        Args:
            config: str, optional
                Name of the YAML configuration file definining the robot operational parameters.
            queue_time_out (int): Time-out for command queue reads

        """
        if config is None:
            raise ValueError('NULL configuration passed.')
        
        self._config           = config
        self._stop_thread      = False
        self._queue_time_out   = queue_time_out
        self.thread            = None
        self.motionInputQ      = None
        self._emergency_stop   = False
        self._prevCmd          = [0,0]
        self._registered_tasks = [RobotIf.ID_MASTER_TASK]
        self.stop_cmd          = MotionCommand(tskId=RobotIf.ID_MASTER_TASK)
        self.slowly_turn_cmd   = MotionCommand(tskId=RobotIf.ID_MASTER_TASK)
        self.state             = RobotIf.STATE_START
        self._rateHz           = RobotIf.LOOP_RATE_HZ
        self.data_store        = config['subject_detector']['data_store']
        self._config           = config['robot_config']
        self.subdet_width      = config['subject_detector']['input_width']
        self.subdet_hfov       = config['subject_detector']['input_hfov']
        self.heading_tolerance = config['subject_detector']['heading_tolerance']
        self.distance_to_person = config['subject_detector']['distance_to_person']

        self.payload_bucket  = PayloadBucket(pin=33,range_angle_min=0, range_angle_max=45)

        self.slowly_turn_cmd.cmd = [0, 0.5]

        self.stop_cmd.cmd = [0.0, 0.0]

        self.logger = logging.getLogger('rovybot_demo.robot_if')

        # Get the parameters for setting up the sensor
        with open( self._config, 'r') as f:
            robot_config = yaml.safe_load(f)

        self.start()

    def sendCommand(self, motionCmd):
        """
        Enqueues the motion command to the input queue.

        Args:
            motionCmd: MotionCommand
                Command to be sent to the motor control. This command will be acted upon
                under the following conditions:
                - The emergency stop condition is False
                - The taskId specified in the message has been enabled through a call to
                  register_control_task() method
                - The processing thread is not in the process of stopping

        Raises:
            Propagates the exception generated by the queue module.

        """
        try:
            if (motionCmd.id in self._registered_tasks) and self.motionInputQ:
                self.motionInputQ.put(motionCmd)
            else:
                # Sender has not been registered to send commands
                print("{} not in registered tasks. Ignoring.".format(motionCmd.id))
                pass 
        except Exception as e:
            raise

    # def _control_thread(self):
    #     if self._robot is None:
    #         raise Exception('No valid robot instance exists')

    #     # Create an input queue
    #     self.inputQ = queue.Queue(maxsize=RobotIf.MAX_IN_QUEUE_SIZE)
    #     self._robot.set_motion(self.stop_cmd.cmd)

    #     # Process input commands
    #     while self._stop_thread == False:
    #         try:
    #             # Get a command from the queue. This will block until it times out
    #             cmd = self.inputQ.get(block=True, timeout=self._queue_time_out)
    #             self.inputQ.task_done()
    #         except queue.Empty:
    #             # Timeout condition
    #             cmd = self.stop_cmd

    #         if self._stop_thread == True:
    #             self.logger.debug("Stop Thread")
    #             self._robot.set_motion(self.stop_cmd.cmd)
    #             break

    #         # Discard the command if emergency stop flag has been set
    #         if self._emergency_stop == True:
    #             self.logger.info("Emergency Stop")
    #             cmd = self.stop_cmd
    #         else:
    #             if ((self._prevCmd[0] != cmd.cmd[0]) or
    #                 (self._prevCmd[1] != cmd.cmd[1])):
    #                 d = self._robot.get_distance_to_obstacle()
    #                 self.logger.debug("Lin: {0:.2f}, Ang: {1:.2f}, D: {2:.2f}".format(cmd.cmd[0], cmd.cmd[1], d))

    #                 # Send command to the robot
    #                 self._robot.set_motion(cmd.cmd)
    #                 self._prevCmd = cmd.cmd
   
    #     # Delete input queue
    #     del self.inputQ
    #     self.inputQ = None

    def _change_state(self, new_state):
        self.prev_state = self.state
        self.state = new_state

    def _pause_for_timer(self, timeout_sec, next_state):
        # Store the current state, which the timer state will switch back to after elapsed time
        self.prev_state = self.state
        self.state = RobotIf.STATE_WAIT_FOR_TIMER
        self.next_state = next_state
        self.logger.info("State change: STATE_WAIT_FOR_TIMER. Timeout: {0:.2f}s".format(timeout_sec))
        self._timer_timeout = timeout_sec * 1e9 # ns
        self._timer_start_time = time.monotonic_ns()

    def _control_thread(self):
        if self._robot is None:
            raise Exception('No valid robot instance exists')

        # Create an input queue
        self.motionInputQ = queue.Queue(maxsize=RobotIf.MAX_IN_QUEUE_SIZE)

        # Keep the robot stationary
        self._robot.set_motion(self.stop_cmd.cmd)

        self.state = RobotIf.STATE_INIT
        self.logger.info("State change: INIT")

        # Main robot control loop.  This is a timed loop
        while self._stop_thread == False:

            if self._stop_thread == True:
                self.logger.debug("Stop Thread")
                self._robot.set_motion(self.stop_cmd.cmd)
                break

            match self.state:
                case RobotIf.STATE_INIT:
                    self.logger.info("State entered: INIT")
                    self.payload_bucket.shake()
                    # Wait for a sec
                    self._pause_for_timer(3, RobotIf.STATE_SEARCHING_FOR_PERSON)


                case RobotIf.STATE_SEARCHING_FOR_PERSON:
                    self.logger.info("State entered: SEARCHING_FOR_PERSON")
                    
                    # Get robot ready to search for a person
                    self._robot.set_motion(self.slowly_turn_cmd.cmd)

                    self._change_state(RobotIf.STATE_WAIT_FOR_PERSON)
                    self.logger.info("State change: WAIT_FOR_PERSON")

                case RobotIf.STATE_WAIT_FOR_PERSON:

                    position = self.data_store.subject_position.get()

                    if (position):
                        (x,y,radius) = position

                        # TBD: Collect several samples over a timespan, then calculate a target heading

                        theta = (self.subdet_width/2 - x)*(self.subdet_hfov/self.subdet_width)*math.pi*2/360
                        current_heading = self._robot.get_heading()
                        self.target_heading = current_heading + theta
                        self.target_heading = np.arctan2(np.sin(self.target_heading), np.cos(self.target_heading))
                        
                        self._robot.set_heading(self.target_heading, max_angular_velocity=0.4)

                        #self.logger.info("X = {0:.2f} Y = {1:.2f} R:{2:.2f} CH:{3:.2f} NH:{3:.2f} CL:{4}".format(x, y, radius, current_heading, self.target_heading, self._robot.get_control_level()))

                        self._change_state(RobotIf.STATE_TURNING_TO_PERSON)
                        self.logger.info("State change: STATE_TURNING_TO_PERSON")

                case RobotIf.STATE_TURNING_TO_PERSON:
                    
                    # Wait for robot to get to its specified heading
                    heading_error = self._robot.get_heading() - self.target_heading
                    heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

                    heading_error_degrees = abs(heading_error*180/math.pi)

                    if (heading_error_degrees < self.heading_tolerance):
                        self._robot.set_control_level(1) # Turn off heading and position controllers
                        self._robot.set_motion([0.0,0.0])
                        
                        self._pause_for_timer(2.0, RobotIf.STATE_TRAVELING_TO_PERSON)
                        self.logger.info("State change: STATE_TRAVELING_TO_PERSON")
                    else:
                        self.logger.debug("STATE_TURNING_TO_PERSON: HE = {0:.2f} CL: {1}".format(heading_error_degrees, self._robot.get_control_level()))


                case RobotIf.STATE_TRAVELING_TO_PERSON:
                    if(self.prev_state != self.state):  # do this once
                        # Move to Person
                        target_velocity = .2
                        self._robot.set_linear_velocity(target_velocity)
                        obstacle_distance = self._robot.get_distance_to_obstacle()
                        self.logger.info("LV: {0:.2f} D: {1:.2f} ".format(target_velocity, obstacle_distance))
                        self.prev_state = self.state
                    else:
                        # Wait until robot gets within distance of obstacle
                        distance_to_obstacle = self._robot.get_distance_to_obstacle()
                        if (distance_to_obstacle < self.distance_to_person):
                            # Stop motion
                            self._robot.set_motion([0.0,0.0])
                            #self._robot.define_heading(0)
                            self._pause_for_timer(1.0, RobotIf.STATE_ROTATE_180_1)
                        else:
                            self.logger.info("STATE_TRAVELING_TO_PERSON D: {0:.2f} CL:{1}".format(distance_to_obstacle, self._robot.get_control_level()))

                case RobotIf.STATE_ROTATE_180_1:

                    if(self.prev_state != self.state):  # do this once
                        self.target_heading = self._robot.get_heading()+math.pi
                        self.target_heading = np.arctan2(np.sin(self.target_heading), np.cos(self.target_heading))
                        self._robot.set_motion([0,0.4])
                        #self._robot.set_heading(self.target_heading, max_angular_velocity=self._robot.max_traveling_angular_velocity)
                        self.prev_state = self.state
                    else:
                        # Wait for robot to get to its specified heading
                        heading_error = self._robot.get_heading() - self.target_heading
                        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

                        heading_error_degrees = abs(heading_error*180/math.pi)

                        if (heading_error_degrees < self.heading_tolerance):
                            #self._robot.control_level = 1 # Turn off heading and position controllers
                            self._robot.set_motion([0.0,0.0])
                            self._pause_for_timer(10.0, RobotIf.STATE_SEARCHING_FOR_PERSON)
                            #self.logger.info("State change: STATE_WAIT_FOR_BALL")
                        else:
                            self.logger.debug("STATE_ROTATE_180_1: HE = {0:.2f}, CL:{1}".format(heading_error_degrees, self._robot.get_control_level()))                                      
                
                case RobotIf.STATE_WAIT_FOR_BALL:

                    # Wait until robot gets within distance of obstacle
                    ball_color = self.data_store.ball_color.get()
                    if(ball_color):
                        # Store the ball color
                        self.ball_color = ball_color
                        self.logger.info("Ball found. Color: {0}".format(self.ball_color))

                        self._robot.set_motion([0.0,0.0])
                        #self._robot.set_motion(self.slowly_turn_cmd.cmd)
                        #self._robot.define_heading(0)
                        #self.target_heading = self._robot.get_heading()+math.pi
                        #self._robot.set_heading(self.target_heading, max_angular_velocity=self._robot.max_traveling_angular_velocity)
 
                        # Change state to looking for patch
                        self._pause_for_timer(2.0, RobotIf.STATE_SEARCHING_FOR_PATCH)

                case RobotIf.STATE_ROTATE_180_2:

                    # Wait for robot to get to its specified heading
                    heading_error = self._robot.get_heading() - self.target_heading
                    heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

                    heading_error_degrees = abs(heading_error*180/math.pi)

                    if (heading_error_degrees < self.heading_tolerance):
                        self._robot.set_control_level(1) # Turn off heading and position controllers
                        self._robot.set_motion([0.0, 0.0])
                        self._change_state(RobotIf.STATE_SEARCHING_FOR_PATCH)
                        self.logger.info("State change: STATE_SEARCHING_FOR_PATCH")
                    else:
                        motion = self._robot.get_motion()
                        self.logger.debug("STATE_ROTATE_180_2: HE = {0:.2f} LV: {1:.2f} AV: {2:.2f}".format(heading_error_degrees, motion[0], motion[1]))                      

                case RobotIf.STATE_SEARCHING_FOR_PATCH:

                    if(self.prev_state != self.state):  # do this once
                        self._robot.set_motion([0,1.0])
                        self.prev_state = self.state
                    else:
                        # Initially, check to see if we see the patch with the matching ball color
                        width = None
                        match(self.ball_color):
                            case "green":
                                patch_position = self.data_store.patch_position_green.get()
                                if (patch_position):
                                    (x, y, width) = patch_position
                            case "blue":
                                patch_position = self.data_store.patch_position_blue.get()
                                if (patch_position):
                                    (x, y, width) = patch_position
                            case "orange":
                                patch_position = self.data_store.patch_position_orange.get()
                                if (patch_position):
                                    (x, y, width) = patch_position
                            case _:
                                # If you get here then you have a bug in your code
                                self.logger.error("STATE_SEARCHING_FOR_PATCH: ball color incorrect")
                        
                        if(width):
                                self._change_state(RobotIf.STATE_NAVIGATING_TO_PATCH)
                                self.patch_position = patch_position
                                self._robot.set_motion([0.0,0.0])
                                self._pause_for_timer(2.0, RobotIf.STATE_NAVIGATING_TO_PATCH)
                                self.logger.info("State change: STATE_NAVIGATING_TO_PATCH")
                        else:
                            motion = self._robot.get_motion()
                            self.logger.debug("STATE_SEARCHING_FOR_PATCH: LV: {0:.2f} AV: {1:.2f} D: {2:.2f}".format(motion[0], motion[1],self._robot.get_distance_to_obstacle()))                      

                case RobotIf.STATE_NAVIGATING_TO_PATCH:
                    if (self.patch_position):
                        (x,y,width) = self.patch_position

                        # TBD: Collect several samples over a timespan, then calculate a target heading

                        theta = (self.subdet_width/2 - x)*(self.subdet_hfov/self.subdet_width)*math.pi/180
                        current_heading = self._robot.get_heading()
                        self.target_heading = current_heading - theta + math.pi # turn 180
                        self.target_heading = np.arctan2(np.sin(self.target_heading), np.cos(self.target_heading))
                        self._robot.set_heading(self.target_heading)

                        self.logger.info("X = {0:.2f} Y = {1:.2f} R:{2:.2f} CH:{3:.2f} NH:{3:.2f}".format(x, y, radius, current_heading, self.target_heading ))

                        self._change_state(RobotIf.STATE_TURNING_TO_PATCH)
                        self.logger.info("State change: STATE_TURNING_TO_PATCH")

                case RobotIf.STATE_TURNING_TO_PATCH:
                    
                    # Wait for robot to get to its specified heading
                    heading_error = self._robot.get_heading() - self.target_heading
                    heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

                    heading_error_degrees = abs(heading_error*180/math.pi)

                    if (heading_error_degrees < self.heading_tolerance):
                        self._robot.set_control_level(1) # Turn off heading and position controllers
                        self._robot.set_motion([0.0,0.0])
                        
                        # Move to Person
                        #target_velocity = self._robot.max_traveling_linear_velocity/2
                        #self._robot.set_linear_velocity(target_velocity)
                        #obstacle_distance = self._robot.get_distance_to_obstacle()
                        #self.logger.info("LV: {0:.2f} D: {1:.2f} ".format(target_velocity, obstacle_distance))

                        self._change_state(RobotIf.STATE_STOP)
                        self.logger.info("State change: STATE_STOP")
                    else:
                        self.logger.debug("STATE_TURNING_TO_PATCH: HE = {0:.2f}".format(heading_error_degrees))
                             

                case RobotIf.STATE_WAIT_FOR_TIMER:
                    current_time = time.monotonic_ns()
                    if(current_time > self._timer_start_time + self._timer_timeout):
                        self._change_state(self.next_state)
                        self.logger.debug("STATE_WAIT_FOR_TIMER: Timer done. Changing state to {}".format(self.prev_state))

                case RobotIf.STATE_SUBJECT_FOLLOWER:
                 
                    # Try to get a command from the Queue
                    try:
                        cmd = None
                        cmd = self.motionInputQ.get(block=False, timeout=self._queue_time_out)
                        self.motionInputQ.task_done()
                    except queue.Empty:  
                        pass

                    # Discard the command if emergency stop flag has been set
                    if self._emergency_stop == True:
                        self.logger.info("Emergency Stop")
                        cmd = self.stop_cmd
                        self._robot.set_motion([0.0,0.0])
                    else:
                        if (cmd):
                            if ((self._prevCmd[0] != cmd.cmd[0]) or
                                (self._prevCmd[1] != cmd.cmd[1])):
                                d = self._robot.get_distance_to_obstacle()
                                self.logger.info("Lin: {0:.2f}, Ang: {1:.2f}, D: {2:.2f}, CL:{3}".format(cmd.cmd[0], cmd.cmd[1], d, self._robot.get_control_level()))

                                # Send command to the robot
                                self._robot.set_motion(cmd.cmd)
                                self._prevCmd = cmd.cmd

                case RobotIf.STATE_EMERGENCY_STOP:
                    # Wait for the stop to be released
                    pass

                case RobotIf.STATE_JOYSTICK_CTRL:
                 
                    # Try to get a command from the Queue
                    try:
                        cmd = None
                        cmd = self.motionInputQ.get(block=False, timeout=self._queue_time_out)
                        self.motionInputQ.task_done()
                    except queue.Empty:  
                        pass

                    # Discard the command if emergency stop flag has been set
                    if self._emergency_stop == True:
                        self.logger.info("Emergency Stop")
                        self._robot.set_motion([0.0,0.0])
                    else:
                        if (cmd):
                            if ((self._prevCmd[0] != cmd.cmd[0]) or
                                (self._prevCmd[1] != cmd.cmd[1])):
                                d = self._robot.get_distance_to_obstacle()
                                self.logger.info("Lin: {0:.2f}, Ang: {1:.2f}, D: {2:.2f}".format(cmd.cmd[0], cmd.cmd[1], d))

                                # Send command to the robot
                                self._robot.set_motion(cmd.cmd)
                                self._prevCmd = cmd.cmd

                case RobotIf.STATE_STOP:
                    if(self.prev_state != RobotIf.STATE_STOP):
                        self._robot.set_motion([0.0,0.0])  # do this once
                case _:
                    pass

            # Service the motion Q so that it doesn't fill up.
            try:
                cmd = None
                cmd = self.motionInputQ.get(block=False, timeout=self._queue_time_out)
                self.motionInputQ.task_done()
            except queue.Empty:  
                pass               
       
            time.sleep(1.0/self._rateHz)

        self.logger.info("Control Thread Exit")
        # Delete input queue
        del self.motionInputQ
        self.motionInputQ = None

    def set_emergency_flag(self, boolFlag):
        """
        Sets or resets the emergency stop condition. If the flag is set to True
            then all subsequent motion commands will be dropped.

        Args:
            boolFlag: bool
                A flag to set/reset emergency stop condition.

        """
        self._emergency_stop = boolFlag
        if self._robot:
            if boolFlag:
                self._robot.set_motion([0.0,0.0])
                self._robot.set_control_level(1)
                self._change_state(RobotIf.STATE_EMERGENCY_STOP)
                self.logger.info("Changed to STATE_EMERGENCY_STOP")
            else:
                self._change_state(RobotIf.STATE_INIT)
                self.logger.info("Changed to STATE_INIT")

    def set_joystick_control(self, boolFlag):
        """
        Sets or resets the joystick control.

        Args:
            boolFlag: bool
                A flag to set/reset emergency stop condition.

        """
        if(self._emergency_stop and boolFlag):
            self.logger.info("Cannot enable joystick control when emergency stop is enabled.")
            return
        
        self._joystick_control = boolFlag
        if self._robot:
            if boolFlag:
                self._robot.set_motion([0.0,0.0])
                self._robot.set_control_level(1)
                self._change_state(RobotIf.STATE_JOYSTICK_CTRL)
                self.logger.info("Joystick Control enabled")
                self.logger.info("Changed to STATE_JOYSTICK_CTRL")
            else:
                self._change_state(RobotIf.STATE_INIT)
                self.logger.info("Joystick Control disabled")
                self.logger.info("Changed to STATE_INIT")               

    def register_control_task(self, taskId):
        """
        Registers the entity with the taskId as a valid source for the motion commands.

        Args:
            taskId: int
                Id of the controlling task. The following are the only allowed values:
                - ID_USER_TASK
                - ID_CONTROLLER_TASK

        """
        if (taskId != ID_USER_TASK and
            taskId != ID_CONTROLLER_TASK):
            print("ERROR: Invalid taskId specified. Valid values are ID_USER_TASK, ID_CONTROLLER_TASK")
            return

        if taskId not in self._registered_tasks:
            self._registered_tasks.append(taskId)

    def unregister_control_task(self, taskId):
        """
        Un-registers a previous source for the motion commands. One unregistered, any commands from
        the source will be dropped.

        Args:
            taskId: int
                Id of the controlling task. The following are the only allowed values:
                - ID_USER_TASK
                - ID_CONTROLLER_TASK

        """
        if taskId in self._registered_tasks:
            self.sendCommand(self.stop_cmd)
            self._registered_tasks.remove(taskId)

    def wait_for_exit(self):
        """
        Method to provide a mechanism for the caller to wait for the processing
        thread to exit.
        """
        if (self.thread):
            self.thread.join()
            self.thread = None
        if self._robot:
            self._robot.stop()

    def start(self):
        """
        Starts the processing thread. The caller should call wait_for_exit() if
        the calling application has no other means blocking until this object is
        destroyed.
        """
        if self.thread == None:
            self._robot = None # Plugin your code here
            self.thread = threading.Thread(target=self._control_thread)
            self.thread.start()

    def stop(self):
        """
        Stops the processing thread. This call blocks until the processing thread exits.
        """
        if self._stop_thread == False:
            # Send a dummy message to wakeup the thread
            self.sendCommand(self.stop_cmd)
            self._stop_thread = True
            self.wait_for_exit()

    def __del__(self):
        self.stop()

