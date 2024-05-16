#!/usr/bin/python3
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

""" EdgeAI subject follower

This script implements the EdgeAI processing chain based subject follower application.

"""

import sys
import yaml
import subprocess
import cmd
import app_config
import numpy as np
import cv2
import imutils

from common.logger import *
from common.command_line_parse import get_cmdline_args
from common.subject_follower import SubjectFollower
from common.gamepad_motor_control import *
from common.robot_drive import *
from common.data_store import *

from subject_detector_class import SubjectDetector

class subjectDetectorCallbackObj:
    """
    A class for providing the interface to a robot under control. An object
    of this type is registered with the EdgeAI processing chain such that
    once the data has been through the inference stage, the post-processed
    image and the associated detection information is passed to this object
    for further action.

    The details of the detected object are passed to the subject follower
    algorithm instance for deriving the robot motion control commands and
    sending them over to the robot control interface.

    """

    def __init__(self, config, gamepad_control):
        self.thresh        = config['subject_detector']['prob_threshold']
        self.target_class  = config['subject_detector']['target_class']
        min_radius         = config['subject_detector']['min_radius']
        target_radius      = config['subject_detector']['target_radius']
        center_threshold   = config['subject_detector']['center_threshold']
        self.input_width   = config['subject_detector']['input_width']
        self.input_height  = config['subject_detector']['input_height']
        self.data_store    = config['subject_detector']['data_store']

  
        self.logger = logging.getLogger('rovybot_demo.subjectDetectorCallbackObj')

        try:
            self.follower = SubjectFollower(input_width=self.input_width,
                                          input_height=self.input_height,
                                          min_radius=min_radius,
                                          target_radius=target_radius,
                                          center_threshold=center_threshold)
            self.robot = get_robot_control_instance(config)

            # If there is no gamepad controller connected then take the robot
            # out of the emergency stop state

            # Eventually, the gamepad needs to be used in an override mode.
            # In that mode, the gamepad should control the robot and override
            # the commands from the user task.  Then, when gamepad control is
            # done, the user can press a button and allow the user to take over.
            # The main issue is the gamepadcontroller needs to be a bit smarter
            # so that it only sends commands when it is in the correct mode.
            if gamepad_control == None:
                self.robot.register_control_task(ID_USER_TASK)
        except Exception as e:
            print("Exception: {0}".format(e))
            sys.exit(2)

    def __call__(self, img, box, class_name, reset=False):
        """
        A functor invoked by the tailend of the EdgeAI processing chain.

        Args:
            img (image): Post-processed image from the EdgeAI object detection chain
            box (list): A list of 4 values representing the top-left and bottom-right
                        co-ordinates of the bounding box surrounding a detected object.
            class_name (str): A string representation of the class of the detected object
            reset (bool): A flag to indicate if the detection data should be looked at.
        """
        # Linear and angular velocities set by default.  [0, 0.5] would rotate the robot
        # at 0.5 rad/s by default in a search mode.
        #chassisTargets = [0, 0.3]

        if reset == False:
            # Compute the center (x,y) of the object and derive the width.
            x      = (box[0]+box[2])/2
            y      = (box[1]+box[3])/2
            radius = (box[2]-box[0])/2

            # Obtain the linear and angular valocity components
            #chassisTargets = self.follower.getChassisTargets(x, y, radius)
            self.data_store.subject_position.set((x,y,radius))
            #self.logger.info("X = {0:.2f} Y = {1:.2f} R:{2:.2f}".format(x, y, radius))
            #self.logger.info("X = {0:.2f} Y = {1:.2f} R:{2:.2f} T:{3}".format(x, y, radius, chassisTargets))
        #else:
            #self.logger.debug("T:{}".format(chassisTargets))



        # Build the robot motion command message and send it to the robot control
        #cmd = MotionCommand(chassisTargets)
        #self.robot.sendCommand(cmd)

    def stop(self):
        # Release the robot object context
        release_robot_control_instance()

    def __del__(self):
        self.stop()

# class subjectDetectorCallbackObj

class ballDetectorCBObj:
    """
    A class for providing feedback to the main program on the status
    of the balls in the payload bin.
    """

    def __init__(self, config):
        # robot_config       = config['robot_config']
        # min_radius         = config['subject_detector']['min_radius']
        # self.input_width   = config['subject_detector']['input_width']
        # self.input_height  = config['subject_detector']['input_height']
  
        self.logger = logging.getLogger('rovybot_demo.ballDetectorCB')

        self.ball_color = None

        # Used to show masks for color filter mask adjustments
        self._color_upper =  app_config.color_upper
        self._color_lower =  app_config.color_lower

        self.greenLower = np.array( \
            config['subject_detector']['ball_detector_config']['ball_colors']['green']['min'])
        self.greenUpper = np.array( \
            config['subject_detector']['ball_detector_config']['ball_colors']['green']['max'])

        self.blueLower = np.array( \
            config['subject_detector']['ball_detector_config']['ball_colors']['blue']['min'])
        self.blueUpper = np.array( \
            config['subject_detector']['ball_detector_config']['ball_colors']['blue']['max'])

        self.orangeLower = np.array( \
            config['subject_detector']['ball_detector_config']['ball_colors']['orange']['min'])
        self.orangeUpper = np.array( \
            config['subject_detector']['ball_detector_config']['ball_colors']['orange']['max'])

        self.min_radius = config['subject_detector']['ball_detector_config']['min_radius']

        self.roi = None
        self.roi_w = config['subject_detector']['ball_detector_config']['roi']['w']
        self.roi_h = config['subject_detector']['ball_detector_config']['roi']['h']
        self.roi_cx = config['subject_detector']['ball_detector_config']['roi']['cx']
        self.roi_cy = config['subject_detector']['ball_detector_config']['roi']['cy']
        self.roi_show_box = config['subject_detector']['ball_detector_config']['roi']['show_bounding_box']
        self.use_roi = config['subject_detector']['ball_detector_config']['use_roi']
        self.data_store    = config['subject_detector']['data_store']


    def __call__(self, img, reset=False):
        """
        A functor invoked by the tailend of the ball detection OpenCV processing chain

        Args:
            img (image): Post-processed image from the OpenCV chain
            reset (bool): A flag to indicate if the detection data should be looked at.
        """
        if not self.roi:
            w = float(img.shape[1])
            h = float(img.shape[0])
            # compute the width and height of ROI based on percentage
            # of the image.
            b_w = int(w * self.roi_w)
            b_h = int(h * self.roi_h)
            
            # compute the center of the ROI based on percentage
            # of the image.
            b_cx = int(w * self.roi_cx)
            b_cy = int(h * self.roi_cy)

            # x1, y1, x2, y2
            self.roi = [int(b_cx-b_w/2), int(b_cy - b_h/2), int(b_cx+b_w/2), int(b_cy + b_h/2)]

        if (not self.use_roi):
            self.roi = [0, 0, img.shape[1], img.shape[0]]

        box = self.roi

        # if color has changed, then set up the local color to the new color
        if (self._color_upper != app_config.color_upper) or \
           (self._color_lower != app_config.color_lower):

            self._color_upper =  app_config.color_upper
            self._color_lower =  app_config.color_lower           
        
        # img = self.highlight_ball_with_color(img.copy(),
        #                                       color_lower=self._color_lower,
        #                                       color_upper=self._color_upper,
        #                                       show_mask=True)

        ball_color = None
        img, ball_color = self.is_ball_detected(img)

        if(self.roi_show_box and self.use_roi):
            self.overlay_bounding_box(img, box, "none")
        
        # Check to see if the color state has changed
        if(ball_color != self.ball_color):
            self.ball_color = ball_color
            if(self.ball_color is None):
                self.data_store.ball_color.clear()
            else:
                self.data_store.ball_color.set(self.ball_color)
            self.logger.info("ball color changed from {0} to {1}".format(self.ball_color, ball_color))

    def overlay_bounding_box(self, frame, box, class_name):
        """
        draw bounding box at given co-ordinates.

        Args:
            frame (numpy array): Input image where the overlay should be drawn
            bbox : Bounding box co-ordinates in format [X1 Y1 X2 Y2]
            class_name : Name of the class to overlay
        """
        box_color = (20, 220, 20)
        cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), box_color, 2)

        return frame
    
    def highlight_ball_with_color(self, frame, color_lower, color_upper, show_mask=False):

        # blur the image
        #local_frame = imutils.resize(frame, width=300)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        lower = np.array(color_lower)
        upper = np.array(color_upper)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > self.min_radius:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)                   

        if show_mask:
            # return the mask
            rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            return rgb

        return frame
    
    def detect_ball(self, img, colorLower, colorUpper):
        """
        Search an image for one ball in the range of color provided.

        returns: (x, y), radius where
            (x,y) are the center of the ball
            if no ball, radius  = None
        """

         # blur the image
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # only proceed if the radius meets a minimum size
            if radius > self.min_radius:
                return (x,y), radius
    
        # Nothing found
        return (0,0), None

    def is_ball_detected(self, frame):
        """
        Search a portion of the frame for a ball with the specific
        colors.
        """
        if (self.use_roi):
            crop = frame[self.roi[1]:self.roi[3],self.roi[0]:self.roi[2]]
        else:
            crop = frame

        # Detect Green
        (x,y), radiusGreen = self.detect_ball(crop, self.greenLower, self.greenUpper)

        if (radiusGreen):
            # x,y are the center of the circle in the ROI
            # Translate this to frame:
            (xf, yf) = (self.roi[0] + x, self.roi[1]+ y )

            cv2.circle(frame, (int(xf), int(yf)), int(radiusGreen),
                (0, 255, 0), 2)
            ball_color = 'green'
            return frame, ball_color

        # Detect Blue
        (x,y), radiusBlue = self.detect_ball(crop.copy(), self.blueLower, self.blueUpper)

        if (radiusBlue):
            # x,y are the center of the circle in the ROI
            # Translate this to frame:
            (xf, yf) = (self.roi[0] + x, self.roi[1]+ y )

            cv2.circle(frame, (int(xf), int(yf)), int(radiusBlue),
                (0, 0, 255), 2)
            ball_color = 'blue'
            return frame, ball_color
            
        (x,y), radiusOrange = self.detect_ball(crop.copy(), self.orangeLower, self.orangeUpper)

        if (radiusOrange):
            # x,y are the center of the circle in the ROI
            # Translate this to frame:
            (xf, yf) = (self.roi[0] + x, self.roi[1]+ y )

            cv2.circle(frame, (int(xf), int(yf)), int(radiusOrange),
                (244, 99, 58), 2)
            
            ball_color = 'orange'
            return frame, ball_color

        ball_color = None
        return frame, ball_color

    def stop(self):
        # Release the robot object context
        print("{}: stopping".format(__name__))

    def __del__(self):
        self.stop()

# class ballDetectorCBObj

class targetDetectorCBObj:
    """
    A class for pipeline processing to locate patch targets in the
    field of view

    """

    def __init__(self, config):
        # robot_config       = config['robot_config']
        # min_radius         = config['subject_detector']['min_radius']
        # self.input_width   = config['subject_detector']['input_width']
        # self.input_height  = config['subject_detector']['input_height']
  
        self.logger = logging.getLogger('rovybot_demo.targetDetectorCB')

        self.patch_color = None

        # Used to show masks for color filter mask adjustments
        self._color_upper =  app_config.color_upper
        self._color_lower =  app_config.color_lower

        self.greenLower = np.array( \
            config['subject_detector']['drop_zone_config']['patch_colors']['green']['min'])
        self.greenUpper = np.array( \
            config['subject_detector']['drop_zone_config']['patch_colors']['green']['max'])

        self.blueLower = np.array( \
            config['subject_detector']['drop_zone_config']['patch_colors']['blue']['min'])
        self.blueUpper = np.array( \
            config['subject_detector']['drop_zone_config']['patch_colors']['blue']['max'])

        self.orangeLower = np.array( \
            config['subject_detector']['drop_zone_config']['patch_colors']['orange']['min'])
        self.orangeUpper = np.array( \
            config['subject_detector']['drop_zone_config']['patch_colors']['orange']['max'])

        self.min_width = config['subject_detector']['drop_zone_config']['min_width']

        self.roi = None
        self.roi_w = config['subject_detector']['drop_zone_config']['roi']['w']
        self.roi_h = config['subject_detector']['drop_zone_config']['roi']['h']
        self.roi_cx = config['subject_detector']['drop_zone_config']['roi']['cx']
        self.roi_cy = config['subject_detector']['drop_zone_config']['roi']['cy']
        self.roi_show_box = config['subject_detector']['drop_zone_config']['roi']['show_bounding_box']
        self.use_roi = config['subject_detector']['drop_zone_config']['use_roi']
        self.data_store    = config['subject_detector']['data_store']


    def __call__(self, img, reset=False):
        """
        A functor invoked by the tailend of the target patch detection OpenCV processing chain

        Args:
            img (image): Post-processed image from the OpenCV chain
            reset (bool): A flag to indicate if the detection data should be looked at.
        """
        if not self.roi:
            w = float(img.shape[1])
            h = float(img.shape[0])
            # compute the width and height of ROI based on percentage
            # of the image.
            b_w = int(w * self.roi_w)
            b_h = int(h * self.roi_h)
            
            # compute the center of the ROI based on percentage
            # of the image.
            b_cx = int(w * self.roi_cx)
            b_cy = int(h * self.roi_cy)

            # x1, y1, x2, y2
            self.roi = [int(b_cx-b_w/2), int(b_cy - b_h/2), int(b_cx+b_w/2), int(b_cy + b_h/2)]

        if (not self.use_roi):
            self.roi = [0, 0, img.shape[1], img.shape[0]]

        box = self.roi      
        
        img = self.is_patch_detected(img)

        if(self.roi_show_box and self.use_roi):
            self.overlay_bounding_box(img, box, "none")
        
    def overlay_bounding_box(self, frame, box, class_name):
        """
        draw bounding box at given co-ordinates.

        Args:
            frame (numpy array): Input image where the overlay should be drawn
            bbox : Bounding box co-ordinates in format [X1 Y1 X2 Y2]
            class_name : Name of the class to overlay
        """
        box_color = (20, 220, 20)
        cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), box_color, 2)

        return frame

    def detect_patch(self, img, colorLower, colorUpper):
        """
        Search an image for one patch in the range of color provided.

        returns: (x, y), radius where
            (x,y) are the center of the ball
            if no ball, radius  = None
        """

         # blur the image
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute see if it is rectangle
            c = max(cnts, key=cv2.contourArea)

            # Get the perimeter
            peri = cv2.arcLength(c, True)

            # Create an approximage polygon
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)

            # if the poly has 4 points, then we call it a rectangle
            if len(approx) == 4:
                shape = "rectangle"
                (x,y,w,h) = cv2.boundingRect(approx)
                ar = w/float(h)

                if(w > self.min_width):
                    return (x,y,w,h), w
    
        # Nothing found
        return (0,0,0,0), None

    def is_patch_detected(self, frame):
        """
        Search a portion of the frame for a patch with the specific
        colors.
        """
        if (self.use_roi):
            crop = frame[self.roi[1]:self.roi[3],self.roi[0]:self.roi[2]]
        else:
            crop = frame
        
        patch_color = None
        
        # Detect Green
        (x,y,w,h), width_green = self.detect_patch(crop, self.greenLower, self.greenUpper)

        if (width_green):
            # x,y are the center of the circle in the ROI
            # Translate this to frame:
            (x1, y1) = (int(self.roi[0]) + x, int(self.roi[1]+ y) )

            cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
            xp = x + w/2
            yp = y + w/2
            self.data_store.patch_position_green.set((xp, yp, w))
        else:
            self.data_store.patch_position_green.clear()

        # Detect Blue
        (x,y,w,h), width_blue = self.detect_patch(crop, self.blueLower, self.blueUpper)

        if (width_blue):
            # x,y are the center of the circle in the ROI
            # Translate this to frame:
            (x1, y1) = (int(self.roi[0]) + x, int(self.roi[1]+ y) )
            cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 0, 255), 2)
            xp = x + w/2
            yp = y + w/2
            self.data_store.patch_position_blue.set((xp, yp, w))
        else:
            self.data_store.patch_position_blue.clear()
            
        # Detect Orange
        (x,y,w,h), width_orange = self.detect_patch(crop, self.orangeLower, self.orangeUpper)

        if (width_orange):
            # x,y are the center of the circle in the ROI
            # Translate this to frame:
            (x1, y1) = (int(self.roi[0]) + x, int(self.roi[1]+ y) )
            cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (244, 99, 58), 2)
            xp = x + w/2
            yp = y + w/2
            self.data_store.patch_position_orange.set((xp, yp, w))
        else:
            self.data_store.patch_position_orange.clear()

        return frame

    def stop(self):
        # Release the robot object context
        print("{}: stopping".format(__name__))

    def __del__(self):
        self.stop()

# class targetDetectorCBObj
        
class CLI(cmd.Cmd):
    def __init__(self):
        super().__init__()

    def do_help(self, line):
        """List commands"""
        print("Help!")  # We may get around to this...  :-)

    def do_cl(self, color):
        """Set color range"""
        print("Setting lower color to {0}".format(color))
        
        # Stuff this into the config object
        color_list = color.split(',')
        try:
            app_config.color_lower = [int(x) for x in color_list]
        except:
            print("Invalid color entry.")

    def do_cu(self, color):
        """Set color range"""
        print("Setting upper color to {0}".format(color))

        # Stuff this into the config object
        color_list = color.split(',')
        try:
            app_config.color_upper =[int(x) for x in color_list]
        except:
            print("Invalid color entry.")

    def do_quit(self, line):
        """Exit the CLI"""
        print("Quitting CLI, press Ctrl-C to exit.")
        return True

    def do_exit(self, line):
        """Exit the CLI"""
        print("Quitting CLI, press Ctrl-C to exit.")
        return True


def main(sys_argv):
    """
    Main function implementing the application.
    """
    args = get_cmdline_args(sys_argv)

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    config['log_level'] = args.log_level
    config['subject_detector']['dump_dot'] = args.dump_dot

    logger = create_logger(name='rovybot_demo',
                       fileName="rovybot_demo.log",
                       level=config['log_level'])

    # If the display output is a kmssink, then stop weston
    weston_was_stopped=False
    if config['output']['sink'] == 'kmssink' :
        print("Stopping Weston...")
        subprocess.run(["systemctl", "stop", "weston"])
        weston_was_stopped=True

    # Set camera line frequency
    print("Stopping Weston...")
    subprocess.run(["v4l2-ctl", "-d", "/dev/v4l-tevs-subdev0","--set-ctrl","power_line_frequency=1"])
    subprocess.run(["v4l2-ctl", "-d", "/dev/v4l-tevs-subdev1","--set-ctrl","power_line_frequency=1"])
    subprocess.run(["v4l2-ctl", "-d", "/dev/v4l-tevs-subdev2","--set-ctrl","power_line_frequency=1"])
    subprocess.run(["v4l2-ctl", "-d", "/dev/v4l-tevs-subdev3","--set-ctrl","power_line_frequency=1"])

    #subprocess.run(["v4l2-ctl", "-d", "/dev/v4l-tevs-subdev2","--set-ctrl","saturation=8192"])

    subjectDetectorObj     = None
    subjectDetectorCBObj   = None
    ballDetectCBObj        = None
    patchDetectCBObj       = None 
 
    dataStore = None

    # Data Store
    dataStore = DataStore()

    # Stuff it into the config object so that the callbacks can all get to it
    config['subject_detector']['data_store'] = dataStore

    # If the gamepad is not connected, issue a warning and proceed
    try:
        # Launch the gamepad controller thread
        gamepad_control = GamepadMotorControl(config)
        gamepad_control.start()
    except Exception as e:
        print("WARNING:. During gamepad start exception:", e)
        gamepad_control = None

    try:

        # Create a callback object and register it with the processing chain
        subjectDetectorCBObj = subjectDetectorCallbackObj(config, gamepad_control)
        config['subject_detector']['callback_object_dl'] = subjectDetectorCBObj

        # Create an OpenCV Callback Object
        ballDetectCBObj = ballDetectorCBObj(config)
        config['subject_detector']['callback_object_opencv_ball_detect'] = ballDetectCBObj

        # Create an OpenCV Callback Object for the patch detector
        patchDetectCBObj = targetDetectorCBObj(config)
        config['subject_detector']['callback_object_opencv_patch_detect'] = patchDetectCBObj

        # Create the subjectDetectorObj object
        subjectDetectorObj = SubjectDetector(config['subject_detector'])

        # Start the demo
        subjectDetectorObj.start()

        # Create the cli object
        cli = CLI()

        cli.cmdloop()

        subjectDetectorObj.wait_for_exit()

    except KeyboardInterrupt:
        if(subjectDetectorObj):
            subjectDetectorObj.stop()
    finally:
        pass

    print("Here 1")

    if subjectDetectorObj:
        #subjectDetectorObj.stop()
        del subjectDetectorObj

    print("Here 2")

    if subjectDetectorCBObj:
        subjectDetectorCBObj.stop()
        del subjectDetectorCBObj

    print("Here 3")

    if ballDetectCBObj:
        ballDetectCBObj.stop()
        del ballDetectCBObj

    if patchDetectCBObj:
        patchDetectCBObj.stop()
        del patchDetectCBObj

    if gamepad_control:
        gamepad_control.stop()

    if weston_was_stopped :
        # If Weston was stopped before, then restart it
        subprocess.run(["systemctl", "start", "weston"])

if __name__ == "__main__":
    main(sys.argv)
