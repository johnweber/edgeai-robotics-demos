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

"""OpenCV post-processing

This script provides the post-processing logic for the OpenCV pipelines

"""

from classnames import *
import cv2
import numpy as np
import copy
import debug
import imutils
import app_config

np.set_printoptions(threshold=np.inf, linewidth=np.inf)

def create_title_frame(title, width, height):
    frame = np.zeros((height, width, 3), np.uint8)
    frame = cv2.putText(frame, "OpenCV", \
                    (40, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 2)
    frame = cv2.putText(frame, title, (40, 70), \
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    return frame

def overlay_model_name(frame, model_name, start_x, start_y, width, height):
    row_size = 40 * width//1280
    font_size = width/1280
    cv2.putText(frame, "Model : " + model_name, \
                (start_x + 5, start_y - row_size//4), cv2.FONT_HERSHEY_SIMPLEX, \
                font_size, (255, 255, 255), 2)
    return frame

class OpenCVPostProcess:
    """
    Class to create a post process context
    """
    def __init__(self, flow):
        self.flow = flow
        self.model = flow.model
        self.debug = None
        self.debug_str = ""
        if flow.debug_config and flow.debug_config.post_proc:
            self.debug = debug.Debug(flow.debug_config, "post")

    def get(flow, cbObj):
        """
        Create a object of a subclass based on the task type
        """
        if (flow.model.task_type == "opencv"):
            return OpenCVPostProcessTracking(flow, cbObj)
        else:
            return None

class OpenCVPostProcessTracking(OpenCVPostProcess):
    def __init__(self, flow, cbObj):
        super().__init__(flow)
        
        self.flow = flow

        self.cbObj = cbObj
 
    def __call__(self, img):
        """
        Post process function for detection
        Args:
            img: Input frame
        """
        self.cbObj(img)
        
        if self.debug:
            self.debug.log(self.debug_str)
            self.debug_str = ""

        return img

    


