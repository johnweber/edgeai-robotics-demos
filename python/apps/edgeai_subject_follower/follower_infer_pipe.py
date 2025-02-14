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

"""EdgeAI Inference Pipe

This script provides the logic for the EdgeAI inference pipeline setup.

"""

from time import time
import threading
from follower_post_process import PostProcess

class InferPipe:
    """
    Class to abstract the threading of multiple inference pipelines
    """
    def __init__(self, sub_flow, gst_pipe, cbObj):
        """
        Constructor to create an InferPipe object.
        Args:
            sub_flow: sub_flow configuration
            gst_pipe: gstreamer pipe object
        """
        self.sub_flow = sub_flow
        self.gst_pipe = gst_pipe
        self.gst_pre_inp = gst_pipe.get_src(sub_flow.gst_pre_src_name, \
                                                               sub_flow.flow.id)
        self.gst_sen_inp = gst_pipe.get_src(sub_flow.gst_sen_src_name, \
                                                               sub_flow.flow.id)
        self.run_time = sub_flow.model.run_time
        self.post_proc = PostProcess.get(sub_flow, cbObj)
        self.gst_post_out = gst_pipe.get_sink(sub_flow.gst_post_sink_name, \
                                              sub_flow.width,sub_flow.height, \
                                              sub_flow.input.fps)
        self.param = sub_flow.model
        self.pipeline_thread = threading.Thread(target=self.pipeline)
        self.stop_thread = False

    def start(self):
        """
        Start the pipeline
        """
        self.pipeline_thread.start()

    def stop(self):
        """
        Stop the pipeline
        """
        print("Stopping infer pipe...\n")
        self.stop_thread = True

    def wait_for_exit(self):
        """
        Waiting for exit, to be called by parent thread
        """
        self.pipeline_thread.join()

    def pipeline(self):
        """
        Callback function for pipeline thread
        """
        while self.stop_thread == False:
            #capture and pre-process
            input_img = self.gst_pipe.pull_tensor(
                self.gst_pre_inp,
                self.sub_flow.input.loop,
                self.sub_flow.model.crop[0],
                self.sub_flow.model.crop[1],
                self.sub_flow.model.data_layout,
                self.sub_flow.model.input_tensor_types[0],
            )
            if (type(input_img) == type(None)):
                break
            #Inference
            start = time()
            result = self.run_time(input_img)
            end = time()
            self.sub_flow.report.report_proctime('dl-inference', (end - start))
            #post-process
            frame = self.gst_pipe.pull_frame(self.gst_sen_inp, \
                                                       self.sub_flow.input.loop)
            if (type(frame) == type(None)):
                break
            out_frame = self.post_proc(frame, result)
            self.gst_pipe.push_frame(out_frame, self.gst_post_out)
            #Increment frame count
            self.sub_flow.report.report_frame()

        self.stop_thread = False
        self.gst_pipe.send_eos(self.gst_post_out)
        
