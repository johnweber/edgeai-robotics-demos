#  Copyright (C) 2024 TechNexion Ltd - http://www.technexion.com/

"""OpenCV Pipe

This script provides the logic for the OpenCV pipeline setup.

"""

from time import time
import threading
from opencv_pipe_post_process import OpenCVPostProcess

class OpenCVPipe:
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

        # Get the source. 
        self.gst_sen_inp = gst_pipe.get_src(sub_flow.gst_sen_src_name, \
                                                               sub_flow.flow.id)
        self.post_proc = OpenCVPostProcess.get(sub_flow, cbObj)

        # Get the sink
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

            #capture
            frame = self.gst_pipe.pull_frame(self.gst_sen_inp, self.sub_flow.input.loop)
            if (type(frame) == type(None)):
                break

            # Process frame
            out_frame = self.post_proc(frame)

            # Push
            self.gst_pipe.push_frame(out_frame, self.gst_post_out)

            #Increment frame count
            self.sub_flow.report.report_frame()

        self.stop_thread = False
        self.gst_pipe.send_eos(self.gst_post_out)
        
