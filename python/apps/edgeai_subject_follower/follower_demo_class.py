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

"""EdgeAI based follower demo class

This demo implements the core follower demo application logic.

"""

import sys
import config_parser
import gst_wrapper
from gst_element_map import gst_element_map
from edgeai_dl_inferer import ModelConfig
from follower_infer_pipe import InferPipe
import time

class FollowerDemo:
    """
    Abstract the functionality required for the Edge AI demo.
    Creates Input, Model, Output and Flow objects. Sets up infer pipes
    for each flow and starts the infer pipes
    """

    C7_CORE_ID_INDEX = 0

    def __init__(self, config):
        """
        Constructor of FollowerDemo class
        Args:
            config: Dictionary of params pased from config file
        """
        self.config = config
        self.models = {}
        self.inputs = {}
        self.outputs = {}
        self.flows = []
        self.infer_pipes = []
        self.title = config['title']
        for f in config['flows']:
            flow = config['flows'][f]

            input = flow[0]
            model = flow[1]
            output = flow[2]

            # Parse Input/Model/Output Objects
            if model not in self.models:
                model_config =  config["models"][model]
                model_path = model_config["model_path"]
                # Make model Config. This class is present in edgeai_dl_inferer
                enable_tidl = False
                core_id = 1
                if (gst_element_map['inferer']['target'] == 'dsp'):
                    enable_tidl = True
                    if 'core-id' in gst_element_map['inferer']:
                        core_id = gst_element_map['inferer']['core-id'][FollowerDemo.C7_CORE_ID_INDEX]
                        FollowerDemo.C7_CORE_ID_INDEX += 1
                        if FollowerDemo.C7_CORE_ID_INDEX >= len(gst_element_map['inferer']['core-id']):
                            FollowerDemo.C7_CORE_ID_INDEX = 0
                elif (gst_element_map['inferer']['target'] != 'arm'):
                    print("[WARNING] Invalid target specified for inferer. Defaulting to ARM.")

                model_obj = ModelConfig(model_path,enable_tidl,core_id)

                # Initialize the runtime
                model_obj.create_runtime()

                # task specific params
                if "alpha" in model_config:
                    model_obj.alpha = model_config["alpha"]
                if "viz_threshold" in model_config:
                    model_obj.viz_threshold = model_config["viz_threshold"]
                if "topN" in model_config:
                    model_obj.topN = model_config["topN"]

                self.models[model] = model_obj

            if input not in self.inputs:
                input_config = config["inputs"][input]
                input_obj = config_parser.Input(input_config)
                input_obj.name = input
                self.inputs[input] = input_obj

            if output not in self.outputs:
                output_config = config["outputs"][output]
                output_obj = config_parser.Output(output_config, self.title)
                self.outputs[output] = output_obj

           # Set mosaic and start bg_pipeline
            if len(flow) > 3 and flow[3] and not self.outputs[output].mosaic:
                self.outputs[output].set_mosaic()

        #Check if debug is enabled
        if "debug" in config:
            if not "enable_mask" in config["debug"]:
                print("[ERROR] enable_mask needs to be set if debug is enabled.")
                sys.exit()
            debug_config = config["debug"]
        else:
            debug_config = None

        for input in self.inputs:
            input_obj = self.inputs[input]
            subflow_dictionary = {}
            subflow_list = []
            """
            Loop over the flow to group configuration by common
            input and followed by common model.

            Ex: [   input0,
                    [
                      [model0 , [output0,output1] , [mosaic,None], [None,debug]]
                      [model1 , [output0] , [mosaic], [debug]]
                    ]
                ]
            """
            for i in config["flows"]:
                flow = config["flows"][i]
                if flow[0] != input:
                    continue

                model = flow[1]
                output = flow[2]
                mosaic_info = None
                if len(flow) > 3:
                    mosaic_info = flow[3]

                if model not in subflow_dictionary:
                    subflow_dictionary[model] = [
                        [self.outputs[output]],
                        [mosaic_info]
                        ]
                else:
                    subflow_dictionary[model][0].append(self.outputs[output])
                    subflow_dictionary[model][1].append(mosaic_info)

            for model in subflow_dictionary:
                model_obj = self.models[model]
                output_objs, mosaic_list = subflow_dictionary[model]
                subflow_list.append([model_obj, output_objs, mosaic_list])

            self.flows.append(config_parser.Flow(input_obj, subflow_list, debug_config))

        self.src_pipes, self.sink_pipe = gst_wrapper.get_gst_pipe(
            self.flows, self.outputs
        )
        self.gst_pipe = gst_wrapper.GstPipe(self.src_pipes, self.sink_pipe)

        for o in self.outputs.values():
            o.gst_pipe = self.gst_pipe

        cbObj = config['callback_object']
        for f in self.flows:
            for s in f.sub_flows:
                self.infer_pipes.append(InferPipe(s, self.gst_pipe, cbObj))

    def start(self):
        """
        Member function to start the demo
        """
        self.gst_pipe.start()
        for i in self.infer_pipes:
            i.start()

    def wait_for_exit(self):
        while (1):
            if all(i.stop_thread for i in self.infer_pipes):
               self.stop()
               break
        time.sleep(1)

    def stop(self):
        # Issue stop commands to the inference pipes
        
        print("Stopping demo...\n")

        for i in self.infer_pipes:
            i.stop()

        self.gst_pipe.free()

        # Hack del for model_obj is not called since refcount is not 1 here.
        for _,model_obj in self.models.items():
            del model_obj.run_time
