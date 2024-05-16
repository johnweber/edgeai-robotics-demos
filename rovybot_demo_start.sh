!/bin/bash

# Start AP
/usr/bin/startwlanap.sh

# Setup cameras
/opt/robot/edgeai-robotics-demos/scripts/setup_cameras.sh

# Start the rovybot demo application
/opt/robot/edgeai-robotics-demos/python/apps/rovybot_demo/rovybot_demo.py /opt/robot/edgeai-robotics-demos/python/apps/rovybot_demo/rovybot_demo_config.yaml
 
