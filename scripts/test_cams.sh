#!/bin/bash

# Exposure, gain, and WB TEMP settings
EXPOSURE=34000
GAIN=35
WB_TEMP=3000

# Default - use auto WB
WB_MODE=1

# Default mode is mosaic mode
MOSAIC_MODE=1

# Default sink is waylandsink
SINK=waylandsink
KMSSINK=0

# default mode in manual exposure
AUTO_EXPOSURE=1

# Show ti perf overlay off by default
PERFOVERLAY=0

# Camera subdevs for use with v4l2-ctl
SUBDEVS=("0" "1" "2" "3")

# Camera I2C busses
I2C_BUSSES=("9" "10" "11" "12")

while getopts pkwhmgc: flag
do
    case "${flag}" in
        w) #Auto WB mode
            WB_MODE=0
            echo "Disable auto white balance"
            ;;
        m) # Manual exposure
            AUTO_EXPOSURE=0
            echo "Manual exposure mode"
            ;;
        c) # Single camera mode
            CAM=${OPTARG}
            echo Test camera /dev/video-tevs-cam$CAM
            MOSAIC_MODE=0
            ;;
        g) # Use glimagesink
            echo "Use glimagesink"
            SINK="glimagesink render-rectangle=<0,0,1280,800>"
            ;;
        k) # Use kmssink
            echo "Use kmssink"
            SINK="kmssink driver-name=tidss force-modesetting=true"
            KMSSINK=1
            ;;
        p) # Show perfoverlay
            echo "Show perfoverlay"
            PERFOVERLAY=1
            ;;
        h) # Print help
            echo "test_cams.sh -c [camera number] -w -h -m"
            ;;
    esac
done

if [ $AUTO_EXPOSURE == 0 ]; then

    # Set manual exposure
 #   echo Setting manual exposure for all cameras, gain=$GAIN, exposure=$EXPOSURE
    echo Setting auto exposure with limits
    for subdev in "${SUBDEVS[@]}"; do
#        v4l2-ctl -d /dev/v4l-tevs-subdev"${subdev}" --set-ctrl exposure_mode=0
        v4l2-ctl -d /dev/v4l-tevs-subdev"${subdev}" --set-ctrl exposure_mode=1
    done

    # Set manual exposure with auto gain
    #for i2cbus in "${I2C_BUSSES[@]}"; do
    #    i2ctransfer -y -f  $i2cbus w4@0x48 0x31 0x16 0X00 0X09
    #done

    # Set auto exposure max exposure time
    for i2cbus in "${I2C_BUSSES[@]}"; do
        i2ctransfer -y -f  $i2cbus w4@0x48 0x31 0x0c 0x00 0x00
        i2ctransfer -y -f  $i2cbus w4@0x48 0x31 0x0e 0x80 0xe8
        i2ctransfer -y -f  $i2cbus w4@0x48 0x31 0x10 0x00 0x00
        i2ctransfer -y -f  $i2cbus w4@0x48 0x31 0x12 0x80 0xe8
    done

#    for subdev in "${SUBDEVS[@]}"; do
#        v4l2-ctl -d /dev/v4l-tevs-subdev${subdev} --set-ctrl exposure=$EXPOSURE
#        v4l2-ctl -d /dev/v4l-tevs-subdev${subdev} --set-ctrl gain=$GAIN
#    done
else
    # Set auto exposure
    echo Setting automatic exposure for all cameras
    for subdev in "${SUBDEVS[@]}"; do
        v4l2-ctl -d /dev/v4l-tevs-subdev"${subdev}" --set-ctrl exposure_mode=1
    done
fi

for subdev in "${SUBDEVS[@]}"; do
    v4l2-ctl -d /dev/v4l-tevs-subdev${subdev} --set-ctrl white_balance_mode=$WB_MODE

    # Set white balance temperature.  If WB_MODE=1, this is ignored by the camera
    v4l2-ctl -d /dev/v4l-tevs-subdev${subdev} --set-ctrl white_balance_temperature=$WB_TEMP
done

if [ $KMSSINK == 1 ]; then
    # Disable weston
    systemctl stop weston
fi

if [ $PERFOVERLAY == 1 ] ; then
    # Add perfoverlay to sink
    SINK=" tiperfoverlay main-title=\"TechNexion Camera Test\" ! ${SINK}"
fi

echo Setting display sink to sink: $SINK

if [ $MOSAIC_MODE == 0 ]; then
    # Just a single camera with LDC
    gst-launch-1.0 v4l2src device=/dev/video-tevs-cam${CAM} io-mode=2 ! video/x-raw,width=1280,height=720,format=UYVY ! tiovxldc out-block-width=128 ! video/x-raw,format=NV12 ! ${SINK} sync=false
    
    # Dennis basic pipeline (no ldc, no io-mode, no format setting to UYVY)
    #gst-launch-1.0 v4l2src device=/dev/video-tevs-cam${CAM} ! video/x-raw,width=1280,height=720 ! ${SINK} sync=false

else
    #2x2 Mosaic, default
    gst-launch-1.0 \
    v4l2src device=/dev/video-tevs-cam3 io-mode=2 ! video/x-raw,width=1280,height=720,format=UYVY ! tiovxldc out-block-width=128 ! video/x-raw,format=NV12 ! tiovxmultiscaler target=0 ! video/x-raw, width=640, height=400  ! mosaic.sink_3 \
    v4l2src device=/dev/video-tevs-cam2 io-mode=2 ! video/x-raw,width=1280,height=720,format=UYVY ! tiovxldc out-block-width=128 ! video/x-raw,format=NV12 ! tiovxmultiscaler target=0 ! video/x-raw, width=640, height=400  ! mosaic.sink_2 \
    v4l2src device=/dev/video-tevs-cam1 io-mode=2 ! video/x-raw,width=1280,height=720,format=UYVY ! tiovxldc out-block-width=128 ! video/x-raw,format=NV12 ! tiovxmultiscaler target=1 ! video/x-raw, width=640, height=400  ! mosaic.sink_1 \
    v4l2src device=/dev/video-tevs-cam0 io-mode=2 ! video/x-raw,width=1280,height=720,format=UYVY ! tiovxldc out-block-width=128 ! video/x-raw,format=NV12 ! tiovxmultiscaler target=1 ! video/x-raw, width=640, height=400  ! mosaic.sink_0 \
    tiovxmosaic name=mosaic \
    latency=12345678900 \
    src::pool-size=12 \
    sink_0::startx="<0>" sink_0::starty="<0>" \
    sink_1::startx="<640>" sink_1::starty="<0>" \
    sink_2::startx="<0>" sink_2::starty="<400>" \
    sink_3::startx="<640>" sink_3::starty="<400>" ! \
    ${SINK} sync=false

    # gst-launch-1.0 \
    # v4l2src device=/dev/video-tevs-cam3 io-mode=2 ! video/x-raw,width=1280,height=720,format=UYVY ! tiovxldc out-block-width=128 ! video/x-raw,format=NV12 ! tiovxmultiscaler target=0 ! video/x-raw, width=640, height=400  ! mosaic.sink_3 \
    # v4l2src device=/dev/video-tevs-cam2 io-mode=2 ! video/x-raw,width=1280,height=720,format=UYVY ! tiovxldc out-block-width=128 ! video/x-raw,format=NV12 ! tiovxmultiscaler target=0 ! video/x-raw, width=640, height=400  ! mosaic.sink_2 \
    # v4l2src device=/dev/video-tevs-cam1 io-mode=2 ! video/x-raw,width=1280,height=720,format=UYVY ! tiovxldc out-block-width=128 ! video/x-raw,format=NV12 ! tiovxmultiscaler target=1 ! video/x-raw, width=640, height=400  ! mosaic.sink_1 \
    # v4l2src device=/dev/video-tevs-cam0 io-mode=2 ! video/x-raw,width=1280,height=720,format=UYVY ! tiovxldc out-block-width=128 ! video/x-raw,format=NV12 ! tiovxmultiscaler target=1 ! video/x-raw, width=640, height=400  ! mosaic.sink_0 \
    # compositor name=mosaic latency=1000000000 background=0 \
    # sink_0::xpos=0 sink_0::ypos= 0 \
    # sink_1::xpos=640 sink_1::ypos= 0 \
    # sink_2::xpos=0 sink_2::ypos= 400 \
    # sink_3::xpos=640 sink_3::ypos= 400 ! \
    # video/x-raw, width=1280, height=800,framerate=30/1 ! queue ! \
    # ${SINK} sync=false

fi

