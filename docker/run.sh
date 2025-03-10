#!/bin/bash

docker rm ros2_streamer
docker run  \
    --name ros2_streamer  \
    --net host \
    --ipc host \
    --pid host \
    --privileged \
    -it \
    -v /dev:/dev \
    -v `pwd`/../../ros2_streamer/:/ros2_ws/src/ros2_streamer \
    -w /ros2_ws \
    ros2_streamer
