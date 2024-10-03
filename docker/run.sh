#!/bin/bash

docker run  --name ros2_streamer  --net host --privileged -v $HOME/ros2_ws:/ros2_ws -w /ros2_ws ros2_streamer:2024.05
