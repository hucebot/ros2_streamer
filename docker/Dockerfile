FROM ros:humble
ENV ROS_DISTRO humble

LABEL maintainer="jean-baptiste.mouret@inria.fr"


ENV DEBIAN_FRONTEND="noninteractive" TZ="Europe/Paris"

RUN  apt -q -qq update && apt install -y --allow-unauthenticated \
  gstreamer1.0-plugins-good \ 
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-rtp \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-tools \
  python3-gst-1.0 \
  net-tools \
  ntpdate
  
RUN apt install python-is-python3

RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc



CMD ["bash"]
