# ros2_streamer
A gstreamer launch interface with ROS2 services

## Parameters
- bitrate
- local_time_frequency
- rtp_host
- rtp_port
- source
- (use_sim_time)

### Default values 
- bitrate: 1000
- local_time_frequency: 10
- rtp_host: 192.168.1.141
- rtp_port: 5000
- source: videotestsrc pattern=ball is-live=true
- use_sim_time: false
