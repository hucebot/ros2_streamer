# python
import time

# Gstreamer
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst #,GstRtp,GstRtsp,GObject
import subprocess

# ROS2
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import SetParametersResult

#import yaml
#import sys

class GstreamerService(Node):

    def __init__(self):
        print('---')
        super().__init__('gstreamer_service')
        print('init')
        self.get_logger().info('Gstreamer service is starting...')

        self.declare_parameter('rtp_port', 5000)
        self.declare_parameter('rtp_dest', '192.168.1.116')
        self.declare_parameter('device', '/dev/video0')
        
        self.declare_parameter('ntp_server', 'time.apple.com')

        self.declare_parameter('bitrate', 1000)
        
        self.declare_parameter('local_time_frequency', 10)
        self.add_on_set_parameters_callback(self.parameters_callback)

        # topic : clock
        self.ntp_sync(self.get_parameter('ntp_server').get_parameter_value().string_value)
        self.pub_clock = self.create_publisher(Time, '/gstreamer_service/local_time', 1)
        self.timer_pub_clock = self.create_timer(1.0 / 10.0, self.publish_time)

        self._init_gstreamer()
        self.need_reinit = False
        self.timer_reinit = self.create_timer(1.0 / 50.0, self.reinit)

        self.get_logger().info('Gstreamer service is started')

    def ntp_sync(self, server):
        print('syncing clock (this might take a few secs...)')
        subprocess.Popen(['ntpdate', server])

    def publish_time(self):
        # https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Time.html
        msg = Time()
        t = time.time_ns()
        msg.sec = int(t / 1_000_000_000)
        msg.nanosec = t - msg.sec * 1_000_000_000
        self.pub_clock.publish(msg)

    def reinit(self):
        if self.need_reinit:
            self.need_reinit = False
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self._init_gstreamer()

    def parameters_callback(self, params):
        for p in params:
            if p.name == 'local_time_frequency':
                f = p.value
                self.timer_pub_clock.destroy()        
                self.timer_pub_clock = self.create_timer(1.0 / f, self.publish_time)
                self.get_logger().info(f'Setting local time frequency to {f} Hz')
            elif p.name == 'ntp_server':
                self.ntp_sync(p.value)
            else:
                self.need_reinit = True
        return SetParametersResult(successful=True)


    def _init_gstreamer(self):
        self.get_logger().info('Initializing gstreamer...')

        device = self.get_parameter('device').get_parameter_value().string_value
        rtp_dest = self.get_parameter('rtp_dest').get_parameter_value().string_value
        rtp_port = self.get_parameter('rtp_port').get_parameter_value().integer_value
        
        pipeline_string = f"v4l2src device={device} ! video/x-h264,width=1280,height=720 \
        ! rtph264pay ! queue \
        ! udpsink host={rtp_dest} port={rtp_port} sync=true async=false"

        print("Gstreamer pipeline:", pipeline_string.replace("\n"," "))
        self.pipeline = None
        try:
            Gst.init(None)
            self.pipeline = Gst.parse_launch(pipeline_string)
            self.pipeline.set_state(Gst.State.PLAYING)
        except Exception as e:
            print("\nERROR launching GSTREAMER. Check the GST_PLUGIN_PATH.\n")
            print("Error received:")
            print(e)
            return
        self.get_logger().info('Gstreamer OK')



def main():
    print("main")
    rclpy.init()
    service = GstreamerService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


