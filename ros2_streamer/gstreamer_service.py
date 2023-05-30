# python
import time


# ROS2
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import SetParametersResult

# Gstreamer
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst #,GstRtp,GstRtsp,GObject
#import yaml
#import sys

class GstreamerService(Node):

    def __init__(self):
        super().__init__('gstreamer_service')
        self.get_logger().info('Gstreamer service is starting...')

        self.declare_parameter('rtp_port', 5000)
        self.declare_parameter('rtp_host', '192.168.1.141')
        self.declare_parameter('source', 'videotestsrc pattern=ball is-live=true')
        self.declare_parameter('bitrate', 1000)
        
        self.declare_parameter('local_time_frequency', 10)
        self.add_on_set_parameters_callback(self.parameters_callback)

        # service : restart
        # service : ntp

        # topic : clock
        self.pub_clock = self.create_publisher(Time, '/gstreamer_service/local_time', 1)
        self.timer_pub_clock = self.create_timer(1.0 / 10.0, self.publish_time)

        self._init_gstreamer()

        self.get_logger().info('Gstreamer service is started')

    def publish_time(self):
        # https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Time.html
        msg = Time()
        t = time.time_ns()
        msg.sec = int(t / 1_000_000_000)
        msg.nanosec = t - msg.sec * 1_000_000_000
        self.pub_clock.publish(msg)


    def parameters_callback(self, params):
        for p in params:
            if p.name == 'local_time_frequency':
                f = p.value
                self.timer_pub_clock.destroy()        
                self.timer_pub_clock = self.create_timer(1.0 / f, self.publish_time)
                self.get_logger().info(f'Setting local time frequency to {f} Hz')
            else:
                self._init_gstreamer()
        return SetParametersResult(successful=True)


    def _init_gstreamer(self):
        self.get_logger().info('Initializing gstreamer...')

        source = self.get_parameter('source').get_parameter_value().string_value
        rtp_host = self.get_parameter('rtp_host').get_parameter_value().string_value
        rtp_port = self.get_parameter('rtp_port').get_parameter_value().integer_value

        pipeline_string = f"{source} \
        ! x264enc tune=zerolatency bitrate=100 speed-preset=superfast \
        ! h264parse \
        ! rtph264pay ! queue \
        ! udpsink host={rtp_host} port={rtp_port} sync=true async=false"

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


