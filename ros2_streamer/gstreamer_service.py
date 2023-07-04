# python
import time

# Gstreamer
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject#,GstRtp,GstRtsp,GObject
import subprocess
import sys

# ROS2
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64

# v4l2
import v4l2
import fcntl

#import yaml
#import sys

class GstreamerService(Node):

    def __init__(self):
        print('---')
        super().__init__('gstreamer_service')

        print('init')
        self.get_logger().info('Gstreamer service is starting...')

        # parameters
        self.declare_parameter('rtp_port', 5000)
        self.declare_parameter('rtp_dest', '192.168.1.141')
        self.declare_parameter('device', '/dev/video4')
        self.declare_parameter('ntp_server', 'time.apple.com')
        self.declare_parameter('bitrate', 1000)
        self.declare_parameter('local_time_frequency', 10)
        self.declare_parameter('pan_tilt_frequency', 10)
        self.add_on_set_parameters_callback(self.parameters_callback)


        # ini video for linux
        self.is_pantilt = False
        self._init_v4l() # this is only for pan-tilt support

        # ros needs to know if pan-tilt 
        self._init_ros()

        # init gstreamer
        self._init_gstreamer()
        self.need_reinit = False
        self.timer_reinit = self.create_timer(1.0 / 50.0, self.reinit)

        self.get_logger().info('Gstreamer ROS2 service is started')

    def _init_ros(self):
        device = self.get_parameter('device').get_parameter_value().string_value.split('/')[-1]

        # publisher : clock
        self.ntp_sync(self.get_parameter('ntp_server').get_parameter_value().string_value)
        self.pub_clock = self.create_publisher(Time, f'/{device}/local_time', 1)
        local_time_frequency = self.get_parameter('local_time_frequency').get_parameter_value().integer_value
        self.timer_pub_clock = self.create_timer(1.0 / local_time_frequency, self.publish_time)

        # topics are not created if this is not a pan/tilt camera
        if self.is_pantilt:
            # subscriber : pan/tilt
            self.sub_pan = self.create_subscription(Float64, f'/{device}/pan', self.pan_callback, 1)
            self.sub_tilt = self.create_subscription(Float64, f'/{device}/tilt', self.tilt_callback, 1)

            self.pub_pan = self.create_publisher(Float64, f'/{device}/current_pan', 1)
            self.pub_tilt = self.create_publisher(Float64, f'/{device}/current_tilt', 1)
            pan_tilt_frequency = self.get_parameter('pan_tilt_frequency').get_parameter_value().integer_value
            self.timer_pub_pan = self.create_timer(1.0 / pan_tilt_frequency, self.publish_pan)
            self.timer_pub_tilt = self.create_timer(1.0 / pan_tilt_frequency, self.publish_tilt)

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

    def reinit(self, force=False):
        if self.need_reinit or force:
            self.need_reinit = False
            self.pipeline.send_event(Gst.Event.new_eos())
            self.pipeline.set_state(Gst.State.NULL)
            time.sleep(0.1)
            del self.pipeline
            self.pipeline = None
            self._init_gstreamer()

    def parameters_callback(self, params):
        for p in params:
            if p.name == 'local_time_frequency':
                f = p.value
                self.timer_pub_clock.destroy()        
                self.timer_pub_clock = self.create_timer(1.0 / f, self.publish_time)
                self.get_logger().info(f'Setting local time frequency to {f} Hz')
            elif p.name == 'pan_tilt_frequency':
                pan_tilt_frequency = p.value
                self.timer_pub_pan.destroy()
                self.timer_pub_tilt.destroy()
                self.timer_pub_pan = self.create_timer(1.0 / pan_tilt_frequency, self.publish_pan)
                self.timer_pub_tilt = self.create_timer(1.0 / pan_tilt_frequency, self.publish_pan)
            elif p.name == 'ntp_server':
                self.ntp_sync(p.value)
            elif p.name == 'device':
                self._init_ros()
                self._init_v4l()
                self.need_reinit = True
            else:
                self.need_reinit = True
        return SetParametersResult(successful=True)


    def publish_pan(self):
        self._publish_v4l(self.pub_pan, v4l2.V4L2_CID_PAN_ABSOLUTE)

    def publish_tilt(self):
        self._publish_v4l(self.pub_tilt, v4l2.V4L2_CID_TILT_ABSOLUTE)

    def _publish_v4l(self, topic, cmd):
        control = v4l2.v4l2_control()
        control.id = cmd
        fcntl.ioctl(self.video_device, v4l2.VIDIOC_G_CTRL, control)
        msg = Float64()
        msg.data = float(control.value)
        topic.publish(msg)

    def pan_callback(self, msg):
        if not self.is_pantilt:
            self.get_logger().error("No pan/tilt capability!")
            return

        control = v4l2.v4l2_control()
        control.id = v4l2.V4L2_CID_PAN_ABSOLUTE
        if msg.data > 100:
            msg.data = 100
            self.get_logger().info("pan must be in [-100%, 100%]")
        if msg.data < -100:
            msg.data = -100
            self.get_logger().info("pan must be in [-100%, 100%]")
        if msg.data < 0:
            control.value = int((round(float(msg.data) / 100. * (-self.min_pan)) / self.step_tilt) * self.step_pan)
        else:
            control.value = int((round(float(msg.data) / 100. * (self.max_pan)) / self.step_tilt) * self.step_pan)

        print("setting pan to", control.value)        
        error = fcntl.ioctl(self.video_device, v4l2.VIDIOC_S_CTRL, control)

    def tilt_callback(self, msg):
        if not self.is_pantilt:
            self.get_logger().error("No pan/tilt capability!")
            return

        control = v4l2.v4l2_control()
        control.id = v4l2.V4L2_CID_TILT_ABSOLUTE
        if msg.data > 100:
            msg.data = 100
            self.get_logger().info("tilt must be in [-100%, 100%]")
        if msg.data < -100:
            msg.data = -100
            self.get_logger().info("tilt must be in [-100%, 100%]")
        print('mintilt:', self.min_tilt)
        if msg.data < 0:
            control.value = int((round(float(msg.data) / 100. * (-self.min_tilt)) / self.step_tilt) * self.step_tilt)
        else:
            control.value = int((round(float(msg.data) / 100. * (self.max_tilt)) / self.step_tilt) * self.step_tilt)

        print("setting tilt to", control.value)        
        error = fcntl.ioctl(self.video_device, v4l2.VIDIOC_S_CTRL, control)


    # these are the Gstreamer messages (errors)
    def check_messages(self):
        message = self.bus.timed_pop_filtered(10000000, Gst.MessageType.ERROR |Gst.MessageType.STATE_CHANGED| Gst.MessageType.EOS | Gst.MessageType.ELEMENT |  Gst.MessageType.BUFFERING)
        #print("state:", self.pipeline.get_state())
        if message == None:
            return True
        
        #print("message", message.type)
        if message.type == Gst.MessageType.BUFFERING:
            print('buffering')
        if message.type == Gst.MessageType.EOS:
            self.pipeline.set_state(Gst.State.NULL)
            print("END OF STREAM")
            return False
        elif message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Gstreamer Error: {err} ", debug)
            self.pipeline.set_state(Gst.State.NULL)
            #sys.exit(1)
            return False
        elif message.type == Gst.MessageType.STATE_CHANGED:
            old_state, new_state, pending_state = message.parse_state_changed()
           # print("State changed from {} to {}".format(old_state, new_state))
        elif message.type  == Gst.MessageType.ELEMENT:
            src_element = message.src
            #print('element')
        return True

    def _init_v4l(self):
        self.get_logger().info("initializing v4l2...")

        try:
            device = self.get_parameter('device').get_parameter_value().string_value
            self.video_device = open(device, 'w')
            cp = v4l2.v4l2_capability()
            fcntl.ioctl(self.video_device, v4l2.VIDIOC_QUERYCAP, cp)
            print(cp.driver, cp.card,  cp.capabilities)

            if cp.capabilities & v4l2.V4L2_CID_PAN_ABSOLUTE and cp.capabilities & v4l2.V4L2_CID_TILT_ABSOLUTE:
                self.is_pantilt = True
            
                control = v4l2.v4l2_queryctrl()
                control.id = v4l2.V4L2_CID_PAN_ABSOLUTE
                fcntl.ioctl(self.video_device, v4l2.VIDIOC_QUERYCTRL, control)
                self.min_pan = control.minimum
                self.max_pan = control.maximum
                self.step_pan = control.step

                control = v4l2.v4l2_queryctrl()
                control.id = v4l2.V4L2_CID_TILT_ABSOLUTE
                fcntl.ioctl(self.video_device, v4l2.VIDIOC_QUERYCTRL, control)
                self.min_tilt = control.minimum
                self.max_tilt = control.maximum
                self.step_tilt = control.step

        except Exception as e:
            print("ERROR initializng v4l2", e)

    def _init_gstreamer(self):
        self.get_logger().info('Initializing gstreamer...')
        device = self.get_parameter('device').get_parameter_value().string_value

        rtp_dest = self.get_parameter('rtp_dest').get_parameter_value().string_value
        rtp_port = self.get_parameter('rtp_port').get_parameter_value().integer_value
        
        pipeline_string = f"v4l2src device={device}  ! video/x-h264,width=1280,height=720 \
        ! rtph264pay  \
        ! udpsink host={rtp_dest} port={rtp_port} sync=true async=false"

        print("Gstreamer pipeline:", pipeline_string.replace("\n"," "))
        self.pipeline = None
        try:
            Gst.init(None)
            self.pipeline = Gst.parse_launch(pipeline_string)
            self.bus = self.pipeline.get_bus()
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
#    rclpy.spin(service)
    while True:
            if not service.check_messages():
                time.sleep(1.0)
                service.reinit(True)
                time.sleep(1.0)
            rclpy.spin_once(service)
            time.sleep(0.001)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


