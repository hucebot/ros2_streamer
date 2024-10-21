# python
import time
import signal

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
        name = 'gstreamer_service'
        super().__init__(name)

        print('init')
        self.get_logger().info('Gstreamer service is starting...')

        # parameters
        self.declare_parameter('camera_name', 'insta360')
        self.declare_parameter('pan_tilt', False)
        self.declare_parameter('rtp_port', 5000)
        self.declare_parameter('rtp_dest', '192.168.50.16') #192.168.1.117
        self.declare_parameter('device', '/dev/video1')
        self.declare_parameter('ntp_server', 'time.apple.com')
        self.declare_parameter('bitrate', 1000)
        self.declare_parameter('local_time_frequency', 10)
        self.declare_parameter('pan_tilt_frequency', 10)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.is_pantilt = False
        if self.get_parameter('pan_tilt').get_parameter_value().bool_value == True:
            # ini video for linux
            self.is_pantilt = True
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
        name = self.get_name().replace("_"+self.get_parameter('camera_name').get_parameter_value().string_value, "")

        # publisher : clock
        self.ntp_sync(self.get_parameter('ntp_server').get_parameter_value().string_value)
        self.pub_clock = self.create_publisher(Time, f'/{name}/local_time', 1)
        local_time_frequency = self.get_parameter('local_time_frequency').get_parameter_value().integer_value
        self.timer_pub_clock = self.create_timer(1.0 / local_time_frequency, self.publish_time)

        # topics are not created if this is not a pan/tilt camera
        if self.is_pantilt:
            # subscriber : pan/tilt
            self.sub_pan = self.create_subscription(Float64, f'/{name}/pan', self.pan_callback, 1)
            self.sub_tilt = self.create_subscription(Float64, f'/{name}/tilt', self.tilt_callback, 1)

            self.pub_pan = self.create_publisher(Float64, f'/{name}/current_pan', 1)
            self.pub_tilt = self.create_publisher(Float64, f'/{name}/current_tilt', 1)
            pan_tilt_frequency = self.get_parameter('pan_tilt_frequency').get_parameter_value().integer_value
            self.timer_pub_pan = self.create_timer(1.0 / pan_tilt_frequency, self.publish_pan)
            self.timer_pub_tilt = self.create_timer(1.0 / pan_tilt_frequency, self.publish_tilt)
        else:
            self.get_logger().warning("No pan/tilt detected")

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

    def stop_node(self, **args):
        self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = None

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

        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        if self.get_parameter('camera_name').get_parameter_value().string_value == 'insta360':
            
            pipeline_string = f"v4l2src device={device}  ! video/x-h264,width={width},height={height} \
            ! rtph264pay  \
            ! udpsink host={rtp_dest} port={rtp_port} sync=true async=false"
        
        if self.get_parameter('camera_name').get_parameter_value().string_value == 'c930e':

            pipeline_string = f"v4l2src device={device}  ! image/jpeg,width={width},height={height} \
                ! jpegdec ! videoconvert ! x264enc tune=zerolatency  speed-preset=superfast \
                ! rtph264pay ! udpsink host={rtp_dest} port={rtp_port}"

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
    # signals
    def sighandler(*args):
        service.stop_node()
        service.destroy_node()
        sys.exit(0)
#        rclpy.shutdown()
    signal.signal(signal.SIGINT, sighandler)
    signal.signal(signal.SIGTERM, sighandler)
    while True:
            if not service.check_messages(): # gstreamer error
                time.sleep(0.2)
                service.reinit(True)
                time.sleep(0.2)
            rclpy.spin_once(service)
            time.sleep(0.001)


if __name__ == '__main__':
    main()


