import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import os
import sys
sys.path.append('/opt/nvidia/deepstream/deepstream/lib')

import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst

import pyds

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import SetBool
from cv_bridge import CvBridge

import time
import numpy as np
import cv2
import math


class DataCapture(Node):

    def __del__(self):
        self.file.close()

    def __init__(self):
        super().__init__('data_capture')

        GObject.threads_init()
        Gst.init(None)

        # Gstreamerランチャの宣言
        launch_str = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),fromat=NV12,width=1280,height=720,framerate=60/1 ! m.sink_0 "
        launch_str+= "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM),fromat=NV12,width=1280,height=720,framerate=60/1 ! nvvideoconvert flip-method=rotate-180 ! m.sink_1 "
        launch_str+= "nvstreammux name=m width=1280 height=1440 batch-size=2 num-surfaces-per-frame=1 "
        launch_str+= "! nvmultistreamtiler columns=1 rows=2 width=1280 height=1440 "
        launch_str+= "! nvvideoconvert ! video/x-raw(memory:NVMM),width=640,height=720,format=RGBA ! tee name=t ! queue ! fakesink name=sink sync=false "
        launch_str+= "t.src_1 ! queue ! nvvidconv ! video/x-raw,width=640,height=720 ! jpegenc ! rtpjpegpay ! udpsink host=192.168.3.9 port=8554 sync=false"
        self.pipeline = Gst.parse_launch(launch_str)
        if not self.pipeline:
            raise RuntimeError('unable to create pipeline')
        
        # Gstreamerプローブの取得
        sink = self.pipeline.get_by_name("sink")
        if not sink:
            raise RuntimeError('unable to get sink element')
        sinkpad = sink.get_static_pad("sink")
        if not sinkpad:
            raise RuntimeError('unable to get sink pad')
        sinkpad.add_probe(Gst.PadProbeType.BUFFER, self.sink_pad_buffer_probe, 0)

        # Gstreamerバス状態の監視
        self.loop = GObject.MainLoop()
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect('message', self.bus_call, self.loop)

        # ROSパラメータ登録
        self.timespan = 0.5
        self.path = 'data/'

        self.declare_parameter('timespan', self.timespan)
        self.timespan = self.get_parameter('timespan').get_parameter_value().double_value
        self.declare_parameter('path', self.path)
        self.path = self.get_parameter('path').get_parameter_value().string_value

        # ROSトピック購読登録
        self.state_vel_sub = self.create_subscription(
            Int32, '/status', self.status_cb, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/pose', self.pose_cb, 10
        )
        self.curvel_sub = self.create_subscription(
            Twist, '/cur_vel', self.curvel_cb, 10
        )
        self.preview_img_pub = self.create_publisher(
            CompressedImage,
            '/preview',
            10
        )

        #ROSサービスサーバー登録
        self.record_srv = self.create_service(
            SetBool, 'record', self.record_cb)
        
        # ROSパラメータ購読登録
        self.add_on_set_parameters_callback(self.parameters_cb)

        # 共有用データ
        self.bridge = CvBridge()
        self.last_time = time.time()
        self.current_vel = [ 0.0, 0.0 ]
        self.current_pose = [0.0, 0.0, 0.0]
        self.capture_count = 0
        self.record_flag = False

        # 保存フォルダとデータ作成
        if not os.path.isdir(self.path):
            os.mkdir(self.path)
        self.file = open(os.path.join(self.path, 'data.csv'), 'w')

    def parameters_cb(self, params):        
        for param in params:
            if param.name == 'timespan':
                self.timespan = param.value

        return SetParametersResult(successful=True)
    
    def status_cb(self, msg: Int32):
        # print('status', msg)
        pass

    def pose_cb(self, msg: PoseStamped):
        self.current_pose[0] = msg.pose.position.x
        self.current_pose[1] = msg.pose.position.y
        self.current_pose[2] = math.acos(msg.pose.orientation.w) * 2.0

    def curvel_cb(self, msg: Twist):
        self.current_vel[0] = msg.linear.x
        self.current_vel[1] = msg.angular.z

    def record_cb(self, request, response):

        response.success = True
        if request.data:
            self.record_flag = True
            response.message = "record start"
        else:
            self.record_flag = False
            response.message = "record end"

        return response


    def bus_call(self, bus, message, loop):
        """パイプラインバスコール

        Args:
            bus : パイプラインバス
            message : パイプラインメッセージ
            loop : パイプラインループ

        Returns:
            bool : 結果
        """
        t = message.type
        if t == Gst.MessageType.EOS:
            self.get_logger().error("End-of-stream")
            loop.quit()
        elif t==Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            self.get_logger().warn("Warning: %s: %s\n" % (err, debug))
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error("Error: %s: %s\n" % (err, debug))
            loop.quit()
        return True
    
    def sink_pad_buffer_probe(self, pad, info, data):
        caps = pad.get_current_caps()
        gst_buffer = info.get_buffer()
        if not gst_buffer:
            return Gst.PadProbeReturn.DROP
    
        batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
        l_frame = batch_meta.frame_meta_list
        while l_frame is not None:
            try:
                frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
            except StopIteration:
                break
            
            now_time = time.time()
            if now_time - self.last_time > self.timespan:
                image = pyds.get_nvds_buf_surface(hash(gst_buffer), frame_meta.batch_id)
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
                
                if self.record_flag:
                    filename = f'{self.capture_count}'.zfill(8) + '.jpg'
                    self.file.write(filename + ', ')
                    self.file.write(f'{self.current_vel[0]}, {self.current_vel[1]}, ')
                    self.file.write(f'{self.current_pose[0]}, {self.current_pose[1]}, {self.current_pose[2]}, ')
                    self.file.write('\n')
                    cv2.imwrite(os.path.join(self.path, filename), image)
                    self.capture_count += 1

                v = int(image.shape[1] / 2.0 * (1.0 - self.current_vel[1]))
                u = int(image.shape[0] / 2.0 * (1.0 - self.current_vel[0]))                
                image = cv2.circle(image, (v, u), 10, (255,0,0), thickness=-1)

                imgmsg = self.bridge.cv2_to_compressed_imgmsg(image)
                self.preview_img_pub.publish(imgmsg)

                self.last_time = now_time

            try:
                l_frame=l_frame.next
            except StopIteration:
                break            

        return Gst.PadProbeReturn.OK
    
    def start_pipeline(self):
        """パイプライン開始
        """
        self.get_logger().info("Starting pipeline")
        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            rclpy.spin(self)
            # self.loop.run()
        except:
            pass
        self.pipeline.set_state(Gst.State.NULL)
