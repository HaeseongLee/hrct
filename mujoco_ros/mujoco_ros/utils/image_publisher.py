from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class MujocoCameraBridge(Node):
    def __init__(self, model, camera_info):
        super().__init__(f'mujoco_{camera_info[0]}_bridge')

        # Mujoco model
        self.model = model

        # camera_info = [name, width, height, fps]
        self.camera_name = camera_info[0]
        self.width = camera_info[1]
        self.height = camera_info[2]
        self.fps = camera_info[3]

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50
        )

        # ROS2 퍼블리셔 설정
        image_topic = "mujoco_sensor/" + self.camera_name
        self.image_publisher = self.create_publisher(Image, image_topic, qos_profile=qos_profile)
        self.image_timer = self.create_timer(1/self.fps, self.image_publish_callback)
        
        self.image = []
        self.ctrl_step = 0

        self.allow_to_pub = False


        self.get_logger().info(f"{self.camera_name} node is loaded")

    def getImage(self, image, time):
        self.image = image
        self.ctrl_step = time

    def image_publish_callback(self):
        if self.allow_to_pub:
            image_msg = Image()
            image_msg.header.stamp.sec = self.ctrl_step
            image_msg.height = self.height
            image_msg.width = self.width
            image_msg.encoding = "rgb8"
            image_msg.is_bigendian = False
            image_msg.step = self.width * 3
            image_msg.data = np.array(self.image).tobytes()

            # 이미지 퍼블리시
            self.image_publisher.publish(image_msg)
            # self.get_logger().info(f'Image published at {self.ctrl_step}')

            self.allow_to_pub = False
