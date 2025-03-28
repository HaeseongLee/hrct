import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from rclpy.executors import MultiThreadedExecutor
import subprocess

class VisionSystem(Node):
    def __init__(self, camera_name):
        super().__init__(f'{camera_name}_node')

        # camera_info = [name, width, height, fps]
        self.camera_name = camera_name

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50
        )
    
        # ROS2 퍼블리셔 설정
        image_topic = "mujoco_sensor/" + self.camera_name
        self.image_subscriber = self.create_subscription(Image, image_topic, self.image_callback, qos_profile)
        # self.image_subscriber = self.create_subscription(Image, image_topic, self.image_callback)

        self.bridge = CvBridge()

        self.image = None 

        # # To save images
        # self.save_dir = os.path.join(os.getcwd(), "images")

        file_path = os.path.abspath(__file__)
        cur_dir = os.path.dirname(file_path)
        self.save_dir = os.path.join(cur_dir, "../images", self.camera_name)

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.running = True
    
        # ros2 bag 실행
        self.bag_process = None
        self.start_ros2_bag()

    def image_callback(self, msg):
        image_time = msg.header.stamp.sec
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # ROS Image → OpenCV Image 변환
        filename = os.path.join(self.save_dir, f"{self.camera_name}_{image_time:06d}.jpg")
        # cv2.imwrite(filename, self.image)  # 파일 저장

    def display(self):
        if self.image is not None:
            cv2.imshow("Image", self.image)
            key = cv2.waitKey(1) & 0xFF 
            if key == 27:  
                self.running = False
        else:
            self.get_logger().warn("No image received yet.")

    def destroy(self):
        if self.bag_process:
            self.bag_process.terminate()
            self.bag_process.wait()
            self.get_logger().info("ros2 bag recording stopped.")
        self.destroy_node()

    def start_ros2_bag(self):
        # ros2 bag 명령어 생성
        bag_command = [
            "ros2", "bag", "record", "-o", os.path.join(self.save_dir, "camera_data"),
            "/mujoco_sensor/" + self.camera_name
        ]

        # subprocess로 ros2 bag 실행
        self.bag_process = subprocess.Popen(bag_command)
        self.get_logger().info("ros2 bag recording started.")


def main():

    rclpy.init()
    # vs = VisionSystem(camera_name)

    hc = VisionSystem("hand_eye")
    tc = VisionSystem("top_down_cam")

    executor = MultiThreadedExecutor()
    executor.add_node(hc)
    executor.add_node(tc)

    start_time = time.time()
    try:
        # while time.time() - start_time < 2.0:  # 2초 동안 실행
        while rclpy.ok():
        # executor.spin()
            # executor.spin_once(timeout_sec=0.01)  # 0.01초마다 한 번씩 실행
            rclpy.spin_once(hc, timeout_sec=0.01)
            rclpy.spin_once(tc, timeout_sec=0.01)
            hc.display()
            tc.display()

        # executor.spin()  # 멀티스레드 방식으로 실행
    except KeyboardInterrupt:
        hc.destroy()
        tc.destroy()
        rclpy.shutdown()

if __name__ =="__main__":
    main()