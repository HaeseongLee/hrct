import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import cv2
import subprocess
from rclpy.qos import QoSProfile
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import MultiThreadedExecutor

class ImageExtractor(Node):
    def __init__(self, camera_name, rosbag_file_path):
        super().__init__(f'{camera_name}_node')

        self.camera_name = camera_name

        # ros2 bag 파일 경로 설정
        self.rosbag_file_path = rosbag_file_path

        # ros2 bag play 명령어 실행 (백그라운드)
        self.play_bag()

        # 이미지 구독할 토픽 이름 설정
        self.image_topic = "mujoco_sensor/" + self.camera_name

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50
        )
    
        # 이미지 구독자 생성
        self.image_subscriber = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile)
        
        # CV Bridge 생성 (ROS 이미지 -> OpenCV 이미지 변환)
        self.bridge = CvBridge()
        
        # 저장 경로 설정
        # file_path = os.path.abspath(__file__)
        # cur_dir = os.path.dirname(file_path)
        # self.save_dir = os.path.join(cur_dir, "extracted_images")
        root = "/home/hs_dyros/ros2_ws/src/mujoco_sensor/images"
        self.save_dir = os.path.join(root, self.camera_name)
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)



    def play_bag(self):
        """ros2 bag play 명령어 백그라운드에서 실행"""
        bag_command = ["ros2", "bag", "play", self.rosbag_file_path]
        self.bag_process = subprocess.Popen(bag_command)
        self.get_logger().info(f"ros2 bag play started for {self.rosbag_file_path}")

    def image_callback(self, msg):
        """콜백 함수 - rosbag에서 받은 이미지를 저장"""
        # 메시지에서 시간 정보 추출 (파일명으로 사용)
        timestamp = msg.header.stamp.sec

        # 이미지 변환 (ROS 이미지 -> OpenCV 이미지)
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 저장할 파일 이름 설정
        filename = os.path.join(self.save_dir, f"{self.camera_name}_{timestamp:06d}.png")

        # 이미지 저장
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f"Saved image to {filename}")

def main():
    rclpy.init()

    hand_eye_path = "/home/hs_dyros/ros2_ws/src/mujoco_sensor/images/hand_eye/camera_data/"
    top_down_cam_path = "/home/hs_dyros/ros2_ws/src/mujoco_sensor/images/top_down_cam/camera_data/"

    # ImageExtractor 노드 실행
    hc = ImageExtractor("hand_eye", hand_eye_path)
    tc = ImageExtractor("top_down_cam", top_down_cam_path)

    executor = MultiThreadedExecutor()
    # executor.add_node(hc)
    executor.add_node(tc)


    # 실행
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        hc.destroy()
        tc.destroy()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
