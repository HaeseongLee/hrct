from mujoco_msgs.srv import BlockSortingObs
from mujoco_msgs.msg import BlockSortingGoal
from std_msgs.msg import Int32
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np
from scipy.spatial.transform import Rotation as R
import random
import math

def position_noise(noise):
    x = random.uniform(-1.0, 1.0)
    y = random.uniform(-1.0, 1.0)
    z = random.uniform(-1.0, 1.0)
    
    magnitude = math.sqrt(x**2 + y**2 + z**2)
    
    x, y, z = (x / magnitude) * noise, (y / magnitude) * noise, (z / magnitude) * noise
    return np.array((x, y, z))

def quat_noise(noise):
    # noise -> degree
    magnitude = math.radians(noise)
    
    axis_x = random.uniform(-1.0, 1.0)
    axis_y = random.uniform(-1.0, 1.0)
    axis_z = random.uniform(-1.0, 1.0)
    
    norm = math.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
    axis_x, axis_y, axis_z = axis_x / norm, axis_y / norm, axis_z / norm
    
    angle = random.uniform(0, magnitude)
    
    qw = math.cos(angle / 2)
    qx = axis_x * math.sin(angle / 2)
    qy = axis_y * math.sin(angle / 2)
    qz = axis_z * math.sin(angle / 2)
    
    return np.array((qx, qy, qz, qw))


class TaskMaster(Node):
    def __init__(self, dt = 0.001):
        super().__init__("pick_place_commander")

        self.task_pub_  = self.create_publisher(BlockSortingGoal, 'block_sorting_command', 10)
        self.target_sub_ = self.create_subscription(Int32, 'block_id_update', self.update_callback, 10)
        self.dt = dt

        self.block_names =[]
        self.block_id = 0
        self.task_goals = []

        self.timer = self.create_timer(self.dt, self.command_callback)

        self.pub_trigger = False


    def command_callback(self):        
        if self.pub_trigger:
            self.publish_data()

    def update_callback(self, msg):
        self.block_id = msg.data
        # print(f"Received Target ID: {self.block_id}")


    # recieve all data 
    def set_task_goal(self, n_blocks, block_names, task_goals):
        self.block_names = block_names
        self.task_goals = task_goals        
        self.n_blocks = n_blocks # number of blocks == number of task exeuctions

    def publish_data(self):

        msg = BlockSortingGoal()

        if self.block_id >= self.n_blocks:
            self.get_logger().info("Block soring is completed!!")
            msg.achieved = True
            self.task_pub_.publish(msg)

            return

        pick = self.task_goals[self.block_id][0] # [pos, quat]
        place = self.task_goals[self.block_id][1] 

        msg.name = self.block_names[self.block_id]

        msg.pick.position.x, msg.pick.position.y, msg.pick.position.z = pick[:3]
        msg.pick.orientation.x, msg.pick.orientation.y, msg.pick.orientation.z, msg.pick.orientation.w = pick[3:]

        msg.place.position.x, msg.place.position.y, msg.place.position.z = place[:3]
        msg.place.orientation.x, msg.place.orientation.y, msg.place.orientation.z, msg.place.orientation.w = place[3:]

        msg.achieved = False

        self.task_pub_.publish(msg)

class ServiceClient(Node):
    def __init__(self):
        super().__init__("block_observation_client")
        self.cli = self.create_client(BlockSortingObs, "block_observation")

    def send_request(self):

        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Block Sorting Task Planner is waiting ...")

        request = BlockSortingObs.Request() # request to observe target objects in the scene
        request.set_task = True

        self.get_logger().info("Send Client Request")
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.response = future.result()
        # self._print_response()

        return future.result()
    
    def _print_response(self):
        for i in range(self.response.n_obj):
            print(f"Object: {self.response.names[i].data}")
            pose = self.response.poses.poses[i]
            print(f"  Position  : ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
            print(f"  Quaternion: ({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f})")



class BlockSorting(Node):
    def __init__(self):
        super().__init__("block_sorting_task_manager")
        # super().__init__()
        
        #service client
        self.sc = ServiceClient()

        #Topic publisher
        self.tm = TaskMaster()
        
    def start_block_sorting(self):
        
        # Get block information from mujoco
        self.get_srv()

        # Define pick/place pose
        self.get_pick_pose()
        self.get_place_pose()

        # Set/Send goal to Action Server (Robot Controller)
        self.set_goal()        
        self.send_goal()



    def get_srv(self):
        self.srv_result = self.sc.send_request()

        n_obj = self.srv_result.n_obj
        n_basket = 2
        self.n_block = n_obj - n_basket # exclude baskets

        self.block_names = []
        self.block_pos = np.empty((self.n_block, 3))
        self.block_quat = np.empty((self.n_block, 4))

        self.basket_names = []
        self.basket_pos = np.empty((n_basket, 3))
        self.basket_quat = np.empty((n_basket, 4))

        box_count = 0
        basket_count = 0

        for i in range(n_obj):
            name = self.srv_result.names[i].data  # 물체의 이름
            pose = self.srv_result.poses.poses[i]  # 물체의 위치와 방향 정보
            px, py, pz = pose.position.x, pose.position.y, pose.position.z
            qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

            # 'box'가 이름에 포함된 경우 block_name에 저장
            if 'box' in name:
                self.block_names.append(name)
                self.block_pos[box_count] = [px, py, pz]
                self.block_quat[box_count] = [qx, qy, qz, qw]
                box_count += 1

            elif 'basket' in name:
                self.basket_names.append(name)
                self.basket_pos[basket_count] = [px, py, pz + 0.05]
                self.basket_quat[basket_count] = [qx, qy, qz, qw]
                basket_count += 1

        # self._print_action_goal()
        # self.get_pick_pose()
        # self.get_place_pose()

    def _print_action_goal(self):
        for i in range(self.n_block):
            print(f"Object : {self.block_names[i]}")
            print(f"Position  : ({self.block_pos[i][0]:.3f}, {self.block_pos[i][1]:.3f}, {self.block_pos[i][2]:.3f})")
            print(f"Quaternion  : ({self.block_quat[i][0]:.3f}, {self.block_quat[i][1]:.3f}, {self.block_quat[i][2]:.3f}, {self.block_quat[i][3]:.3f})")


    def get_pick_pose(self):
        # goal = np.array([pick pose], [place pose])

        self.pick_names = self.block_names
        self.pick_pos = self.block_pos

        for i in range(self.n_block):
            p_noise = position_noise(0.005) # 1cm position error
            self.pick_pos[i] = self.pick_pos[i] + p_noise

        self.pick_quat = np.empty((self.n_block, 4))
 
        # y축으로 180도 회전하는 쿼터니언 생성
        q_y_180 = R.from_euler('y', 180, degrees=True).as_quat()  # [x, y, z, w]
        q_z_90 = R.from_euler('z', -90, degrees=True).as_quat() # gripper orientation offsets

        for i in range(self.n_block):            
            r_block = R.from_quat(self.block_quat[i])
            r_pick = r_block * R.from_quat(q_y_180)
            r_pick = r_pick * R.from_quat(q_z_90)

            r_noise = quat_noise(5) # 10 degree error
            r_pick = r_pick * R.from_quat(r_noise)

            self.pick_quat[i] = r_pick.as_quat()         

    def get_place_pose(self):        
        self.place_names = self.basket_names
        self.place_pos = self.basket_pos
        self.place_quat = R.from_quat(np.array([1.0, 0.0, 0.0, 0.0])) # fix orientation

    def set_goal(self):
        self.tm_goal =  np.empty((self.n_block, 2, 7))

        for i in range(self.n_block):
            box_name = self.block_names[i]
            
            if 'red_box' in box_name:
                basket_name = 'red_basket'
            elif 'blue_box' in box_name:
                basket_name = 'blue_basket'
            

            self.tm_goal[i][0][:3] = self.pick_pos[i]
            self.tm_goal[i][0][3:] = self.pick_quat[i]
            
            p_noise = position_noise(0.025) # 5cm position error
            r_noise = quat_noise(10) # 20 degree error

            basket_idx = self.basket_names.index(basket_name)  
            self.tm_goal[i][1][:3] = self.place_pos[basket_idx] + p_noise
            self.tm_goal[i][1][3:] = (self.place_quat*R.from_quat(r_noise)).as_quat() # basket의 방향 설정

        self.tm.set_task_goal(self.n_block, self.block_names, self.tm_goal)
        self.get_logger().info("Update Task Goals!!")
        # # print(self.tm_goal)

    def send_goal(self):
        self.tm.set_task_goal(self.n_block, self.block_names, self.tm_goal)
        self.tm.pub_trigger = True


def main():
    try:
        with rclpy.init():            
            bs = BlockSorting()    
            bs.start_block_sorting()            
            rclpy.spin(bs.tm)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__== "__main__":
    main()