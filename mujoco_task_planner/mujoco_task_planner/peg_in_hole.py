from mujoco_msgs.srv import PegInHoleObs
from mujoco_msgs.msg import PegInHoleGoal
from std_msgs.msg import Int32
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
import random
import math

# def position_noise(noise):
#     x = random.uniform(-1.0, 1.0)
#     y = random.uniform(-1.0, 1.0)
#     z = random.uniform(-1.0, 1.0)
    
#     magnitude = math.sqrt(x**2 + y**2 + z**2)
    
#     x, y, z = (x / magnitude) * noise, (y / magnitude) * noise, (z / magnitude) * noise
#     return np.array((x, y, z))

def position_noise(noise):
    x = random.uniform(-1.0, 1.0)
    y = random.uniform(-1.0, 1.0)
    
    magnitude = math.sqrt(x**2 + y**2)
    
    x, y = (x / magnitude) * noise, (y / magnitude) * noise
    return np.array((x, y, 0.0))

def quat_noise(noise):
    # noise -> degree
    magnitude = math.radians(noise)
    # print(magnitude)
    # magnitude = 10
    
    axis_x = random.uniform(-1.0, 1.0)
    axis_y = random.uniform(-1.0, 1.0)
    axis_z = random.uniform(-1.0, 1.0)
    
    norm = math.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
    axis_x, axis_y, axis_z = axis_x / norm, axis_y / norm, axis_z / norm
    
    angle = random.uniform(-magnitude, magnitude)
    # angle = random.uniform(magnitude, magnitude)

    qw = math.cos(angle / 2)
    qx = axis_x * math.sin(angle / 2)
    qy = axis_y * math.sin(angle / 2)
    qz = axis_z * math.sin(angle / 2)
    
    return np.array((qx, qy, qz, qw))


class TaskMaster(Node):
    def __init__(self, dt = 0.001):
        super().__init__("peg_in_hole_commander")

        self.task_pub_  = self.create_publisher(PegInHoleGoal, 'peg_in_hole_command', 10)
        # self.target_sub_ = self.create_subscription(Int32, 'block_id_update', self.update_callback, 10)
        self.dt = dt
        self.task_goals = []
        self.timer = self.create_timer(self.dt, self.command_callback)
        self.pub_trigger = False

    def command_callback(self):        
        if self.pub_trigger:
            self.publish_data()

    # recieve all data 
    def set_task_goal(self, task_goals):
        self.task_goals = task_goals        

    def publish_data(self):

        msg = PegInHoleGoal()

        # if self.block_id >= self.n_blocks:
        #     self.get_logger().info("Block soring is completed!!")
        #     msg.achieved = True
        #     self.task_pub_.publish(msg)
        #     return

        peg = self.task_goals[0] # [pos, quat]
        hole = self.task_goals[1] 

        msg.name = "peg"

        msg.peg.position.x, msg.peg.position.y, msg.peg.position.z = peg[:3]
        msg.peg.orientation.x, msg.peg.orientation.y, msg.peg.orientation.z, msg.peg.orientation.w = peg[3:]

        msg.hole.position.x, msg.hole.position.y, msg.hole.position.z = hole[:3]
        msg.hole.orientation.x, msg.hole.orientation.y, msg.hole.orientation.z, msg.hole.orientation.w = hole[3:]

        msg.achieved = False

        self.task_pub_.publish(msg)

class ServiceClient(Node):
    def __init__(self):
        super().__init__("pih_observation_client")
        self.cli = self.create_client(PegInHoleObs, "pih_observation")

    def send_request(self):

        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Peg-in-Hole Task Planner is waiting ...")

        request = PegInHoleObs.Request() # request to observe target objects in the scene
        request.set_task = True

        self.get_logger().info("Send Client Request")
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.response = future.result()
        self._print_response()

        return future.result()
    
    def _print_response(self):
        hole = self.response.hole
        peg = self.response.peg

        print(f"Object: Hole")
        print(f"  Position  : ({hole.position.x:.3f}, {hole.position.y:.3f}, {hole.position.z:.3f})")
        print(f"  Quaternion: ({hole.orientation.x:.3f}, {hole.orientation.y:.3f}, {hole.orientation.z:.3f}, {hole.orientation.w:.3f})")

        print(f"Object: Peg")
        print(f"  Position  : ({peg.position.x:.3f}, {peg.position.y:.3f}, {peg.position.z:.3f})")
        print(f"  Quaternion: ({peg.orientation.x:.3f}, {peg.orientation.y:.3f}, {peg.orientation.z:.3f}, {peg.orientation.w:.3f})")


class PegInHole(Node):
    def __init__(self):
        super().__init__("Peg_in_Hole_task_manager")
        
        #service client
        self.sc = ServiceClient()

        #Topic publisher
        self.tm = TaskMaster()
        
    def start_peg_in_hole(self):
        
        # Get block information from mujoco
        self.get_srv()

        # if pose erroer is required
        self.add_noise() 

        # Set/Send goal to Action Server (Robot Controller)
        self.set_goal()        
        self.send_goal()

    def get_srv(self):
        self.srv_result = self.sc.send_request()

        hole = self.srv_result.hole
        self.hole_pos = [hole.position.x, hole.position.y, hole.position.z]
        self.hole_quat = R.from_quat(np.array([1.0, 0.0, 0.0, 0.0])).as_quat() # initial gripper orientation

        peg = self.srv_result.peg
        self.peg_pos = [peg.position.x, peg.position.y, peg.position.z]
        self.peg_quat = R.from_quat(np.array([1.0, 0.0, 0.0, 0.0])).as_quat()


    def add_noise(self):
        # p_noise = position_noise(0.0005) # 1mm position error
        # self.peg_pos = self.peg_pos + p_noise

        p_noise = position_noise(0.000) # 1mm position error
        self.hole_pos = self.hole_pos + p_noise

        r_noise = quat_noise(5.0) # 10 degree error
        r_hole = R.from_quat(self.hole_quat)*R.from_quat(r_noise)        
        self.hole_quat = r_hole.as_quat()

    def set_goal(self):
        self.tm_goal =  np.empty((2, 7))
        print(self.peg_quat)

        self.tm_goal[0][:3] = self.peg_pos
        self.tm_goal[0][3:] = self.peg_quat
        
        self.tm_goal[1][:3] = self.hole_pos
        self.tm_goal[1][3:] = self.hole_quat

        # self.tm.set_task_goal(self.tm_goal) // TODO: delete this
        self.get_logger().info("Update Task Goals!!")

    def send_goal(self):
        self.tm.set_task_goal(self.tm_goal)
        self.tm.pub_trigger = True


def main():
    try:
        rclpy.init()
        ph = PegInHole()    
        ph.start_peg_in_hole()            
        rclpy.spin(ph.tm)

    except (KeyboardInterrupt):
        ph.destroy_node()
        rclpy.shutdown()

if __name__== "__main__":
    main()