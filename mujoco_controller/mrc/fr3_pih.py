from mujoco_controller_wrapper_cpp import Fr3Controller as Fr3Controllercpp
import numpy as np
from mujoco_msgs.msg import PegInHoleGoal
from mujoco_msgs.srv import ToggleAttachment
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import rclpy

class TaskHandler(Node):
    def __init__(self, dt = 0.001):
        super().__init__("pick_place_handler")

        self.task_sub_ = self.create_subscription(PegInHoleGoal, 'peg_in_hole_command', self.handle_callback, 10)
        self.task_sub_  # prevent unused variable warning

        self.ta_client = self.create_client(ToggleAttachment, "toggle_attachment")

        self.dt = dt

        self.goal_received_ = False

        self.cnt = 0

        self.name = []
        self.achieved = False
        self.peg_goal = np.empty((7,))        
        self.hole_goal = np.empty((7,))

        self.flag = False

    def handle_callback(self, msg):
        self.goal_received_ = True
        self.name = msg.name

        self.name = msg.name
        self.achieved = msg.achieved

        self.peg_goal[:3] = [
            msg.peg.position.x,
            msg.peg.position.y,
            msg.peg.position.z
        ]
        self.peg_goal[3:] = [
            msg.peg.orientation.x,
            msg.peg.orientation.y,
            msg.peg.orientation.z,
            msg.peg.orientation.w
        ]

        self.hole_goal[:3] = [
            msg.hole.position.x,
            msg.hole.position.y,
            msg.hole.position.z
        ]
        self.hole_goal[3:] = [
            msg.hole.orientation.x,
            msg.hole.orientation.z,
            msg.hole.orientation.y,
            msg.hole.orientation.w
        ]

        # print(f"Object: {msg.name}")
        # print(f"  Position  : ({msg.hole.position.x:.3f}, {msg.hole.position.y:.3f}, {msg.hole.position.z:.3f})")
        # print(f"  Quaternion: ({msg.hole.orientation.x:.3f}, {msg.hole.orientation.y:.3f}, {msg.hole.orientation.z:.3f}, {msg.hole.orientation.w:.3f})")

        # self.cnt += 1
        # if self.cnt == 1000:
        #     self.block_id += 1
        #     self.cnt = 0

    # def update_callback(self):
    #     if self.goal_received_:
    #         msg = Int32()
    #         msg.data = self.block_id
    #         self.target_pub_.publish(msg)

    def is_running(self, cond:bool):
        if self.goal_received_ and cond:
            self.flag = True
        else:
            self.task_start = False
        return self.flag
    
    def send_toggle_attachment(self):
        ta_req = ToggleAttachment.Request()
        ta_req.attach = True
        ta_req.child = "peg"
        ta_req.parent = "hand_tcp"

        self.get_logger().info("Send Toggle Attachment Client Request")
        future = self.ta_client.call_async(ta_req)
        rclpy.spin_until_future_complete(self, future)

        # if self.ta_client.future.done():  # async response
        if future.done():  # async response

            response = self.ta_client.future.result()
            self.ta_client.get_logger().info(f"Result: {response.success}")
        
        return future.result()

class Fr3PiHController(Node):
    def __init__(self, urdf_path):
        super().__init__("robot_control_node")

        self.controller = Fr3Controllercpp(urdf_path) # rc = robot controller
        self.dof = 7 + 2 # include gripper state
        self.ready_duration = 2.0
        self.t_ = 0.0
        self.hz_ = 1000 

        self.th = TaskHandler(1/self.hz_)

        self.print_once = True

    def updateModel(self, data, time):
        q = data.qpos[:self.dof]
        qd = data.qvel[:self.dof]      
        # torque sensor[2] = qfrc_actuator
        tau = data.sensordata[2:self.dof*3:3] # torque = 3-axes data
        ft = data.sensordata[-6:]

        self.t_ = time / self.hz_
        self.controller.updateModel(q, qd, tau, ft, time)
        
    def setIdleConfig(self, time):
        tau_d = self.controller.setIdleConfig(time)
        return tau_d
    
    def gripperOpen(self, target_width, time):
        gf = self.controller.gripperOpen(target_width, time)
        return np.array([gf[0]])
    
    def quat2rot(self, quat):
        rotation = R.from_quat(quat)  # [x, y, z, w]
        rot_matrix = rotation.as_matrix()  # Convert to 3x3 rotation matrix
        return rot_matrix
        
    def compute(self):

        if not self.th.is_running(self.t_ >= self.ready_duration):
            time = np.array((0.0, self.ready_duration))
            tau_d = self.setIdleConfig(time)
            gf = self.gripperOpen(0.04, np.array([0.0, 2.0], dtype=np.float64))
            ctrl = np.concatenate((tau_d, gf), axis=0)

        else:
            if(self.th.achieved):
                ctrl = self.controller.idleState()
            else:
                x = np.vstack((self.th.peg_goal[:3], self.th.hole_goal[:3]))
                r = np.vstack((self.quat2rot(self.th.peg_goal[3:]), self.quat2rot(self.th.hole_goal[3:])))
                t = np.array((2, 3))
                ctrl = self.controller.pegInHole(x, r, t)
                            
                # if(self.controller.next_task_):                    
                    # self.controller.next_task_ = False

            self.controller.saveState()

        return ctrl
    
    # def runExecutor(self):
    #     # self.executor.spin()
    #     rclpy.spin(self.th)
