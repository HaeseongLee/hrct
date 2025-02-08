from mujoco_controller_wrapper_cpp import Fr3Controller as Fr3Controllercpp
import numpy as np
from mujoco_msgs.msg import BlockSortingGoal
from std_msgs.msg import Int32
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

class TaskHandler(Node):
    def __init__(self, dt = 0.001):
        super().__init__("pick_place_handler")

        self.task_sub_ = self.create_subscription(BlockSortingGoal, 'block_sorting_command', self.handle_callback, 10)
        self.task_sub_  # prevent unused variable warning

        self.dt = dt

        self.target_pub_ = self.create_publisher(Int32, 'block_id_update', 10)
        self.timer = self.create_timer(self.dt, self.update_callback)

        self.goal_received_ = False

        self.block_id = 0
        self.cnt = 0

        self.name = []
        self.achieved = False
        self.pick_goal = np.empty((7,))        
        self.place_goal = np.empty((7,))

        self.flag = False

    def handle_callback(self, msg):
        self.goal_received_ = True
        self.name = msg.name

        self.name = msg.name
        self.achieved = msg.achieved

        self.pick_goal[:3] = [
            msg.pick.position.x,
            msg.pick.position.y,
            msg.pick.position.z
        ]
        self.pick_goal[3:] = [
            msg.pick.orientation.x,
            msg.pick.orientation.y,
            msg.pick.orientation.z,
            msg.pick.orientation.w
        ]

        self.place_goal[:3] = [
            msg.place.position.x,
            msg.place.position.y,
            msg.place.position.z
        ]
        self.place_goal[3:] = [
            msg.place.orientation.x,
            msg.place.orientation.z,
            msg.place.orientation.y,
            msg.place.orientation.w
        ]

        # print(f"Object: {msg.name}")
        # print(f"  Position  : ({msg.place.position.x:.3f}, {msg.place.position.y:.3f}, {msg.place.position.z:.3f})")
        # print(f"  Quaternion: ({msg.place.orientation.x:.3f}, {msg.place.orientation.y:.3f}, {msg.place.orientation.z:.3f}, {msg.place.orientation.w:.3f})")

        # self.cnt += 1
        # if self.cnt == 1000:
        #     self.block_id += 1
        #     self.cnt = 0

    def update_callback(self):
        if self.goal_received_:
            msg = Int32()
            msg.data = self.block_id
            self.target_pub_.publish(msg)

    def is_running(self, cond:bool):
        if self.goal_received_ and cond:
            self.flag = True
        else:
            self.task_start = False
        return self.flag

class Fr3Controller(Node):
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
                x = np.vstack((self.th.pick_goal[:3], self.th.place_goal[:3]))
                r = np.vstack((self.quat2rot(self.th.pick_goal[3:]), self.quat2rot(self.th.place_goal[3:])))
                t = np.array((2, 3))
                ctrl = self.controller.pickAndPlace(x, r, t)

                if(self.controller.next_task_):
                    self.th.block_id += 1
                    self.controller.next_task_ = False

            self.controller.saveState()

        return ctrl
    
    # def runExecutor(self):
    #     # self.executor.spin()
    #     rclpy.spin(self.th)
