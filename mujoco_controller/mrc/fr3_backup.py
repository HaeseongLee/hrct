from mujoco_controller_wrapper_cpp import Fr3Controller as Fr3Controllercpp
import numpy as np
from mujoco_msgs.action import PickAndPlace
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
import asyncio

class BlockSortingServer(Node):
    def __init__(self):
        super().__init__("block_sorting_task_manager")
        self.action_server = ActionServer(self, PickAndPlace, 'pick_and_place', self.execute_callback, goal_callback=self.goal_callback)

        self.goal = PickAndPlace.Goal()
        self.feedback = PickAndPlace.Feedback()
        self.result = PickAndPlace.Result()

        self.name = []
        self.pick_goal = np.empty((7,))        
        self.place_goal = np.empty((7,))
        
        self.current_task = [] # initial state

        self.is_pending = True

    def goal_callback(self, goal_request):
        self.get_logger().info('Goal Callback Function ...')
        
        self.is_pending = False
        
        self.name = goal_request.name

        self.pick_goal[:3] = [
            goal_request.pick.position.x,
            goal_request.pick.position.y,
            goal_request.pick.position.z
        ]
        self.pick_goal[3:] = [
            goal_request.pick.orientation.x,
            goal_request.pick.orientation.y,
            goal_request.pick.orientation.z,
            goal_request.pick.orientation.w
        ]

        self.place_goal[:3] = [
            goal_request.place.position.x,
            goal_request.place.position.y,
            goal_request.place.position.z
        ]
        self.place_goal[3:] = [
            goal_request.place.orientation.x,
            goal_request.place.orientation.y,
            goal_request.place.orientation.z,
            goal_request.place.orientation.w
        ]

        self.current_task = 'pick'
        self.pick_task_ = True
        self.place_task_ = False

        return GoalResponse.ACCEPT  
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        self.goal_handle = goal_handle

        # while not self.result.is_completed:

        # 작업 상태를 업데이트하고 작업 시작
        # self.result_sent = False        

        self.goal_handle.succeed()

        return PickAndPlace.Result() # defualt value is "False"

    
    def send_feedback(self):
        try:
            if self.goal_handle is not None:                
                self.feedback.current_task = self.current_task
                self.goal_handle.publish_feedback(self.feedback)       
        except:
            pass

    def send_result(self):
        if self.goal_handle is not None and not self.result_sent:
            self.result_sent = True
            # self.goal_handle.succeed()
            result = PickAndPlace.Result()
            result.is_completed = True
            self.is_pending = True
            self.get_logger().info("Task completed. Result sent.")


    
class Fr3Controller:
    def __init__(self, urdf_path):
        self.controller = Fr3Controllercpp(urdf_path) # rc = robot controller
        self.dof = 7 + 2 # include gripper state
        self.ready_duration = 2.0
        self.t_ = 0.0
        self.hz_ = 1000 

        self.bs = BlockSortingServer()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.bs)

        self.print_once = True
    
    def updateModel(self, data, time):
        q = data.qpos[:self.dof]
        qd = data.qvel[:self.dof]      
        self.t_ = time / self.hz_
        self.controller.updateModel(q, qd, time)
        

    def setIdleConfig(self):
        tau_d = self.controller.setIdleConfig()
        return tau_d
    
    def gripperOpen(self, target_width, time):
        gf = self.controller.gripperOpen(target_width, time)
        return np.array([gf[0]])
    
    def compute(self):
        
        # if(self.t_ < self.ready_duration):
        # if(self.bs.is_pending):
        if(self.bs.is_pending):
            tau_d = self.setIdleConfig()
            gf = self.gripperOpen(0.04, np.array([0.0, 2.0], dtype=np.float64))
            ctrl = np.concatenate((tau_d, gf), axis=0)
        
        elif self.bs.current_task == 'pick':
            if self.print_once:
                print("Print Goal State at once")
                print(self.bs.pick_goal)
                print(self.bs.place_goal)
                self.print_once = False

            x = np.array([0.5,0.0,0.35], dtype=np.float64)
            r = np.array([[1,0,0],[0,-1,0],[0,0,-1]], dtype=np.float64)
            t = 1.0
            ctrl = self.controller.pick(x, r, t)
            self.bs.send_feedback()
        # print("here")



        #     # Pick 작업 완료 조건 확인
        #     if np.linalg.norm(self.controller.get_current_position() - x) < 0.01:
        #         self.bs.current_task = 'place'

        # elif self.bs.current_task == 'place':
        #     # Place 동작
        #     x = self.bs.place_goal[:3]
        #     r = np.eye(3)
        #     t = 1.0
        #     ctrl = self.controller.place(x, r, t)
        #     self.bs.send_feedback()

        #     # Place 작업 완료 조건 확인
        #     if np.linalg.norm(self.controller.get_current_position() - x) < 0.01:
        #         self.bs.send_result()


        # else:
        #     # print(self.bs.name)
        #     # print(self.bs.pick_goal[:3])
        #     # print(type(self.bs.pick_goal))

        #     x = np.array([0.5,0.0,0.35], dtype=np.float64)

        #     r = np.array([[1,0,0],[0,-1,0],[0,0,-1]], dtype=np.float64)
        #     t = 1.0
        #     ctrl = self.controller.pick(x, r, t)

        #     self.bs.send_feedback()
            
        
        return ctrl

