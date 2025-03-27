from mujoco_msgs.srv import BlockSortingObs
from mujoco_msgs.action import PickAndPlace
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
import numpy as np
from scipy.spatial.transform import Rotation as R



class ActionClientNode(Node):
    def __init__(self):
        super().__init__("pick_place_action_client")
        
        # action client
        self.ac = ActionClient(self, PickAndPlace, 'pick_and_place')

        self.block_names =[]
        self.ac_goals = []
        self.block_id = 0

    # recieve all data 
    def set_action_goal(self, n_blocks, block_names, ac_goals):
        self.block_names = block_names
        self.ac_goals = ac_goals        
        self.n_blocks = n_blocks # number of blocks == number of task exeuctions

    def send_action_goal(self):
        self._send_next_action_goal()

    def _send_next_action_goal(self):        
        # goal = np.array([pick pose], [place pose])

        if self.block_id >= self.n_blocks:
            self.get_logger().info("Block soring is completed!!")
            return

        # self.get_logger().info("Send Action Goal {}".format(self.block_id))

        pick = self.ac_goals[self.block_id][0] # [pos, quat]
        place = self.ac_goals[self.block_id][1] 

        goal_msg = PickAndPlace.Goal()
        goal_msg.name = self.block_names[self.block_id]

        goal_msg.pick.position.x, goal_msg.pick.position.y, goal_msg.pick.position.z = pick[:3]
        goal_msg.pick.orientation.x, goal_msg.pick.orientation.y, goal_msg.pick.orientation.z, goal_msg.pick.orientation.w = pick[3:]

        goal_msg.place.position.x, goal_msg.place.position.y, goal_msg.place.position.z = place[:3]
        goal_msg.place.orientation.x, goal_msg.place.orientation.y, goal_msg.place.orientation.z, goal_msg.pick.orientation.w = place[3:]


        while not self.ac.wait_for_server(timeout_sec=3.0):
            self.get_logger().info("Action Client is waiting ...")

        self.send_goal_future = self.ac.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        # self.send_goal_future.add_done_callback(self.goal_response_callback) 

        rclpy.spin_until_future_complete(self, self.send_goal_future)

        self.goal_handle = self.send_goal_future.result()
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

            

    def goal_response_callback(self, future):
        print("Goal response callback")
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        # self.get_result_future = self.goal_handle.get_result_async()
        # self.get_result_future.add_done_callback(self.result_callback)


    def result_callback(self, future):
        result = future.result().result

        self.get_logger().info("Settling result callback here")
        if(result.is_completed):
            self.get_logger().info('Result: {}'.format(result.is_completed))
            self.block_id += 1        
            self._send_next_action_goal()
        else:
            print(f"result callback is fisnished : {result.is_completed}")
        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {}'.format(feedback.current_task))


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

        #action client
        self.ac = ActionClientNode()
        
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

        self.n_block = self.srv_result.n_obj

        self.block_names = []
        self.block_pos = np.empty((self.n_block, 3))
        self.block_quat = np.empty((self.n_block, 4))

        for i in range(self.n_block):
            self.block_names.append(self.srv_result.names[i].data)
            pose = self.srv_result.poses.poses[i]
            px, py, pz = pose.position.x, pose.position.y, pose.position.z
            qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            self.block_pos[i] = px, py, pz
            self.block_quat[i] = qx, qy, qz, qw

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

        self.pick_quat = np.empty((self.n_block, 4))
 
        # y축으로 180도 회전하는 쿼터니언 생성
        q_y_180 = R.from_euler('y', 180, degrees=True).as_quat()  # [x, y, z, w]

        for i in range(self.n_block):            
            r_block = R.from_quat(self.block_quat[i])
            r_pick = R.from_quat(q_y_180) * r_block
            self.pick_quat[i] = r_pick.as_quat()         

    def get_place_pose(self):
        # TODO: replace pos, quat info!!
        self.place_pos = np.array([0.1, -0.4, 0.35])
        self.place_quat = np.array([1.0, 0.0, 0.0, 0.0])

    def set_goal(self):
        self.ac_goal =  np.empty((self.n_block, 2, 7))
        for i in range(self.n_block):
            self.ac_goal[i][0][:3] = self.pick_pos[i]
            self.ac_goal[i][0][3:] = self.pick_quat[i]
            self.ac_goal[i][1][:3] = self.place_pos
            self.ac_goal[i][1][3:] = self.place_quat

        self.ac.set_action_goal(self.n_block, self.block_names, self.ac_goal)
        self.get_logger().info("Update Action Goals!!")
        # print(self.ac_goal)

    def send_goal(self):
        self.ac.set_action_goal(self.n_block, self.block_names, self.ac_goal)
        self.ac.send_action_goal()

def main():
    try:
        with rclpy.init():            
            bs = BlockSorting()    
            bs.start_block_sorting()            
            rclpy.spin(bs.ac)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__== "__main__":
    main()