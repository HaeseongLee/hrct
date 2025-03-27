from mujoco_msgs.srv import BlockSortingObs
from mujoco_msgs.action import PickAndPlace
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String
# import sys
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.action import ActionServer
import time


# get list of target objects --> define goal location
class BlockSortingServer(Node):
    def __init__(self):
        super().__init__("block_sorting_task_manager")
        self.action_server = ActionServer(self, PickAndPlace, 'pick_and_place', self.execute_callback)


    def execute_callback(self, goal_handle):
        
        self.get_logger().info('Executing goal...')

        name = goal_handle.request.name
        pick_goal = goal_handle.request.pick
        place_goal = goal_handle.request.place

        feedback_msg = PickAndPlace.Feedback()

        start_time = time.time()

        while time.time() - start_time <= 3.0 :
            feedback_msg.current_task = "pick"
            self.get_logger().info('Feedback: {}'.format(feedback_msg.current_task))
            goal_handle.publish_feedback(feedback_msg)                        
            time.sleep(1.0)

        start_time = time.time()

        while (time.time() - start_time <= 3.0) :
            feedback_msg.current_task = "place"
            self.get_logger().info('Feedback: {}'.format(feedback_msg.current_task))
            goal_handle.publish_feedback(feedback_msg)          
            time.sleep(1.0)

        goal_handle.succeed()

        result = PickAndPlace.Result()
        result.is_completed = True

        return result
        




def main():
    rclpy.init()

    bs = BlockSortingServer()

    rclpy.spin(bs)
    
    rclpy.shutdown()

if __name__== "__main__":
    main()
    