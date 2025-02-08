from mujoco_msgs.msg import BlockSortingGoal
from std_msgs.msg import Int32
import rclpy
from rclpy.node import Node
import numpy as np
import time

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

    def handle_callback(self, msg):
        self.goal_received_ = True
        name = msg.name
        pick_goal = msg.pick
        place_goal = msg.place

        print(f"Object: {name}")
        print(f"  Position  : ({pick_goal.position.x:.3f}, {pick_goal.position.y:.3f}, {pick_goal.position.z:.3f})")
        print(f"  Quaternion: ({pick_goal.orientation.x:.3f}, {pick_goal.orientation.y:.3f}, {pick_goal.orientation.z:.3f}, {pick_goal.orientation.w:.3f})")

        self.cnt += 1
        if self.cnt == 1000:
            self.block_id += 1
            self.cnt = 0

    def update_callback(self):
        if self.goal_received_:
            msg = Int32()
            msg.data = self.block_id
            self.target_pub_.publish(msg)


# get list of target objects --> define goal location
# class BlockSortingServer(Node):
#     def __init__(self):
#         super().__init__("block_sorting_task_manager")
#         self.action_server = ActionServer(self, PickAndPlace, 'pick_and_place', self.execute_callback)


#     def execute_callback(self, goal_handle):
        
#         self.get_logger().info('Executing goal...')

#         name = goal_handle.request.name
#         pick_goal = goal_handle.request.pick
#         place_goal = goal_handle.request.place

#         feedback_msg = PickAndPlace.Feedback()

#         start_time = time.time()

#         while time.time() - start_time <= 3.0 :
#             feedback_msg.current_task = "pick"
#             self.get_logger().info('Feedback: {}'.format(feedback_msg.current_task))
#             goal_handle.publish_feedback(feedback_msg)                        
#             time.sleep(1.0)

#         start_time = time.time()

#         while (time.time() - start_time <= 3.0) :
#             feedback_msg.current_task = "place"
#             self.get_logger().info('Feedback: {}'.format(feedback_msg.current_task))
#             goal_handle.publish_feedback(feedback_msg)          
#             time.sleep(1.0)

#         goal_handle.succeed()

#         result = PickAndPlace.Result()
#         result.is_completed = True

#         return result
        




def main():
    rclpy.init()

    th = TaskHandler()

    rclpy.spin(th)
    
    rclpy.shutdown()

if __name__== "__main__":
    main()
    