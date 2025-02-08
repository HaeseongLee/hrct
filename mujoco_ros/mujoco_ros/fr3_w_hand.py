import rclpy
import os
from mrc import Fr3Controller
from .gen_block import RandomBoxes
from .multi_thread import MujocoROSBridge
# import time

def main():
    current_dir = os.path.dirname(os.path.realpath(__file__))
    
    rclpy.init()

    # Generate task environment
    xml_name = "random_blocks.xml"
    box_generator = RandomBoxes(bias=(0.4, 0.0, 0.33))    
    box_generator.generate_xml(xml_name)
    
    xml_path = os.path.join(current_dir, '../robots', 'picking_block.xml')
    print(xml_path)
    urdf_path = os.path.join(current_dir, '../robots', 'fr3/fr3_hand.urdf')

    rc = Fr3Controller(urdf_path) 

    robot_info = [xml_path, urdf_path, 1000]
    camera_info = ['hand_eye', 320, 240, 30]

    bridge = MujocoROSBridge(robot_info, camera_info, rc)

    # time.sleep(2.0)
    bridge.run()
    
    bridge.destroy_node()
    rclpy.shutdown()

    
if __name__=="__main__":    
    main()
