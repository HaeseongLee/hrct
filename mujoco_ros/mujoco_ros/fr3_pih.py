import rclpy
import os
from mrc import Fr3PiHController
from .env.gen_peg_in_hole import RandomPegHole
from .utils.multi_thread import MujocoROSBridge
# import time

def main():
    current_dir = os.path.dirname(os.path.realpath(__file__))
    
    rclpy.init()

    # Generate task environment
    xml_generator = RandomPegHole(bias=(0.4, 0.0, 0.32))
    xml_generator.generate_asset_xml("hole_sdf.xml")
    xml_generator.generate_hole_xml("hole.xml")
    xml_generator.generate_peg_xml("peg.xml")
    
    xml_path = os.path.join(current_dir, '../robots', 'peg_in_hole.xml')
    urdf_path = os.path.join(current_dir, '../robots', 'fr3/fr3_hand.urdf')

    rc = Fr3PiHController(urdf_path) 

    robot_info = [xml_path, urdf_path, 1000]
    camera_info = ['hand_eye', 320, 240, 30]

    bridge = MujocoROSBridge(robot_info, camera_info, rc)

    # time.sleep(2.0)
    try:
        bridge.run()
    except KeyboardInterrupt:
        bridge.destroy_node()
        rclpy.shutdown()

    
if __name__=="__main__":    
    main()
