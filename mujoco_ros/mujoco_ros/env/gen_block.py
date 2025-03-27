import random
import numpy as np
import mujoco
import mujoco.viewer as mj_view
import os
import time

class RandomBoxes:
    def __init__(self, n_boxes=6, x_range=(0.005, 0.015), y_range=(0.01, 0.025), z_range=(0.01, 0.02), bias=(0.0, 0.0, 0.0), min_distance = 0.05):
    
        self.num_boxes = round(random.uniform(1, n_boxes))
        self.x_range = x_range # half of the length
        self.y_range = y_range
        self.z_range = z_range

        self.bias = bias

        self.min_distance = min_distance
        self.positions = []  # To store already placed positions
        
        self.current_root = os.path.dirname(os.path.realpath(__file__))
        

    def generate_random_color(self):
        
        prob = random.uniform(0.0, 1.0)
        if prob > 0.5:
            color="red"
        else:
            color="blue"

        if(color == "red"):
            red_value = random.uniform(0.5, 1.0)  # Red value in the range [0.5, 1.0]
            green_value = random.uniform(0, 0.3)  # Green value in the range [0, 0.2]
            blue_value = random.uniform(0, 0.3)   # Blue value in the range [0, 0.2]
        
        if(color == "blue"):
            red_value = random.uniform(0.0, 0.3)  # Red value in the range [0.5, 1.0]
            green_value = random.uniform(0, 0.3)  # Green value in the range [0, 0.2]
            blue_value = random.uniform(0.5, 1.0)   # Blue value in the range [0, 0.2]
        
        rgba = f"{red_value} {green_value} {blue_value} 1"

        return color, rgba

    def is_valid_position(self, x_pos, y_pos):
        for existing_pos in self.positions:
            dist = np.sqrt((x_pos - existing_pos[0])**2 + (y_pos - existing_pos[1])**2)
            if dist < self.min_distance:
                return False
        return True
    
    def generate_random_position(self):
        while True:
            print("generating random boxes ...")
            x_pos = np.random.choice(np.arange(-0.15, 0.2, 0.01))
            y_pos = np.random.choice(np.arange(-0.15, 0.2, 0.01))
            # z_pos = random.uniform(0.5, 1.5)   # Random z position

            x_pos += self.bias[0]
            y_pos += self.bias[1]

            if self.is_valid_position(x_pos, y_pos):
                self.positions.append((x_pos, y_pos))
                return x_pos, y_pos, self.bias[2]
        
        # return (x_pos, y_pos, self.bias[2])


    def generate_random_orientation(self):
        rz = random.uniform(0, 360.0)
        return (0, 0, rz)
    
    def generate_random_size(self):
        size_x = random.uniform(self.x_range[0], self.x_range[1])
        size_y = random.uniform(self.y_range[0], self.y_range[1])
        size_z = random.uniform(self.z_range[0], self.z_range[1])
        return (size_x, size_y, size_z)

    def generate_xml(self, filename="random_blocks.xml"):
        
        xml_content = """
<mujocoinclude>
        """        
        for i in range(self.num_boxes):
            # Generate random color, position, and size
            color, rgba = self.generate_random_color()
            x_pos, y_pos, z_pos = self.generate_random_position()
            size_x, size_y, size_z = self.generate_random_size()
            rx, ry, rz = self.generate_random_orientation()

            # Add material and body to the XML content
            xml_content += f"""
    <body name="{color}_box_{i}" pos="{x_pos} {y_pos} {z_pos + size_z}" euler="{rx} {ry} {rz}">
        <joint name="block_joint_{i}" type="free"/>
        <geom type="box" size="{size_x} {size_y} {size_z}" rgba="{rgba}" />
    </body>
            """
        
        xml_content += """            
</mujocoinclude>
        """
        self.save_root = os.path.join(self.current_root, "../../robots/common")
        # Write to the specified file
        with open(os.path.join(self.save_root, filename), "w") as f:
            f.write(xml_content)
        print(f"XML file generated: {filename}")
        print("save root is ", self.save_root)

def main():
    xml_name = "random_blocks.xml"
    box_generator = RandomBoxes(bias=(0.4, 0.0, 0.33))
    box_generator.generate_xml(xml_name)
    # rclpy.init()
    
    current_dir = os.path.dirname(os.path.realpath(__file__))    
    xml_path = os.path.join(current_dir, '../../robots', 'picking_block.xml')


    print(xml_path)
    # MuJoCo XML 생성 및 로드
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Viewer 실행
    try:
        with mj_view.launch_passive(model, data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(model, data)  # 시뮬레이션 실행
                viewer.sync()  # 화면 업데이트
                time.sleep(0.001)  # 1ms 대기

    except KeyboardInterrupt:
        print("\nSimulation interrupted. Closing viewer...")
        # mj_view.close()  # 시뮬레이터 창 닫기
    

if __name__=="__main__":    
    main()