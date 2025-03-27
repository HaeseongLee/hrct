import random
import numpy as np
import mujoco
import mujoco.viewer as mj_view
import os
import time

class RandomPegHole:
    def __init__(self, size=(0.03, 0.04), bias=(0.0, 0.0, 0.0), min_distance = 0.05):
        
        self.size_range = size # 1~3cm
        self.bias = bias
        self.min_distance = min_distance
        self.peg = [] # position
        self.hole = [] # position
        
        self.current_root = os.path.dirname(os.path.realpath(__file__))
        
        # self.clearance = random.uniform(-0.0005, 0.0)
        self.clearance = 0.0

        self.s = self.generate_random_size()
        self.generate_random_position()

    def is_valid_position(self, x_pos, y_pos):
        dist = np.sqrt((x_pos - self.hole[0])**2 + (y_pos - self.hole[1])**2)

        if dist < self.min_distance:
            return False
        return True
    
    def generate_random_position(self):
        x1 = np.random.choice(np.arange(-0.15, 0.2, 0.01)) + self.bias[0]
        y1 = np.random.choice(np.arange(-0.15, 0.2, 0.01)) + self.bias[1]

        x1 = np.round(x1, 2) # 0.xx
        y1 = np.round(y1, 2)

        self.hole = x1, y1, self.bias[2]
        print("Adjusting object positions...")
        while True:
            x2 = np.random.choice(np.arange(-0.15, 0.2, 0.01)) + self.bias[0]
            y2 = np.random.choice(np.arange(-0.15, 0.2, 0.01)) + self.bias[1]

            x2 = np.round(x2, 2)
            y2 = np.round(y2, 2)
            
            if self.is_valid_position(x2, y2):
                self.peg = x2, y2, self.bias[2]
                return 

    
    def generate_random_size(self):
        return random.uniform(self.size_range[0], self.size_range[1])

    def generate_asset_xml(self, filename="hole_sdf.xml"):
        xml_content = """
<mujocoinclude>
        """        
            # Add material and body to the XML content
        xml_content += f"""
    <asset>
        <mesh name="hole" file="../common/mesh/hole.obj" scale="{self.s} {self.s} {self.s}" >
            <plugin instance="sdf" />
        </mesh>
    </asset>
        """

        xml_content += """            
</mujocoinclude>
        """
        self.save_root = os.path.join(self.current_root, "../../robots/common")
        # Write to the specified file
        with open(os.path.join(self.save_root, filename), "w") as f:
            f.write(xml_content)
        print(f"Asset XML file generated: {filename}")
        print("save root is ", self.save_root)

    def generate_hole_xml(self, filename="hole.xml"):
        xml_content = """
<mujocoinclude>
        """        
        xml_content += f"""
    <body name="hole" pos="{self.hole[0]} {self.hole[1]} {self.hole[2] + self.s/2}" euler="1.570796326794897 0 0">
        <geom name="hole" type="sdf" mesh="hole" rgba="0.94 0.50 0.50 0.5">
            <plugin instance="sdf"/>
        </geom>
    </body>       
</mujocoinclude>
        """

        self.save_root = os.path.join(self.current_root, "../../robots/common")
        # Write to the specified file
        with open(os.path.join(self.save_root, filename), "w") as f:
            f.write(xml_content)
        print(f"Object XML file generated: {filename}")
        print("save root is ", self.save_root)

    def generate_peg_xml(self, filename="peg.xml"):
        xml_content = """
<mujocoinclude>
        """        
        xml_content += f"""
    <body name="peg" pos="{self.peg[0]} {self.peg[1]} {self.peg[2] + self.s}">
        <joint name="peg_joint" type="free"/>
        <geom type="cylinder" size="{self.s/2.0 - self.clearance} {self.s}" rgba="0.0 0.0 0.545 1" condim="6" friction="1.5 0.1 0.1"
            density="100" solimp="0.999 0.999 0.001" solref="0.001 2.5" margin = "0.001"/>
    </body>
        """
        xml_content += """            
</mujocoinclude>
        """

        self.save_root = os.path.join(self.current_root, "../../robots/common")
        # Write to the specified file
        with open(os.path.join(self.save_root, filename), "w") as f:
            f.write(xml_content)
        print(f"Object XML file generated: {filename}")
        print("save root is ", self.save_root)

def main():
    asset_xml = "hole_sdf.xml"

    xml_generator = RandomPegHole(bias=(0.4, 0.0, 0.32))
    xml_generator.generate_asset_xml(asset_xml)
    xml_generator.generate_hole_xml("hole.xml")
    xml_generator.generate_peg_xml("peg.xml")
    
    current_dir = os.path.dirname(os.path.realpath(__file__))    
    xml_path = os.path.join(current_dir, '../robots', 'peg_in_hole.xml')


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