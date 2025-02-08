import mujoco
import mujoco.viewer as mj_view
import time
import rclpy

def generate_hole_xml(width, height, depth):

    thickness = 0.01 # 1cm 
    half_w, half_h, half_d = width / 2, height / 2, depth / 2
    half_t = thickness / 2

    r_c = 0.0001 /2 # half of clearance
    
    # XML 템플릿
    xml_template = f"""
    <mujoco> 
    <compiler angle="degree"/>
    <option timestep="0.001"/>

    <asset>        
        <texture name="plane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 0.15 0.2" width="256" height="256" mark="cross" markrgb=".8 .8 .8" />
        <material name="plane" reflectance="0.3" texture="plane" texrepeat="1 1" texuniform="true" />
    </asset>

    <worldbody>
        <!-- Peg -->
        <body name="peg" pos="0.0 0 0.10">
            <joint name="peg_joint" type="free"/>
            <geom  type="box" size="{half_w - r_c} {half_h - r_c} {half_d + half_t}" rgba="0.8 0.2 0.2 1" contype="2" conaffinity="1"/>
        </body>-->

        
        <!-- upper block -->
        <body name="upper" pos="0 {half_h + half_t} {thickness + half_d}">
            <geom type="box" size="{half_w} {half_t} {half_d}" rgba="0.8 0.2 0.2 1"/>
        </body>

        <!-- lower block -->
        <body name="lower" pos="0 -{half_h + half_t} {thickness + half_d}">
            <geom type="box" size="{half_w} {half_t} {half_d}" rgba="0.2 0.8 0.2 1"/>
        </body>

        <!-- right block -->
        <body name="right" pos="{half_w + half_t} 0 {thickness + half_d}">
            <geom type="box" size="{half_t} {half_h + thickness} {half_d}" rgba="0.2 0.2 0.8 1"/>
        </body>

        <!-- left block -->
        <body name="left" pos="-{half_w + half_t} 0 {thickness + half_d}">
            <geom type="box" size="{half_t} {half_h + thickness} {half_d}" rgba="0.8 0.8 0.2 1"/>
        </body>

        <!-- plate -->
        <body name="plate" pos="0 0 {half_t}">
            <geom type="box" size="{half_w + thickness} {half_h + thickness} {half_t}" rgba="0.8 0.8 0.2 1" solref="0.001 0.1"/>
        </body>


        <body name="col" pos="0.05 0 {thickness + half_d}">
            <geom type="box" size="{half_t} {half_h + thickness} {half_d + thickness}" rgba="0.8 0.8 0.2 1"/>
        </body>

        
        <light directional="true" diffuse=".2 .2 .2" specular="0 0 0" pos="0 1 5" dir="0 -1 -1" castshadow="false"/>
        <light directional="false" diffuse=".8 .8 .8" specular="0.3 0.3 0.3" pos="0 -1 4" dir="0 0 -1" />
        <light directional="true" diffuse="0 0 0" specular=".7 .7 .7" pos="0 3 3" dir="0 -3 -3" />
        <geom name="floor" pos="0 0 -.0" group="1" size="0 0 .1" type="plane" material="plane" condim="3" />
    </worldbody>
    </mujoco>
    """
    return xml_template


def main():
    rclpy.init()

    # 사용자 입력 (예: 0.3m x 0.3m x 0.1m 구멍)
    width = 0.02  # 가로
    height = 0.02  # 세로
    depth = 0.03  # 깊이

    # MuJoCo XML 생성 및 로드
    xml_model = generate_hole_xml(width, height, depth)
    model = mujoco.MjModel.from_xml_string(xml_model)
    data = mujoco.MjData(model)

    # Viewer 실행
    try:
        with mj_view.launch_passive(model, data) as viewer:
            while rclpy.ok() and viewer.is_running():
                mujoco.mj_step(model, data)  # 시뮬레이션 실행
                viewer.sync()  # 화면 업데이트
                time.sleep(0.001)  # 1ms 대기

    except KeyboardInterrupt:
        print("\nSimulation interrupted. Closing viewer...")
        # mj_view.close()  # 시뮬레이터 창 닫기

    rclpy.shutdown()

if __name__=="__main__":
    print("123")
    main()