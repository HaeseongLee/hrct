import rclpy
from rclpy.node import Node
import mujoco
import mujoco.viewer as mj_view
import threading
import time
from .scene_monitor import SceneMonitor
from .image_publisher import MujocoCameraBridge
import numpy as np


class MujocoROSBridge(Node):
    def __init__(self, robot_info, camera_info, robot_controller):
        super().__init__('mujoco_ros_bridge')

        # robot_info = [xml, urdf, hz]
        self.xml_path = robot_info[0]
        self.urdf_path = robot_info[1]
        self.ctrl_freq = robot_info[2]

        # camera_info = [name, width, height, fps]
        self.camera_name = camera_info[0]
        self.width = camera_info[1]
        self.height = camera_info[2]
        self.fps = camera_info[3]
          
        self.rc = robot_controller

        # Mujoco 모델 로드
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)

        self.dt = 1 / self.ctrl_freq
        self.model.opt.timestep = self.dt
       
        self.sm = SceneMonitor(self.model, self.data)

        self.hand_eye = MujocoCameraBridge(self.model, camera_info)
        self.top_down_cam = MujocoCameraBridge(self.model, ['top_down_cam', 640, 480, self.fps])
        
        # self.renderer = mujoco.Renderer(self.model, width=self.width, height=self.height)

        self.ctrl_dof = 8 # 7 + 1
        self.ctrl_step = 0

        # 스레드 실행
        self.running = True
        self.lock = threading.Lock()
        self.robot_thread = threading.Thread(target=self.robot_control, daemon=True)
        self.hand_eye_thread = threading.Thread(target=self.hand_eye_control, daemon=True)
        self.td_cam_thread = threading.Thread(target=self.td_cam_control, daemon=True)


    # visualize thread = main thread
    def run(self):        
        try:     
            with mj_view.launch_passive(self.model, self.data) as viewer:            
                # self.sm.getAllObject()        
                # self.sm.getTargetObject()       
                # self.sm.getSensor() 
                self.robot_thread.start()    
                # self.hand_eye_thread.start()
                # self.td_cam_thread.start()

                while self.running and viewer.is_running():   
                    start_time = time.perf_counter()       

                    with self.lock:                        

                        rclpy.spin_once(self.sm, timeout_sec=0.001) # for scene monitor
                        viewer.sync()  # 화면 업데이트          

                    self.time_sync(1/self.fps, start_time, False)

                    # print(time.perf_counter())
                    # print("Time in main node: ", self.ctrl_step)

                    
        except KeyboardInterrupt:
            print("\nSimulation interrupted. Closing viewer...")
            self.running = False
            self.robot_thread.join()
            # self.hand_eye_thread.join()
            # self.td_cam_thread.join()
            self.sm.destroy_node()


    def robot_control(self):
        self.ctrl_step = 0
        # sync_step = int(1/self.fps*self.ctrl_freq)
        sync_step = 30 # evey 30 ctrl_steps

        try:
            while rclpy.ok() and self.running:            
                with self.lock:
                    start_time = time.perf_counter()                        

                    mujoco.mj_step(self.model, self.data)  # 시뮬레이션 실행

                    self.rc.updateModel(self.data, self.ctrl_step)

                    # -------------------- ADD Controller ---------------------------- #
                    rclpy.spin_once(self.rc.th, timeout_sec=0.0001) # for scene monitor

                    self.data.ctrl[:self.ctrl_dof] = self.rc.compute()   

                    # ---------------------------------------------------------------- #
                    if self.ctrl_step % sync_step == 0:
                        self.hand_eye.allow_to_pub = True
                        self.top_down_cam.allow_to_pub = True

                    # print("Time in robot node: ", self.syn_cnt)
                    self.ctrl_step += 1

                self.time_sync(self.dt, start_time, False)
            
        except KeyboardInterrupt:
            self.get_logger().into("\nSimulation interrupted. Closing robot controller ...")
            # self.rc.tm.destroy_node()
            self.rc.destroy_node()


    def hand_eye_control(self):
        renderer = mujoco.Renderer(self.model, width=self.width, height=self.height)
        hand_eye_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)

        while rclpy.ok() and self.running:            
            with self.lock:
                start_time = time.perf_counter()  

                if self.hand_eye.allow_to_pub: 
                    # hand eye view
                    renderer.update_scene(self.data, camera=hand_eye_id)
                    self.hand_eye.getImage(renderer.render(), self.ctrl_step)
                    rclpy.spin_once(self.hand_eye, timeout_sec=0.001) # for hand eye

            # self.time_sync(1/self.fps, start_time, False)
            self.time_sync(self.dt, start_time, False)
        self.hand_eye.destroy_node()

    def td_cam_control(self):
        # 카메라 렌더러 설정
        renderer = mujoco.Renderer(self.model, width=640, height=480)
        top_down_cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "top_down_cam")

        while rclpy.ok() and self.running:            
            with self.lock:
                start_time = time.perf_counter()    
                
                if self.top_down_cam.allow_to_pub: 
                    # top down view
                    renderer.update_scene(self.data, camera=top_down_cam_id)
                    self.top_down_cam.getImage(renderer.render(), self.ctrl_step)     
                    rclpy.spin_once(self.top_down_cam, timeout_sec=0.001) # for hand eye

            # self.time_sync(1/self.fps, start_time, False)
            self.time_sync(self.dt, start_time, False)

        self.top_down_cam.destroy_node()


    def time_sync(self, target_dt, t_0, verbose=False):
        elapsed_time = time.perf_counter() - t_0
        sleep_time = target_dt - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

        if verbose:
            print(f'Time {elapsed_time*1000:.4f} + {sleep_time*1000:.4f} = {(elapsed_time + sleep_time)*1000} ms')