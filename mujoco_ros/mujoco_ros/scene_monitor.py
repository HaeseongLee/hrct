from rclpy.node import Node
import mujoco
from mujoco_msgs.srv import BlockSortingObs, PegInHoleObs, ToggleAttachment
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
import numpy as np

class SceneMonitor(Node):
    def __init__(self, model, data):
        super().__init__("scene_monitor")
        self.model = model # mujoco model
        self.data = data  # mujoco data

        self.object_types = [
            mujoco.mjtObj.mjOBJ_BODY,
            mujoco.mjtObj.mjOBJ_JOINT,
            mujoco.mjtObj.mjOBJ_GEOM,
            mujoco.mjtObj.mjOBJ_SITE,
            mujoco.mjtObj.mjOBJ_CAMERA,
            mujoco.mjtObj.mjOBJ_LIGHT,
            mujoco.mjtObj.mjOBJ_TENDON,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            mujoco.mjtObj.mjOBJ_SENSOR
        ]

        self.bs_srv = self.create_service(BlockSortingObs, "block_observation", self.block_sorting_callback)
        self.ph_srv = self.create_service(PegInHoleObs, "pih_observation", self.peg_in_hole_callback)
        self.ta_srv = self.create_service(ToggleAttachment, "toggle_attachment", self.attachment_callback)
        self.get_logger().info("Scene Monitor is ready")

        # self.scene = mujoco.MjvScene(self.model, maxgeom = 1000)
        # self.option = mujoco.MjvOption()
        # self.pert = mujoco.MjvPerturb()
        # self.cam = mujoco.MjvCamera()


    def getObjectInfo(self, obj_id):
        position = self.data.xpos[obj_id]
        quaternion = self.data.xquat[obj_id]  # [w, x, y, z]
        return position, quaternion

    def getAllObject(self):
        for obj_type in self.object_types:
            print(f"\n{obj_type.name} Objects:")
            for obj_id in range(self.model.nbody if obj_type == mujoco.mjtObj.mjOBJ_BODY else self.model.ngeom):
                name = mujoco.mj_id2name(self.model, obj_type, obj_id)
                if name:
                    print(f"  - {name}")

    def getTargetObject(self):
        # The target name is supposed to be "box_xx"
        objs = {}

        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)            
            if type(name) == str and "box_" in name:
                p, q = self.getObjectInfo(i)
                objs[name] = {"position": p, "orientation": q}

            if type(name) == str and "basket" in name:
                p, q = self.getObjectInfo(i)
                objs[name] = {"position": p, "orientation": q}

        return objs

    def getSensor(self):
        for i in range(self.model.nsensor):  # 모델에 포함된 센서 개수만큼 반복
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SENSOR, i)     
            sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)  # 센서 ID 가져오기
            print(f"Sensor Name: {name}, Sensor ID: {sensor_id}")

    def attachObject(self, child, parent):

        eq_id = None
        child_id = self.model.body(child).id
        parent_id = self.model.body(parent).id

        # check the constraint have been declared
        for i in range(self.model.eq_data.shape[0]):  
            if self.model.eq_type[i] == mujoco.mjtEq.mjEQ_WELD:
                if (self.model.eq_obj1id[i] == parent_id and self.model.eq_obj2id[i] == child_id) or \
                (self.model.eq_obj1id[i] == child_id and self.model.eq_obj2id[i] == parent_id):
                    eq_id = i
                    break  # if the constraint already exist, save it
            
        if eq_id is not None:
                self.model.eq_active0[eq_id] = 1 # active the constraint
                print(f"Reactivating existing weld constraint (ID: {eq_id})")
        else:
            eq_id = len(self.model.eq_obj1id) - 1  # new constraint ID
            self.model.eq_type[eq_id] = mujoco.mjtEq.mjEQ_WELD
            self.model.eq_obj1id[eq_id] = parent_id
            self.model.eq_obj2id[eq_id] = child_id

            # 상대 위치 및 회전 계산
            rel_pos = self.data.xpos[child_id] - self.data.xpos[parent_id]

            # scipy.spatial.transform : quat 2 rot
            child_rot = R.from_quat(self.data.xquat[child_id])
            parent_rot = R.from_quat(self.data.xquat[parent_id])

            relative_rot = child_rot.inv() * parent_rot  # ex) W_R_E.inv() * W_R_peg = E_R_peg
            rel_quat = relative_rot.as_quat()

            # eq_data에 적용 (위치 3개 + 회전 quaternion 4개)
            self.model.eq_data[eq_id, :3] = rel_pos
            self.model.eq_data[eq_id, 3:7] = rel_quat

            print(f"New weld constraint created (ID: {eq_id})")

    def detachObject(self, child, parent):
        child_id = self.model.body(child).id
        parent_id = self.model.body(parent).id
   
        for i in range(self.model.eq_data.shape[0]):  # checl all constraints 
            if self.model.eq_type[i] == mujoco.mjtEq.mjEQ_WELD:  # whether Weld constraint or not
                if (self.model.eq_obj1id[i] == child_id and self.model.eq_obj2id[i] == parent_id) or \
                (self.model.eq_obj1id[i] == parent_id and self.model.eq_obj2id[i] == child_id):
                    self.model.eq_active[i] = 0  # inactive

    def block_sorting_callback(self, req, res):        
        self.get_logger().info("Get Client Request")
        pose_array = PoseArray()
        
        if req.set_task:
            objs = self.getTargetObject()
        
        res.n_obj = len(objs)
        res.names = []

        for key, value in objs.items():
            name = String()
            name.data = key
            res.names.append(name)

            px, py, pz = value['position']
            qw, qx, qy, qz = value['orientation']
            pose = Pose()            
            pose.position = Point(x=px, y=py, z=pz)
            pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            pose_array.poses.append(pose)
            
        res.poses = pose_array
        return res
    

    def peg_in_hole_callback(self, req, res):       
        if req.set_task:
            self.get_logger().info("Get Client Request")
    
        hole = Pose()
        peg = Pose()
        
        hole_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "hole")            
        peg_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "peg")            

        p, q = self.getObjectInfo(hole_id)
        hole.position = Point(x=p[0], y=p[1], z=p[2])
        hole.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        p, q = self.getObjectInfo(peg_id)
        offset = 0.01
        peg.position = Point(x=p[0], y=p[1], z=p[2] + offset)
        peg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        res.hole = hole
        res.peg = peg

        return res
    
    def attachment_callback(self, req, res):       

        self.get_logger().info("Toggle Attachment Request From Client")

        child = req.child
        parent = req.parent

        if req.attach:
            self.attachObject(child, parent)
        else:
            self.detachObject(child, parent)

        res.success = True

        return res
