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
            mujoco.mjtObj.mjOBJ_UNKNOWN,       
            mujoco.mjtObj.mjOBJ_BODY,                     
            mujoco.mjtObj.mjOBJ_XBODY,                    
            mujoco.mjtObj.mjOBJ_JOINT,                    
            mujoco.mjtObj.mjOBJ_DOF,                      
            mujoco.mjtObj.mjOBJ_GEOM,                     
            mujoco.mjtObj.mjOBJ_SITE,                     
            mujoco.mjtObj.mjOBJ_CAMERA,                   
            mujoco.mjtObj.mjOBJ_LIGHT,                    
            mujoco.mjtObj.mjOBJ_FLEX,                     
            mujoco.mjtObj.mjOBJ_MESH,                     
            mujoco.mjtObj.mjOBJ_SKIN,                     
            mujoco.mjtObj.mjOBJ_HFIELD,                   
            mujoco.mjtObj.mjOBJ_TEXTURE,                  
            mujoco.mjtObj.mjOBJ_MATERIAL,                   
            mujoco.mjtObj.mjOBJ_PAIR,                        
            mujoco.mjtObj.mjOBJ_EXCLUDE,                    
            mujoco.mjtObj.mjOBJ_EQUALITY,                  
            mujoco.mjtObj.mjOBJ_TENDON,                   
            mujoco.mjtObj.mjOBJ_ACTUATOR,                 
            mujoco.mjtObj.mjOBJ_SENSOR,
            mujoco.mjtObj.mjOBJ_NUMERIC,                  
            mujoco.mjtObj.mjOBJ_TEXT,                     
            mujoco.mjtObj.mjOBJ_TUPLE,                    
            mujoco.mjtObj.mjOBJ_KEY,                      
            mujoco.mjtObj.mjOBJ_PLUGIN,  
        ]

        self.bs_srv = self.create_service(BlockSortingObs, "block_observation", self.block_sorting_callback)
        self.ph_srv = self.create_service(PegInHoleObs, "pih_observation", self.peg_in_hole_callback)
        self.ta_srv = self.create_service(ToggleAttachment, "toggle_attachment", self.attachment_callback)

        self.offConstraint()

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
                    print(f"  - {name}({obj_id})")

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

    def offConstraint(self):
        for i in range(self.model.eq_type.shape[0]):
            if self.model.eq_type[i] == mujoco.mjtEq.mjEQ_WELD:
                # print(self.model.eq_type[i])
                # print(self.model.equality(i).name)
                id = self.model.equality(i).id
                self.data.eq_active[id] = 0

    # Only Consider pre-defined constraint!!
    def attachObject(self, child, parent):
        
        child_id = self.model.body(child).id
        parent_id = self.model.body(parent).id

        # check the constraint have been declared
        for i in range(self.model.eq_type.shape[0]):
            if self.model.eq_type[i] == mujoco.mjtEq.mjEQ_WELD:
                if (self.model.eq_obj1id[i] == parent_id and self.model.eq_obj2id[i] == child_id):
                    self.get_logger().info(self.model.equality(i).name)
                    id = self.model.equality(i).id

                    # update current relative pose
                    # rel_pos = self.data.xpos[parent_id] - self.data.xpos[child_id]
                    rel_pos = self.data.xpos[child_id] - self.data.xpos[parent_id]

                    child_rot = R.from_quat(self.data.xquat[child_id])
                    parent_rot = R.from_quat(self.data.xquat[parent_id])

                    relative_rot = parent_rot.inv() * child_rot  
                    # relative_rot = parent_rot.inv() * child_rot  
                    rel_quat = relative_rot.as_quat()

                    self.model.eq_data[id][3:6] = [rel_pos[0], rel_pos[1], -rel_pos[2]]
                    # self.model.eq_data[id][3:6] = [0.0, 0.0, -rel_pos[2]]

                    # self.model.eq_data[id][3:6] = [0.0, 0.0, 0.02]
                    self.model.eq_data[id][6:10] = rel_quat#[1.0, 0.0, 0.0, 0.0]
                    
                    self.data.eq_active[id] = 1   

                    break 
            
    def detachObject(self, child, parent):
        child_id = self.model.body(child).id
        parent_id = self.model.body(parent).id
   
        # check the constraint have been declared
        for i in range(self.model.eq_type.shape[0]):
            if self.model.eq_type[i] == mujoco.mjtEq.mjEQ_WELD:
                if (self.model.eq_obj1id[i] == parent_id and self.model.eq_obj2id[i] == child_id):
                    self.get_logger().info(self.model.equality(i).name)
                    id = self.model.equality(i).id
                    self.data.eq_active[id] = 0
                    break

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
        offset = 0.015
        peg.position = Point(x=p[0], y=p[1], z=p[2] + offset)
        peg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        res.hole = hole
        res.peg = peg

        return res
    
    def attachment_callback(self, req, res):       
        child = req.child
        parent = req.parent

        if req.attach:
            self.get_logger().info("Toggle Attachment Request From Client")
            self.attachObject(child, parent)
        else:
            self.get_logger().info("Toggle Detachment Request From Client")
            self.detachObject(child, parent)

        res.success = True

        return res
