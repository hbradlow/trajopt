import cloudprocpy
import trajoptpy
import openravepy as rave
import numpy as np
import json
from trajoptpy import convex_soup
import atexit

def drive_to_reach_request(robot, link_name, xyz_targ, quat_targ):
        
    request = {
        "basic_info" : {
            "n_steps" : 10,
            "manip" : "rightarm+base+r_gripper_l_finger_joint",
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "joint_vel",
            "params": {"coeffs" : np.r_[np.ones(8), 10*np.ones(3)].tolist()}
        },            
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.01]}
        }
        ],
        "constraints" : [
        {
            "type" : "pose",
            "name" : "final_pose",
            "params" : {
                "pos_coeffs" : [1,1,1],
                "rot_coeffs" : [1,1,1],
                "xyz" : list(xyz_targ),
                "wxyz" : list(quat_targ),
                "link" : link_name,
            },
        }
        ],
        "init_info" : {
            "type" : "stationary"
        }
    }

    return request    


env = rave.RaveGetEnvironment(1)
if env is None:
    env = rave.Environment()
    env.StopSimulation()
    env.Load('robots/pr2-beta-static.zae')
    atexit.register(rave.RaveDestroy)

robot = env.GetRobots()[0]
robot.SetDOFValues(np.zeros(robot.GetDOF()))
robot_transform = np.eye(4)
#robot_transform[2,3] = 2
robot.SetTransform(robot_transform)
rgj = robot.GetJoint("r_gripper_l_finger_joint")
robot.SetDOFValues([rgj.GetLimits()[-1]], [rgj.GetDOFIndex()])
tlj = robot.GetJoint("torso_lift_joint")
robot.SetDOFValues([tlj.GetLimits()[-1]], [tlj.GetDOFIndex()])
    
print "processing point cloud"

T = np.eye(4)
T[:3,3] = [ 1.27   , -0.2   ,  0.790675]

if env.GetKinBody("convexsoup") is None:        
    cloud = cloudprocpy.readPCDXYZ("../bigdata/laser_cloud.pcd")
    cloud = cloudprocpy.boxFilter(cloud, -1,5,-5,5,.1,2)
    aabb = robot.GetLink("base_link").ComputeAABB()
    (xmin,ymin,zmin) = aabb.pos() - aabb.extents()
    (xmax,ymax,zmax) = aabb.pos() + aabb.extents()
    cloud = cloudprocpy.boxFilterNegative(cloud, xmin,xmax,ymin,ymax,zmin,zmax)
    #cloud = cloudprocpy.downsampleCloud(cloud, .015)
    convex_soup.create_convex_soup_dynamic(cloud, env, T)

import IPython
viewer = trajoptpy.GetViewer(env)
IPython.lib.inputhook.set_inputhook(viewer.Step)
print "done processing"

##################
#handle = trajoptpy.GetCollisionChecker(env).PlotCollisionGeometry()


pose = rave.poseFromMatrix(T)
request = drive_to_reach_request(robot, "r_gripper_tool_frame", pose[4:7], pose[0:4])
s = json.dumps(request)
print "REQUEST:",s
trajoptpy.SetInteractive(True);
prob = trajoptpy.ConstructProblem(s, env)
result = trajoptpy.OptimizeProblem(prob)
