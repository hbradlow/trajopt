import cloudprocpy
import trajoptpy
import openravepy as rave
import numpy as np
import json
from trajoptpy import convex_soup
import atexit
import brett2.ros_utils as ru
from brett2.PR2 import PR2
from brett2 import trajectories
import basic_controls
import rospy
from experiments_utils import *
from jds_utils import conversions


if rospy.get_name() == "/unnamed": rospy.init_node('cloud_drive_reach_live')

def drive_to_reach_request(robot, link_name, xyz_targ, wxyz_targ, xyz_hinge, wxyz_hinge, n_steps=10):
    n_steps = 15
    n_dof = 10
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            #"manip" : "rightarm+leftarm+base",
            "manip" : "rightarm+base",
            #r_gripper_l_finger_joint
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "joint_vel",
            "params": {"coeffs" : np.r_[np.ones(n_dof-3), 10*np.ones(3)].tolist()}
        },            
        {
            "type" : "collision",
            "params" : {"coeffs" : [1], "dist_pen" : [0.01]}
        },
        ],
        "constraints" : [
        ],
        "init_info" : {
            "type" : "stationary"
        },
    }

    angles = np.linspace(0, -np.pi*.4, n_steps)
    T_world_handle = rave.matrixFromPose(np.r_[wxyz_targ, xyz_targ])
    T_world_hinge = rave.matrixFromPose(np.r_[wxyz_hinge,xyz_hinge])
    T_hinge_handle = np.linalg.inv(T_world_hinge).dot(T_world_handle)
    
    hmats = [T_world_hinge.dot(rave.matrixFromAxisAngle([0,0,1],a)).dot(T_hinge_handle) for a in angles]

    poses = rave.poseFromMatrices(hmats)
    xyzs = poses[:,4:7]
    quats = poses[:,0:4]

    for i in xrange(0, n_steps):
        waypoint_cnt = {
            "type" : "pose",
            "name" : "waypoint_pose",
            "params" : {
                "xyz" : list(xyzs[i]),
                "wxyz" : list(quats[i]),
                "link" : "r_gripper_tool_frame",
                "pos_coeffs" : [1,1,1],
                "rot_coeffs" : [1,1,1],
                "timestep" : i
            }}
        request["constraints"].append(waypoint_cnt)    

    return request    

##################
#cloud_topic = "/cloud_pcd"
cloud_topic = "/drop/points_self_filtered"
##################

pr2 = PR2.create()
env = pr2.env
robot = pr2.robot
from time import sleep
sleep(1)

transform = robot.GetLink("r_gripper_tool_frame").GetTransform().astype("float32")
pose_targ = rave.poseFromMatrix(transform)
xyz_targ = pose_targ[4:7]
wxyz_targ = pose_targ[0:4]

xyz_hinge,wxyz_hinge= select_waypoint("hinge")

cloud = get_cloud(cloud_topic, pr2)
convex_soup.create_convex_soup(cloud, env)


##################

request = drive_to_reach_request(robot, "r_gripper_tool_frame", xyz_targ, wxyz_targ, xyz_hinge, wxyz_hinge)
#request = drive_to_reach_request(robot, "base_footprint", xyz_targ, wxyz_targ)
s = json.dumps(request)
print "REQUEST:",s
trajoptpy.SetInteractive(True);
prob = trajoptpy.ConstructProblem(s, env)
result = trajoptpy.OptimizeProblem(prob)

#from jds_utils.yes_or_no import yes_or_no
#yn = yes_or_no("execute traj?")
#if yn:
#    traj = result.GetTraj()
#    inds = prob.GetDOFIndices()
#    trajectories.follow_rave_trajectory(pr2, traj, inds, use_base=True) 
