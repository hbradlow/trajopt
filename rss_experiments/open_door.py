#!/usr/bin/env python

import cloudprocpy
import trajoptpy
import openravepy as rave
import numpy as np
import json
from trajoptpy import convex_soup, make_kinbodies
import atexit
import brett2.ros_utils as ru
from brett2.PR2 import PR2
from brett2 import trajectories
import basic_controls
import rospy
from experiments_utils import *
from jds_utils import conversions
import sys

if len(sys.argv) == 1:
    arm_side = "left"
    hinge_name = "door_hinge"
    angle = -np.pi/3
    make_collision_shape = True
elif len(sys.argv) == 5:
    arm_side = sys.argv[1]
    hinge_name = sys.argv[2]
    angle = float(sys.argv[3])*np.pi/180.0
    make_collision_shape = int(sys.argv[4])
else:
    raise NameError('Wrong number of arguments. Usage: recording.py [arm_side hinge_name angle(deg)]')

if arm_side=="left":
    manip_name = "leftarm"
    link_name = "l_gripper_tool_frame"
elif arm_side == "right":
    manip_name = "rightarm"
    link_name = "r_gripper_tool_frame"
else:
    raise NameError('Invalid arm side')

print arm_side
print hinge_name
print angle

if rospy.get_name() == "/unnamed": rospy.init_node('cloud_drive_reach_live')

def drive_to_reach_request(robot, hmats, xyz_targ, wxyz_targ, xyz_hinge, wxyz_hinge):
    n_steps = len(hmats)
    n_dof = 2*7+3
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "rightarm+leftarm+base",
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
                "link" : link_name,
                "pos_coeffs" : [1,1,1],
                "rot_coeffs" : [1,1,1],
                "timestep" : i
            }}
        request["constraints"].append(waypoint_cnt)    

    return request    

##################
#cloud_topic = "/cloud_pcd"
#cloud_topic = "/drop/points_self_filtered"
cloud_topic = "/octomap_point_cloud_centers"
##################

pr2 = PR2.create()
env = pr2.env
robot = pr2.robot
from time import sleep
sleep(1)

transform = robot.GetLink(link_name).GetTransform().astype("float32")
pose_targ = rave.poseFromMatrix(transform)
xyz_targ = pose_targ[4:7]
wxyz_targ = pose_targ[0:4]

xyz_hinge,wxyz_hinge= select_waypoint(hinge_name)

# wapoints
n_steps = 15
angles = np.linspace(0, angle, n_steps)
T_world_handle = rave.matrixFromPose(np.r_[wxyz_targ, xyz_targ])
T_world_hinge = rave.matrixFromPose(np.r_[wxyz_hinge,xyz_hinge])
T_hinge_handle = np.linalg.inv(T_world_hinge).dot(T_world_handle)

hmats = [T_world_hinge.dot(rave.matrixFromAxisAngle([0,0,1],a)).dot(T_hinge_handle) for a in angles]

if make_collision_shape:
    T_box = T_world_handle.copy()
    T_box[:3,2] = T_world_hinge[:3,2]
    T_box[:3,0] = T_world_hinge[:3,3] - T_box[:3,3]
    T_box[:3,0] -= T_box[:3,0].dot(T_box[:3,2])
    T_box[:3,0] /= np.linalg.norm(T_box[:3,0])
    T_box[:3,1] = np.cross(T_box[:3,2],T_box[:3,0])
    door_pose = rave.poseFromMatrix(T_box)
    #select_waypoint("door", False, door_pose[4:], door_pose[:4])
    
    half_extents = np.array([0.2, 0.025, 0.8])
    offset = np.array([0.15-0.2,0.03,0]) # box center offset in gripper frame
    box = make_kinbodies.create_mesh_box(env, [0,0,0], half_extents)
    box.SetTransform(T_box)
    robot.Grab(box)

cloud = get_cloud(cloud_topic, pr2)
convex_soup.create_convex_soup(cloud, env)


##################

request = drive_to_reach_request(robot, hmats, xyz_targ, wxyz_targ, xyz_hinge, wxyz_hinge)
#request = drive_to_reach_request(robot, "base_footprint", xyz_targ, wxyz_targ)
s = json.dumps(request)
print "REQUEST:",s
trajoptpy.SetInteractive(True);
prob = trajoptpy.ConstructProblem(s, env)
result = trajoptpy.OptimizeProblem(prob)

from jds_utils.yes_or_no import yes_or_no
yn = yes_or_no("execute traj?")
if yn:
    traj = result.GetTraj()
    inds = prob.GetDOFIndices()
    trajectories.follow_rave_trajectory(pr2, traj, inds, use_base=True) 
