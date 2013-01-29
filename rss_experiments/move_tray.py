#!/usr/bin/env python

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
import time
from threading import Thread
import roslib; 
roslib.load_manifest("tf")
import rospy
from experiments_utils import *
import sys

link1_name = "l_gripper_tool_frame"
link2_name = "r_gripper_tool_frame"

if len(sys.argv) == 1:
    arm_side = "left"
    target_name = "move_tray"
elif len(sys.argv) == 3:
    arm_side = sys.argv[1]
    target_name = sys.argv[2]
else:
    raise NameError('Wrong number of arguments. Usage: recording.py [arm_side target_name]')

if arm_side=="left":
    link_name = "l_gripper_tool_frame"
elif arm_side == "right":
    link_name = "r_gripper_tool_frame"
else:
    raise NameError('Invalid arm side')

print arm_side
print target_name

def move_tray_request(robot, link_name, xyz_targ, quat_targ, xyz_rel, quat_rel):
    n_steps = 10
    n_dof = 2*7+3
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "leftarm+rightarm+base",
            "start_fixed" : True,
        },
        "costs" : [
        {
            "type" : "joint_vel",
            "params": {"coeffs" : np.r_[np.ones(n_dof-4), 10*np.ones(3)].tolist()}
        },            
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.01]}
        },
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
        },
        {
            "type" : "rel_pose",
            "name" : "gripper_relative_pose",
            "params" : {
                "pos_coeffs" : [1,1,1],
                "rot_coeffs" : [1,1,1],
                "xyz" : list(xyz_rel),
                "wxyz" : list(quat_rel),
                "link1" : link1_name,
                "link2" : link2_name,
            },
        },
        ],
        "init_info" : {
            "type" : "stationary"
        }
    }
    
    return request    

if rospy.get_name() == "/unnamed": rospy.init_node('reach_grab')

##################
#cloud_topic = "/cloud_pcd"
cloud_topic = "/drop/points_self_filtered"
##################

pr2 = PR2.create()
env = pr2.env
robot = pr2.robot

xyz_targ, wxyz_targ = select_waypoint(target_name)

# constraint the gripper's relative transform based on their initial transform
T_link1 = robot.GetLink(link1_name).GetTransform().astype("float32")
T_link2 = robot.GetLink(link2_name).GetTransform().astype("float32")
T_relative = np.linalg.inv(T_link1).dot(T_link2) # link2's transform in link1's frame
pose_rel = rave.poseFromMatrix(T_relative)
xyz_rel = pose_rel[4:7]
wxyz_rel = pose_rel[0:4]

cloud = get_cloud(cloud_topic, pr2)
convex_soup.create_convex_soup(cloud, env)


##################

request = move_tray_request(robot, link_name, xyz_targ, wxyz_targ, xyz_rel, wxyz_rel)
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
print "DONE"
