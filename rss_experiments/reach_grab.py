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

def drive_to_reach_request(robot, link_name, xyz_targ, quat_targ):
    n_steps = 20
    n_dof = 10
    fixed_base_n_steps = 5 # number of steps whose base should all be equal at the end
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "rightarm+base",
            "start_fixed" : True,
            "dofs_fixed": [[-1,n_steps-fixed_base_n_steps,n_steps],[-2,n_steps-fixed_base_n_steps,n_steps],[-3,n_steps-fixed_base_n_steps,n_steps]]
        },
        "costs" : [
        {
            "type" : "joint_vel",
            "params": {"coeffs" : np.r_[np.ones(n_dof-3), 100*np.ones(3)].tolist()}
        },            
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.01]}
        },
#        {
#            "type" : "pose",
#            "name" : "final_pose",
#            "params" : {
#                "pos_coeffs" : [200,200,200],
#                "rot_coeffs" : [100,100,100],
#                "xyz" : list(xyz_targ),
#                "wxyz" : list(quat_targ),
#                "link" : link_name,
#            },
#        }
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
            "type" : "cart_vel",
            "name" : "cart_vel",
            "params" : {
                "distance_limit" : .01,
                "first_step" : n_steps,
                "last_step" : n_steps-1, #inclusive
                "link" : link_name
            },
        }
        ],
        "init_info" : {
            "type" : "stationary"
        }
    }
    
    return request    

if rospy.get_name() == "/unnamed": rospy.init_node('cloud_drive_reach_live1')

##################
sim = False
cloud_topic = "/cloud_pcd"
#cloud_topic = "/drop/points_self_filtered"
##################

if sim:
    frame_publisher_thread = Thread(target = publish_sim_kinect_frame)
    frame_publisher_thread.start()

pr2 = PR2.create()
env = pr2.env
robot = pr2.robot

#xyz_targ, wxyz_targ = select_waypoint("reach_cup_target")
xyz_targ, wxyz_targ = select_waypoint("trash")

#cloud = get_cloud(cloud_topic, pr2)
print "waiting for point cloud on " + cloud_topic
import sensor_msgs.msg as sm
pc = rospy.wait_for_message(cloud_topic, sm.PointCloud2)
print "ok"
xyz = ru.pc2xyz(pc)
xyz = ru.transform_points(xyz, pr2.tf_listener, "base_footprint", pc.header.frame_id)
xyz = xyz.reshape(-1,3).astype('float32')
cloud = cloudprocpy.PointCloudXYZ()
cloud.from2dArray(xyz)

convex_soup.create_convex_soup(cloud, env)


##################

request = drive_to_reach_request(robot, "r_gripper_tool_frame", xyz_targ, wxyz_targ)
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

if sim:
    frame_publisher_thread.join()


