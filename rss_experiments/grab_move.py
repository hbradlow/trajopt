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
import trajoptpy.math_utils as mu

def drive_to_reach_request(robot, link_name, xyz_targ, quat_targ):
    n_steps = 20
    n_dof = 10
    request = {
    	"basic_info" : {
    		"n_steps" : n_steps,
    		"manip" : "rightarm+base",
    		"start_fixed" : True
    	},
    	"costs" : [
    	{
    		"type" : "joint_vel",
    		"params": {"coeffs" : np.r_[np.ones(n_dof-3),10*np.ones(3)].tolist()}
    	},            
    	{
    		"type" : "continuous_collision",
    		"params" : {"coeffs" : [10],"dist_pen" : [0.05]}
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
            "type" : "cart_vel",
            "name" : "cart_vel",
            "params" : {
                "distance_limit" : .1,
                "first_step" : 0,
                "last_step" : n_steps-1, #inclusive
                "link" : link_name
            },
        },
    	],
    	"init_info" : {
    		"type" : "stationary"
    	}
    }
    # remove collision cost
    collision_cost = request["costs"].pop()
    assert collision_cost["type"]=="continuous_collision"
    
    saver = rave.Robot.RobotStateSaver(robot)
    s = json.dumps(request)
    print "REQUEST:",s
    trajoptpy.SetInteractive(False);
    prob = trajoptpy.ConstructProblem(s, env)
    result = trajoptpy.OptimizeProblem(prob)
    del saver # reverts the robot state
    
    traj = result.GetTraj()
    path_init = mu.linspace2d(traj[0,:],traj[-1,:],n_steps)
    request["init_info"]["type"] = "given_traj"
    request["init_info"]["data"] = [x.tolist() for x in path_init]
    
    # add collision cost back again
    request["costs"].append(collision_cost)
    
    return request

if rospy.get_name() == "/unnamed": rospy.init_node('grab_move')

##################
#cloud_topic = "/cloud_pcd"
cloud_topic = "/drop/points_self_filtered"
##################

pr2 = PR2.create()
env = pr2.env
robot = pr2.robot

xyz_targ, wxyz_targ = select_waypoint("trash")

cloud = get_cloud(cloud_topic, pr2)

make_collision_shape = False
if make_collision_shape:
    half_extents = np.array([0.03, 0.03, 0.08])
    offset = np.array([0.05,0,0]) # box center offset in gripper frame
    T_gripper = robot.GetLink("r_gripper_tool_frame").GetTransform()
    T_gripper_offset = np.eye(4)
    T_gripper_offset[:3,3] = offset
    T_box = T_gripper.dot(T_gripper_offset)
    box = make_kinbodies.create_mesh_box(env, [0,0,0], half_extents)
    box.SetTransform(T_box)
    robot.Grab(box)
    
    cloud = cloudprocpy.orientedBoxFilter(cloud, T_box.astype("float32"), -half_extents[0], half_extents[0], -half_extents[1], half_extents[1], -half_extents[2], half_extents[2], True) # removes the object being held

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
    pr2.rgrip.open()
print "DONE"
