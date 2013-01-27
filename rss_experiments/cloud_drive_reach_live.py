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


if rospy.get_name() == "/unnamed": rospy.init_node('cloud_drive_reach_live')

def drive_to_reach_request(robot, link_name, xyz_targ, quat_targ):
        
    request = {
        "basic_info" : {
            "n_steps" : 30,
            #"manip" : "rightarm+leftarm+base",
            "manip" : "leftarm+base",
            #r_gripper_l_finger_joint
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "joint_vel",
            #"params": {"coeffs" : np.r_[10*np.ones(3)].tolist()}
            "params": {"coeffs" : np.r_[np.ones(7), 10*np.ones(3)].tolist()}
        },            
        {
            "type" : "collision",
            "params" : {"coeffs" : np.r_[10*np.ones(28), np.ones(2)].tolist(),"dist_pen" : np.r_[0.05*np.ones(28), 0.01*np.ones(2)].tolist()}
        },
        {
            "type" : "pose",
            "name" : "final_pose",
            "params" : {
                "pos_coeffs" : [200,200,200],
                "rot_coeffs" : [200,200,200],
                "xyz" : list(xyz_targ),
                "wxyz" : list(quat_targ),
                "link" : link_name,
            },
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


pr2 = PR2.create()
env = pr2.env
robot = pr2.robot
from time import sleep
sleep(1)

marker = basic_controls.SixDOFControl( False , [1,0,0], [0,0,0,1])
raw_input("now choose target pose with interactive markers. press enter when done")

xyz_targ = marker.xyz
wxyz_targ = np.r_[marker.xyzw[3], marker.xyzw[:3]].tolist()
print xyz_targ
print wxyz_targ

print "waiting for point cloud on /drop/points_self_filtered"
import sensor_msgs.msg as sm
#pc = rospy.wait_for_message("/drop/points_self_filtered", sm.PointCloud2)
pc = rospy.wait_for_message("/cloud_pcd", sm.PointCloud2)
print "ok"
xyz = ru.pc2xyz(pc)
xyz = ru.transform_points(xyz, pr2.tf_listener, "base_footprint", pc.header.frame_id)
xyz = xyz.reshape(-1,3).astype('float32')
np.save("../bigdata/xyz_cloud", xyz)
cloud = cloudprocpy.PointCloudXYZ()
cloud.from2dArray(xyz)
cloud = cloudprocpy.boxFilter(cloud, -1,5,-3,3,.1,2)
#aabb = robot.GetLink("base_link").ComputeAABB()
#(xmin,ymin,zmin) = aabb.pos() - aabb.extents()
#(xmax,ymax,zmax) = aabb.pos() + aabb.extents()
#cloud = cloudprocpy.boxFilterNegative(cloud, xmin,xmax,ymin,ymax,zmin,zmax)
#cloud = cloudprocpy.downsampleCloud(cloud, .015)
convex_soup.create_convex_soup(cloud, env)
#convex_soup.create_convex_soup_dynamic(cloud, env, rave.matrixFromPose(np.r_[wxyz_targ,xyz_targ]))


##################

request = drive_to_reach_request(robot, "l_gripper_tool_frame", xyz_targ, wxyz_targ)
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
