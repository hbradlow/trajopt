import cloudprocpy
import numpy as np
from trajoptpy import convex_soup
import brett2.ros_utils as ru
import basic_controls
import rospy
import roslib; 
roslib.load_manifest("tf")
import rospy
from jds_utils import conversions
import tf

def get_transform(frame1, frame2):
    tf_listener = tf.TransformListener()
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans, rot =  tf_listener.lookupTransform(frame1, frame2, rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
    return trans, rot

def publish_sim_kinect_frame():
    tf_listener = tf.TransformListener()
    
    print "listening"
    trans, rot = get_transform("/odom_combined", "/head_mount_kinect_ir_link")
    print "listen ok"
    
    print "broadcasting"
    tf_broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), "/camera_link", "/odom_combined")
        rate.sleep()

def select_waypoint(name, use_cached=True, xyz_base=[1,0,1], xyzw_base=[0,0,0,1]):
    base_to_odom_tf = conversions.trans_rot_to_hmat(*get_transform("/odom_combined", "/base_link"))
    odom_to_base_tf = np.linalg.inv(base_to_odom_tf)

    if use_cached:
        try:
            tf_odom = np.load("../data/"+name+".npy")
            tf_base = odom_to_base_tf.dot(tf_odom)
            xyz_base,  xyzw_base = conversions.hmat_to_trans_rot(tf_base)
        except IOError:
            pass
    marker = basic_controls.SixDOFControl( False , xyz_base, xyzw_base)
    raw_input("now choose target pose with interactive markers. press enter when done")
    xyz_base = marker.xyz
    xyzw_base = marker.xyzw
    
    tf_base = conversions.trans_rot_to_hmat(xyz_base, xyzw_base)
    tf_odom = base_to_odom_tf.dot(tf_base)
    np.save("../data/"+name+".npy", tf_odom)
    
    return xyz_base, np.r_[xyzw_base[3], xyzw_base[:3]].tolist()

def get_cloud_and_create_convex_soup(cloud_topic, pr2, env):
    print "waiting for point cloud " + cloud_topic
    import sensor_msgs.msg as sm
    pc = rospy.wait_for_message(cloud_topic, sm.PointCloud2)
    print "ok"
    xyz = ru.pc2xyz(pc)
    xyz = ru.transform_points(xyz, pr2.tf_listener, "base_footprint", pc.header.frame_id)
    xyz = xyz.reshape(-1,3).astype('float32')
    cloud = cloudprocpy.PointCloudXYZ()
    cloud.from2dArray(xyz)
    convex_soup.create_convex_soup(cloud, env)
