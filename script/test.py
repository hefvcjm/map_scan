#!/usr/bin/env python
import rospy
import tf
from map_scan.srv import GetMapScan
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, PoseStamped, Point, Quaternion, Pose
from tf.transformations import quaternion_from_euler
import random

if __name__ == "__main__":
    rospy.init_node("map_scan_test")
    pub = rospy.Publisher("/map_scan", LaserScan, queue_size=2)
    pose_pub = rospy.Publisher("/laser_pose", PoseStamped, queue_size=2)
    br = tf.TransformBroadcaster()
    br.sendTransform((0.0, 0.0, 0.0),
                     tf.transformations.quaternion_from_euler(0, 0, 1.0),
                     rospy.Time.now(),
                     "virtual_laser",
                     "map")
    print("test start")
    rospy.wait_for_service('/map_scan')
    scan = LaserScan()
    scan.angle_min = -1.0
    scan.angle_max = 1.0
    scan.angle_increment = 0.017
    scan.range_min = 0.01
    scan.range_max = 3.5
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = "virtual_laser"
    while not rospy.is_shutdown():
        # try:
        get_scan = rospy.ServiceProxy('map_scan', GetMapScan)
        # pose = Vector3()
        pose = Pose2D()
        pose.x = random.random()*2
        pose.y = random.random()*2
        pose.theta = random.random()*6
        br.sendTransform((pose.x, pose.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, pose.theta),
                         rospy.Time.now(),
                         "virtual_laser",
                         "map")
        p = PoseStamped()
        p.pose = Pose(Point(0.0, 0.0, 0), Quaternion(
            *quaternion_from_euler(0, 0, 0.0, axes="sxyz")))
        p.header.frame_id = "virtual_laser"
        p.header.stamp = rospy.Time.now()
        pose_pub.publish(p)

        resp = get_scan(pose, scan)
        pub.publish(resp.scan)
        # except:
        #     print("exception")
        rospy.sleep(1)
    rospy.spin()
