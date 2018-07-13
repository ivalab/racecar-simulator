#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
import tf.transformations

class OdometryNode:
    # Set publishers
    pub_odom = rospy.Publisher('/vesc/odom', Odometry, queue_size=1)

    def __init__(self):
        # init internals
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None

        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try:
            arrayIndex = msg.name.index('racecar::base_link')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
        self.last_recieved_stamp = rospy.Time.now()

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return
        
        #use the velocity angle (atan2) and the yaw to determine the direction
        #they would be 0 or 180 to each other
        try:
            orient = self.last_received_pose.orientation
            quat = [orient.x, orient.y, orient.z, orient.w]
        
            euler = tf.transformations.euler_from_quaternion(quat)
        
            #getting the magnitude of the velocity and
            #determining forward or backward movement(actual car moves forwrd only)
            v_x = self.last_received_twist.linear.x
            v_y = self.last_received_twist.linear.y
            
            angle = math.atan2(v_y, v_x)
            
            range1 = euler[2] + math.pi/4
            range2 = euler[2] - math.pi/4
        
            num = 1
            if angle > range1 or angle < range2 :
                num = -1 # velocity and car orintation are in the same direction
            v = num*math.sqrt(math.pow(v_x, 2) + math.pow(v_y, 2))
        
            cmd = Odometry()
            cmd.header.stamp = self.last_recieved_stamp
            cmd.header.frame_id = 'odom'
            cmd.child_frame_id = 'base_link'
            cmd.pose.pose = self.last_received_pose
            cmd.twist.twist.linear.x = v
            cmd.twist.twist.linear.y = 0
            cmd.twist.twist.linear.z = 0
            cmd.twist.twist.angular = self.last_received_twist.angular
            self.pub_odom.publish(cmd)
            
            tf10 = TransformStamped(
                header=Header(
                    frame_id=cmd.header.frame_id,
                    stamp=cmd.header.stamp
                ),
                child_frame_id=cmd.child_frame_id,
                transform=Transform(
                    translation=cmd.pose.pose.position,
                    rotation=cmd.pose.pose.orientation
                )
            )
            self.tf_pub.sendTransform(tf10)
            
        except Exception as e:
            rospy.loginfo(e)

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
rospy.spin()
