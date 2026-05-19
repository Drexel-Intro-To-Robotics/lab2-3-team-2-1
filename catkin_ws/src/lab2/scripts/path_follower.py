#!/usr/bin/env python3

import rospy
import math
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
import tf

class PathFollower:
    # setup
    def __init__(self):
        rospy.init_node('path_follower')
        
        # initial pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Robot parameters - change as needed
        self.linear_speed = 0.15
        self.angular_speed = 0.40
        self.goal_tolerance = 0.10
        self.angle_tolerance = 0.05

        self.path = []
        self.running = False

        # TF listener — converts between coordinate frames
        self.tf_listener = tf.TransformListener()
        
        # Publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_remaining_path = rospy.Publisher('/remaining_path', Path, queue_size=10, latch=True)
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/planned_path', Path, self.path_cb)

        rospy.spin()

    #------------------------callback functions---------------------
    # odometry
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # get the path from astar_planner
    def path_cb(self, msg):
        if self.running:
            rospy.logwarn("Already following a path, ignoring")
            return

        # Wait for TF to be ready
        try:
            self.tf_listener.waitForTransform('odom', 'map', rospy.Time(0), rospy.Duration(3.0))
        except tf.Exception as e:
            rospy.logerr("TF not ready: %s", str(e))
            return

        # Convert every waypoint from map frame → odom frame
        converted = []
        for pose_stamped in msg.poses:
            try:
                # Build a PoseStamped in map frame
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.header.stamp = rospy.Time(0)  # use latest transform
                p.pose = pose_stamped.pose

                # Transform to odom frame
                p_odom = self.tf_listener.transformPose('odom', p)
                converted.append((
                    p_odom.pose.position.x,
                    p_odom.pose.position.y
                ))
            except tf.Exception as e:
                rospy.logerr("Transform failed: %s", str(e))
                return

        self.path = converted
        rospy.loginfo("Path received: %d waypoints (converted to odom frame)", len(self.path))
        self.follow_path()


    #----------MAth functions--------------
    # find angle to pose
    def angle_to(self, tx, ty):
        raw = math.atan2(ty - self.y, tx - self.x)
        return self.normalize_angle(raw)  # always wrap

    # pythagorean
    def distance_to(self, tx, ty):
        return math.sqrt((tx - self.x)**2 + (ty - self.y)**2)

    # normalize angle if necessary
    def normalize_angle(self, a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ------------Robot Motion----------------------

    # stop the robot
    def stop(self):
        self.cmd_pub.publish(Twist())

    # rotate the robot
    def rotate(self, target):
        rate = rospy.Rate(20)
        
        # Check before doing anything
        error = self.normalize_angle(target - self.yaw)
        if abs(error) < self.angle_tolerance:
            return

        while not rospy.is_shutdown():
            error = self.normalize_angle(target - self.yaw)
            if abs(error) < self.angle_tolerance:
                self.stop()
                return
            twist = Twist()
            twist.angular.z = max(-self.angular_speed,min(self.angular_speed, 2.0 * error))
            self.cmd_pub.publish(twist)
            rate.sleep()

    # go to specified point
    def nav_to_point(self, tx, ty):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # check if reached [point]
            if self.distance_to(tx, ty) < self.goal_tolerance:
                self.stop()
                rospy.sleep(0.5)
                return
            heading_error = self.normalize_angle(self.angle_to(tx, ty) - self.yaw)
            twist = Twist()
            twist.linear.x  = self.linear_speed
            twist.angular.z = max(-self.angular_speed, min(self.angular_speed, 1.5 * heading_error))
            self.cmd_pub.publish(twist)
            rate.sleep()
    
    # publisher for showing path
    def publish_remaining_path(self, remaining_waypoints):
        nav_path = Path()
        nav_path.header.frame_id = 'map'
        nav_path.header.stamp = rospy.Time.now()
        for (wx, wy) in remaining_waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            nav_path.poses.append(pose)
        self.pub_remaining_path.publish(nav_path)

    # follow specified path
    def follow_path(self):
        self.running = True
        rate = rospy.Rate(10)

        # Skip current position [0]
        for i, (wx, wy) in enumerate(self.path[1:], start=1):
            self.publish_remaining_path(self.path[i:])

            # Only rotate if the waypoint is far enough to have a meaningful angle
            if self.distance_to(wx, wy) < self.goal_tolerance:
                continue

            self.rotate(self.angle_to(wx, wy))
            rate.sleep()
            self.nav_to_point(wx, wy)
            rate.sleep()

        self.publish_remaining_path([])
        rospy.loginfo("Path complete!")
        self.stop()
        self.running = False

if __name__ == '__main__':
    PathFollower()