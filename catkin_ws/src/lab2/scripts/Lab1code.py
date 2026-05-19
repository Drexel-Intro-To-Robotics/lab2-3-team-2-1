#!/usr/bin/env python3

import math
import rospy

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import csv
import time
from sensor_msgs.msg import Imu


class myTurtle:

    def __init__(self):
        rospy.init_node("tb3_motion_node", anonymous=True)

        # Publishers and subscribers
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.nav_to_pose)
        # IMU subscriber
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_cb)

        # IMU state
        self.imu_ax = 0.0
        self.imu_ay = 0.0
        self.imu_az = 0.0
        self.imu_wx = 0.0
        self.imu_wy = 0.0
        self.imu_wz = 0.0

        # CSV setup
        self.csv_file = open('/workspaces/lab2-3-team-2-1/catkin_ws/robot_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time',
            'odom_x', 'odom_y', 'odom_yaw',
            'vel_linear', 'vel_angular',
            'imu_ax', 'imu_ay', 'imu_az',
            'imu_wx', 'imu_wy', 'imu_wz'
        ])

        self.start_time = time.time()
        self.last_cmd = Twist()  # track last published velocity

        self.rate = rospy.Rate(20)

        # Robot pose from odometry
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False

        rospy.loginfo("tb3_motion_node started")

    def odom_cb(self, msg):
        # Update robot position and yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.yaw = self.convert_to_euler(q)

        self.odom_ready = True

    def imu_cb(self, msg):
        self.imu_ax = msg.linear_acceleration.x
        self.imu_ay = msg.linear_acceleration.y
        self.imu_az = msg.linear_acceleration.z
        self.imu_wx = msg.angular_velocity.x
        self.imu_wy = msg.angular_velocity.y
        self.imu_wz = msg.angular_velocity.z

    def nav_to_pose(self, goal):
        # Move to clicked goal in RViz
        rospy.loginfo("New goal received")

        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y

        q = goal.pose.orientation
        _, _, goal_yaw = self.convert_to_euler(q)

        # Face the goal
        dx = goal_x - self.x
        dy = goal_y - self.y
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.yaw)
        self.rotate(angle_error)

        # Move straight to goal
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        self.drive_straight(distance, 0.15)

        # Match final direction
        final_error = self.normalize_angle(goal_yaw - self.yaw)
        self.rotate(final_error)

        self.stop()
        rospy.loginfo("Goal reached")

    def stop(self):
        # Stop the robot
        twist = Twist()

        for _ in range(10):
            self.publish_cmd(twist)
            self.rate.sleep()

    def drive_straight(self, dist, vel):
        # Drive straight for given distance
        self.wait_for_odom()

        start_x = self.x
        start_y = self.y

        twist = Twist()
        direction = 1.0 if dist >= 0 else -1.0
        twist.linear.x = direction * abs(vel)

        while not rospy.is_shutdown():
            dx = self.x - start_x
            dy = self.y - start_y
            travelled = math.sqrt(dx**2 + dy**2)

            if travelled >= abs(dist):
                break

            self.publish_cmd(twist)
            self.rate.sleep()

        self.stop()

    def rotate(self, angle):
        # Rotate by angle in radians
        self.wait_for_odom()

        start_yaw = self.yaw
        target_yaw = self.normalize_angle(start_yaw + angle)

        twist = Twist()
        twist.angular.z = 0.5 if angle >= 0 else -0.5

        while not rospy.is_shutdown():
            error = self.normalize_angle(target_yaw - self.yaw)

            if abs(error) < 0.02:
                break

            self.publish_cmd(twist)
            self.rate.sleep()

        self.stop()

    def spin_wheels(self, u1, u2, run_time):
        # Approximate left and right wheel motion using cmd_vel
        L = 0.16

        twist = Twist()
        twist.linear.x = (u1 + u2) / 2.0
        twist.angular.z = (u2 - u1) / L

        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()

            if current_time - start_time >= run_time:
                break

            self.publish_cmd(twist)
            self.rate.sleep()

        self.stop()

    def drive_circle(self, radius, linear_speed=0.15):
        # Drive one full circle
        if radius <= 0:
            rospy.logwarn("Radius should be positive")
            return

        angular_speed = linear_speed / radius
        run_time = (2.0 * math.pi) / angular_speed

        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()

            if current_time - start_time >= run_time:
                break

            self.publish_cmd(twist)
            self.rate.sleep()

        self.stop()

    def drive_square(self, side_length=0.5, linear_speed=0.15):
        # Drive a square path
        for _ in range(4):
            self.drive_straight(side_length, linear_speed)
            rospy.sleep(0.5)
            self.rotate(math.pi / 2.0)
            rospy.sleep(0.5)

    def random_dance(self):
        # Small demo movement
        self.spin_wheels(0.1, -0.1, 1.0)
        rospy.sleep(0.3)

        self.drive_straight(0.2, 0.15)
        rospy.sleep(0.3)

        self.rotate(math.pi / 2.0)
        rospy.sleep(0.3)

        self.spin_wheels(-0.1, 0.1, 1.0)
        rospy.sleep(0.3)

        self.drive_straight(0.15, 0.15)
        rospy.sleep(0.3)

        self.rotate(-math.pi / 2.0)
        rospy.sleep(0.3)

        self.stop()

    def convert_to_euler(self, quat):
        # Quaternion to roll, pitch, yaw
        q = [quat.x, quat.y, quat.z, quat.w]
        return euler_from_quaternion(q)

    def normalize_angle(self, angle):
        # Keep angle between -pi and pi
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def wait_for_odom(self):
        # Wait until odom data starts coming
        while not rospy.is_shutdown() and not self.odom_ready:
            self.rate.sleep()
    
    def log_data(self):
        t = time.time() - self.start_time
        self.csv_writer.writerow([
            round(t, 4),
            round(self.x, 4),      round(self.y, 4),   round(self.yaw, 4),
            round(self.last_cmd.linear.x, 4),           round(self.last_cmd.angular.z, 4),
            round(self.imu_ax, 4), round(self.imu_ay, 4), round(self.imu_az, 4),
            round(self.imu_wx, 4), round(self.imu_wy, 4), round(self.imu_wz, 4)
        ])
    def cleanup(self):
        self.stop()
        self.csv_file.close()
        rospy.loginfo("Data saved to robot_data.csv")
    
    def publish_cmd(self, twist):
        self.last_cmd = twist
        self.log_data()
        self.cmd_pub.publish(twist)

def main():
    robot = myTurtle()

    rospy.sleep(2.0)

    rospy.loginfo("Starting turtlebot demo")

    rospy.loginfo("Circle")
    robot.drive_circle(0.5)
    rospy.sleep(2)

    rospy.loginfo("Square")
    robot.drive_square(0.5)
    rospy.sleep(2)

    rospy.loginfo("Dance")
    robot.random_dance()
    rospy.sleep(2)

    rospy.loginfo("Straight")
    robot.drive_straight(0.5, 0.15)
    rospy.sleep(2)

    rospy.loginfo("Rotate")
    robot.rotate(math.pi / 2)
    rospy.sleep(2)

    robot.stop()
    rospy.loginfo("All tasks complete")

    rospy.spin()


if __name__ == "__main__":
    main()
