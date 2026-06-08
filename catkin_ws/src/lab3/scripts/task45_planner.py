import rospy
import moveit_commander
from geometry_msgs.msg import Pose

rospy.init_node('task4_planner')
arm = moveit_commander.MoveGroupCommander("arm")


# Set desired planner
arm.set_planner_id("RRTConnectkConfigDefault")
# arm.set_planner_id("PRMkConfigDefault")
# arm.set_planner_id("KPIECEkConfigDefault")

# Define end-effector pose in task space (meters + quaternion)
target_pose = Pose()

#--------------Pose 1---------------
target_pose.position.x = 0.23
target_pose.position.y = 0.0
target_pose.position.z = 0.275

#--------------Pose 2---------------
# target_pose.position.x = -0.06
# target_pose.position.y = -0.30
# target_pose.position.z = 0.3

#--------------Pose 3---------------
# target_pose.position.x = -0.06
# target_pose.position.y = 0.21
# target_pose.position.z = 0.2




# Orientation as quaternion (w=1 = no rotation from base)
target_pose.orientation.x = 0.0
target_pose.orientation.y = 0.0
target_pose.orientation.z = 0.0
target_pose.orientation.w = 1.0

# Set the target and plan
arm.set_pose_target(target_pose)

success, plan, planning_time, error_code = arm.plan()

if success:
    arm.execute(plan, wait=True)
    arm.stop()
    arm.clear_pose_targets()
else:
    rospy.logwarn("Planning failed!")


arm.stop()
arm.clear_pose_targets()