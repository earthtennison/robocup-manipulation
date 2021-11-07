#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def talker():
    pub = rospy.Publisher('object_coor', Pose, queue_size=10)
    rospy.init_node('cv_node', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        x = input("X ")
        y = input("y ")
        z = input("z ")
        row = input("row ")
        pitch = input("pitch ")
        yaw = input("yaw ")
        q = quaternion_from_euler(row, pitch, yaw)

        pose_goal = Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        rospy.loginfo(pose_goal)
        pub.publish(pose_goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
        
    except rospy.ROSInterruptException:
        pass