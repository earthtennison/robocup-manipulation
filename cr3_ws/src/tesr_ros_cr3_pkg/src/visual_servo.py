#!/usr/bin/env python
"""
Gets the position of the blob and it commands to steer the wheels

Subscribes to 
    /blob/point_blob
    
Publishes commands to 
    /dkcar/control/cmd_vel    

"""
import math
import time
import threading
import numpy as np

# ros library
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

# cr3 library
from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType


def on_shutdown():
    global client_feedback, client_dashboard
    client_dashboard.DisableRobot()
    client_dashboard.close()
    client_feedback.close()

# The feedback information about port 30003 is displayed
def cr3_feedback():
    global client_feedback, dobot_enable
    global cr3_joint
    global cr3_endpoint
    while dobot_enable:
        time.sleep(0.05)
        all = client_feedback.socket_feedback.recv(10240)
        data = all[0:1440]
        a = np.frombuffer(data, dtype=MyType)
        try:
            if hex((a['test_value'][0]))[:13] == '0x123456789ab':
                print("============== Feed Back ===============")
                cr3_endpoint =  np.around(a['tool_vector_actual'], decimals=4)[0]
                print("cr3_endpoint: [x:{0}] , [y:{1}] , [z:{2}] , [rx:{3}] , [ry:{4}] , [rz:{5}]".format(cr3_endpoint[0],cr3_endpoint[1],cr3_endpoint[2],cr3_endpoint[3],cr3_endpoint[4],cr3_endpoint[5]))                            
                cr3_joint = np.around(a['q_actual'], decimals=4)[0]
                print("cr3_joint: [j1:{0}] , [j2:{1}] , [j3:{2}] , [j4:{3}] , [j5:{4}] , [j6:{5}]".format(cr3_joint[0],cr3_joint[1],cr3_joint[2],cr3_joint[3],cr3_joint[4],cr3_joint[5]))
                print("robot_mode: {}\n safety_mode: {}\nprogram_state: {}\n safety_status: {}".format(a['robot_mode'],a['safety_mode'], a['program_state'], a['safety_status']))
                print("========================================")

                # check robot alarm
                if int(a['robot_mode'][0]) == 9:
                    rospy.logerr("Some error with robot, clearing alarm")
                    client_dashboard.ClearError()
                    time.sleep(0.5)
                    client_dashboard.EnableRobot()
                    time.sleep(0.5)
                elif int(a['robot_mode'][0]) == 7:
                    rospy.loginfo("robot moving...")
                elif int(a['robot_mode'][0]) == 5:
                    rospy.loginfo("robot standby")

        except Exception as e:
            rospy.logerr(e)

class VisualServo():
    def __init__(self):
        global cr3_endpoint
        self.blob_x         = 0.0
        self.blob_y         = 0.0
        self._time_detected = 0.0
        
        self.sub_center = rospy.Subscriber("/blob/point_blob", Point, self.update_position)
        rospy.loginfo("Subscribers set")
        
        self._time_steer        = 0
        self._steer_sign_prev   = 0
        self.stable_pose = cr3_endpoint
        
    @property
    def is_detected(self): return(time.time() - self._time_detected < 1.0)
        
    def update_position(self, message):
        self.blob_x = message.x
        self.blob_y = message.y
        self._time_detected = time.time()
        # rospy.loginfo("Ball detected: %.1f  %.1f "%(self.blob_x, self.blob_y))

    def arm_command(self):
        """
        +Z           
        |         +y   Camera frame
        |         |     o object
        |         |
        |         o---> +x
        |
        0------------------> -Y Robot Base Frame

        """
        global client_feedback

        rospy.loginfo("commanding for x,y = {},{}".format(self.blob_x, self.blob_y))
        x_tolerance = 10
        y_tolerance = 10
        if self.is_detected:

            # safty check the endpoint coordinates
            x_eff, y_eff, z_eff = cr3_endpoint[0], cr3_endpoint[1], cr3_endpoint[2]
            rospy.loginfo("cr3_endpoint: [x:{0}] , [y:{1}] , [z:{2}]".format(x_eff, y_eff, z_eff))
            if not -200 < y_eff < 200:
                rospy.logerr("cr3 out of workspace!")
                client_dashboard.ResetRobot()
            else:
                if self.blob_x > x_tolerance:
                    rospy.loginfo("moving +x")
                    client_feedback.MoveJog("Y-")
                elif self.blob_x < -1*x_tolerance:
                    rospy.loginfo("moving -x")
                    client_feedback.MoveJog("Y+")
                else:
                    rospy.loginfo("stop")
                    client_dashboard.ResetRobot()

            # if self.blob_y > y_tolerance:
            #     rospy.loginfo("moving +z")
            #     client_feedback.MoveJog("Z+")
            # elif self.blob_y < -1*y_tolerance:
            #     rospy.loginfo("moving -z")
            #     client_feedback.MoveJog("Z-")
            # else:
            #     rospy.loginfo("stop")
            #     client_dashboard.ResetRobot()
        else:
            rospy.loginfo("stop")
            client_dashboard.ResetRobot()

            
if __name__ == "__main__":

    rospy.init_node('visual_servo', anonymous=True)
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rate = rospy.Rate(10)
    rospy.on_shutdown(on_shutdown)

    # Enable threads on ports 29999 and 30003
    client_dashboard = dobot_api_dashboard('192.168.5.6', 29999)
    client_feedback = dobot_api_feedback('192.168.5.6', 30003)


    client_dashboard.DisableRobot()
    time.sleep(1)

    # Remove alarm
    client_dashboard.ClearError()
    time.sleep(0.5)

    # Description The upper function was enabled successfully
    client_dashboard.EnableRobot()
    time.sleep(0.5)
    # Select user and Tool coordinate system 0
    client_dashboard.User(0)
    client_dashboard.Tool(0)

    # initiate variable

    cr3_joint = [0,0,0,0,0,0]
    cr3_endpoint = [0,0,0,0,0,0]

    msg = JointState()
    msg.header.frame_id = ""
    msg.name = ["joint1","joint2","joint3","joint4","joint5","joint6", "joint8", "joint9"]

    dobot_enable = True

    # run feedback in Background
    t1 = threading.Thread(name="thread1", target=cr3_feedback)
    t1.start()
    
    # rospy.loginfo("start node")
    robot = VisualServo()
    # rospy.loginfo("start")
    while not rospy.is_shutdown():
        try:
            # Initialize the time of publishing
            msg.header.stamp = rospy.Time.now()
            # Joint angle values
            msg.position = list(map(lambda x: x*math.pi / 180, cr3_joint))
            # Publish message
            pub.publish(msg)
            # Increase sequence
            msg.header.seq += 1
            # Change angle value

            robot.arm_command()
            # Delay execution to match rate
            rate.sleep()

        except KeyboardInterrupt:
            dobot_enable = False
            break