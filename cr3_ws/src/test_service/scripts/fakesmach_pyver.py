#!/usr/bin/env python

import rospy

from test_service.srv import *

if __name__ == "__main__":
    rospy.init_node('smachclient')
    rospy.wait_for_service('obj_detect')
    srv=rospy.ServiceProxy('obj_detect',detectobj)
    try:
        res = srv(obj_name = "lol")
        print(res)
    except:
        print("failed to connect server")
    rospy.spin()