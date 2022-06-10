#!/usr/bin/env python

import rospy

from test_service.srv import *

def reqObj_callback(req):
    print("Find Object name: %s"%req.obj_name)
    return detectobjResponse(complete = True)



if __name__ == "__main__":
    rospy.init_node('objdetect')
    srv = rospy.Service('obj_detect', detectobj, reqObj_callback)
    print("Start object detection")
    rospy.spin()