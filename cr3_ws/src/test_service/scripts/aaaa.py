import rospy

from geometry_msgs.msg import Pose
from test_service.srv import *
from gb_visual_detection_3d_msgs.msg import *

class server:
    def __init__(self):
        self.server = rospy.Service('objreq_service',image2obj_service,self.objreq_callback)
        self.client = rospy.ServiceProxy('obj2pose_service',obj2pose_service,persistent=True)
        self.subscriber = rospy.Subscriber('/darknet_ros_3d/bounding_boxes',BoundingBoxes3d,self.darknet3d_callback)

    def objreq_callback(self,req):
        self.obj_name = req.obj_name
        self.obj_request = req.request_obj

        if self.obj_request == True:
            self.req2obj()
            if self.response_obj == True:
                rospy.loginfo("---Sending response Service---")
            else:
                rospy.loginfo("---Sending false response Service as None of pose and size---")

            self.test_serviceRespond(response_obj = self.response_obj, obj_pose = self.obj_pose, obj_size = self.obj_size)
            self.obj_request = False


        else:
            pass

    def darknet3d_callback(self,bbx3d_msg):
        obj_name = self.obj_name
        bbx3d_array = bbx3d_msg.bounding_boxes

        if self.obj2pose_respond == True & self.obj_request ==True:
            bestBBX = self.filterBBX(obj_name,bbx3d_array)

            if bestBBX != None:
                pose_bbx = Pose()
                pose_bbx.position.x = bestBBX[0]
                pose_bbx.position.y = bestBBX[1]
                pose_bbx.position.z = bestBBX[2]
                pose_bbx.orientation.x = 0.0
                pose_bbx.orientation.y = 0.0
                pose_bbx.orientation.z = 0.0
                pose_bbx.orientation.w = 1.0

                self.obj_size = (bestBBX[3],bestBBX[4],bestBBX[5])
                rospy.loginfo("----Server got obj_size----")

                self.obj_pose = pose_bbx
                rospy.loginfo("----Server got obj_pose----")

                self.response_obj = True
                rospy.loginfo("----Server set respond as True----")

            else:
                self.obj_size = None
                rospy.loginfo("----Server got obj_size as None----")

                self.obj_pose = None
                rospy.loginfo("----Server got obj_pose as None----")

                self.response_obj = False
                rospy.loginfo("----Server set respond as False----")

    def req2obj(self):
        self.obj2pose_respond = self.client(obj2pose_request = True)
        if self.obj2pose_respond == True:
            rospy.loginfo("Request to darknet3d and recieve Respond with obj_pose")
        else:
            rospy.logerr("Something wrong darknet3d respond = false")

    def filterBBX(obj_name,bbx3d_array):
        objbbx_list = []
        for bbx in bbx3d_array:
            bbx_name = bbx.Class
            bbx_probability = bbx.probability
            if bbx_name == obj_name:
                x = (bbx.xmax + bbx.xmin)/2
                y = (bbx.ymax + bbx.ymin)/2
                z = (bbx.zmax + bbx.zmin)/2
                l = bbx.xmax - bbx.xmin
                h = bbx.ymax - bbx.ymin
                d = bbx.zmax - bbx.zmin
                distance = ((x**2)+(y**2)+(z**2))**(1/2)
                data = (x,y,z,l,h,d,distance)
                objbbx_list.append(data)

        # -----finish list all bbx-----

        list_size = len(objbbx_list)
        if list_size < 1:
            rospy.loginfo("Cannot detect any %s" %obj_name)
            best_bbx = None
        elif list_size == 1 :
            best_bbx = objbbx_list[0]
        elif list_size > 1:
            best_bbx = min(objbbx_list, key = lambda t: t[6])

        return best_bbx



if __name__ == '__main__':
    rospy.init_node('objreq_server')
    print("Start Object Request Server")
    ObjReqServer1 = server()
    rospy.spin()