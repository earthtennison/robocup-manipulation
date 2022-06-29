#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros

class Stand_By(smach.State):
    def __init__(self):
        rospy.loginfo('initiating the stand by state')
        smach.State.__init__(self, outcomes = ['continue_Navigate_To_Shelf'])
    def execute(self, userdata):
        return 'continue_Navigate_To_Shelf'

class Navigate_To_Shelf(smach.State):
    def __init__(self):
        rospy.loginfo('initiating navigate to shelf state')
        smach.State.__init__(self, outcomes = ['continue_Place_Object', 'continue_Save_Catagory'])
        self.x = 1
    def execute(self, userdata):
        if self.x == 1:
            return 'continue_Place_Object'
        else:
            return 'continue_Save_Catagory'

class Place_Object(smach.State):
    def __init__(self):
        rospy.loginfo('initiating place object state')
        smach.State.__init__(self, outcomes = ['continue_succeeded', 'continue_Navigate_To_Table'])
        self.x = 1
    def execute(self, userdata):
        if self.x == 1:
            return 'continue_succeeded'
        else:
            return 'continue_Navigate_To_Table'

class Save_Catagory(smach.State):
    def __init__(self):
        rospy.loginfo('initiating the save catagory state')
        smach.State.__init__(self, outcomes = ['continue_Navigate_To_Table'])
    def execute(self, userdata):
        return 'continue_Navigate_To_Table'

class Navigate_To_Table(smach.State):
    def __init__(self):
        rospy.loginfo('initiating the navigate to table state')
        smach.State.__init__(self, outcomes = ['continue_Pick_Object'])
    def execute(self, userdata):
        return 'continue_Pick_Object'

class Pick_Object(smach.State):
    def __init__(self):
        rospy.loginfo('initiating the Pick Object state')
        smach.State.__init__(self, outcomes = ['continue_Navigate_To_Shelf'])
    def execute(self, userdata):
        return 'continue_Navigate_To_Shelf'

def main():
    rospy.init_node('smach_storing_groceries_state_machine')
    # Create the top level state machine
    sm_top = smach.StateMachine(outcomes = ['succeeded', 'aborted'])

    # Open the container

    with sm_top:
        smach.StateMachine.add('STAND_BY', Stand_By(),
                               transitions = {'continue_Navigate_To_Shelf':'NAVIGATE_TO_SHELF'})
        smach.StateMachine.add('NAVIGATE_TO_SHELF', Navigate_To_Shelf(),
                               transitions = {'continue_Place_Object':'PLACE_OBJECT',
                                              'continue_Save_Catagory':'SAVE_CATAGORY'})
        smach.StateMachine.add('PLACE_OBJECT', Place_Object(),
                               transitions = {'continue_succeeded':'succeeded',
                                              'continue_Navigate_To_Table':'NAVIGATE_TO_TABLE'})
        smach.StateMachine.add('SAVE_CATAGORY', Save_Catagory(),
                               transitions = {'continue_Navigate_To_Table':'NAVIGATE_TO_TABLE'})
        smach.StateMachine.add('NAVIGATE_TO_TABLE', Navigate_To_Table(),
                               transitions = {'continue_Pick_Object':'PICK_OBJECT'})
        smach.StateMachine.add('PICK_OBJECT', Pick_Object(),
                               transitions = {'continue_Navigate_To_Shelf':'NAVIGATE_TO_SHELF'})


    # create and start the introspection server
    sis = smach_ros.IntrospectionServer('storing_groceries_server', sm_top, 'SM_STORING_GROCERIES')
    sis.start()

    # execute the smach state machine
    outcomes = sm_top.execute()

    # wait until ctrl-z to stop the application

    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()
