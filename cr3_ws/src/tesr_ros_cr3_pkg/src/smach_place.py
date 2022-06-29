import roslib
import rospy
import smach
import smach_ros

class Input(smach.State):
    def __init__(self):
        rospy.loginfo('initiating input state')
        smach.State.__init__(self, outcomes = ['continue_CreateEnvironment'])

    def execute(self, userdata):
        return 'continue_CreateEnvironment'

# --- sm_place -------

class CreateEnvironment(smach.State):
    def __init__(self):
        rospy.loginfo('initiating create environment state')
        smach.State.__init__(self, outcomes = ['continue_GetObjectPoseList'])

    def execute(self, userdata):
        return 'continue_GetObjectPoseList'

class GetObjectPoseList(smach.State):
    def __init__(self):
        rospy.loginfo('initiating get object pose list state')
        smach.State.__init__(self, outcomes = ['continue_Place'])

    def execute(self, userdata):
        return 'continue_Place'

class Place(smach.State):
    def __init__(self):
        rospy.loginfo('initiating place state')
        smach.State.__init__(self, outcomes =['continue_aborted','continue_succeeded'])
    def execute(self, userdata):
        if True:
            return 'continue_aborted'
        else:
            return 'continue_succeeded'

def main():
    rospy.init_node('smach_sm_place_state_machine')

    sm_top = smach.StateMachine(outcomes = ['succeeded1', 'aborted1'])
    with sm_top:
        smach.StateMachine.add('INPUT', Input(),
                                transitions = {'continue_CreateEnvironment':'CREATEENVIRONMENT'})
        
        smach.StateMachine.add('CREATEENVIRONMENT', CreateEnvironment(),
                                   transitions = {'continue_GetObjectPoseList':'SM_PLACE'})

        sm_place = smach.StateMachine(outcomes = ['succeeded', 'aborted'])
        with sm_place:
            smach.StateMachine.add('GETOBJECTPOSELIST', GetObjectPoseList(),
                                   transitions = {'continue_Place':'PLACE'})
            smach.StateMachine.add('PLACE', Place(),
                                   transitions = {'continue_aborted':'aborted',
                                                  'continue_succeeded':'succeeded'})
        smach.StateMachine.add('SM_PLACE', sm_place,
                               transitions = {'aborted':'aborted1','succeeded':'succeeded1'})

    sis = smach_ros.IntrospectionServer('sm_place_server', sm_top, '/SM_PLACE')
    sis.start()
    
    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
