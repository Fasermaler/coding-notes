#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Faser
class Faser(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Faser')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Maler
class Maler(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Maler')
        return 'outcome2'
        



# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['EXIT', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Faser', Faser(), 
                               transitions={'outcome1':'Faser', 
                                            'outcome2':'EXIT'})
        smach.StateMachine.add('Maler', Maler(), 
                               transitions={'outcome2':'Faser'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
