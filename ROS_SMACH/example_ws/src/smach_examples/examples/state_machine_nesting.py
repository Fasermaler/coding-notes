#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Fas
class Fas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Fas')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Ser
class Ser(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Ser')
        return 'outcome1'
        


# define state Maler
class Maler(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Maler')
        return 'outcome3'




def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['EXIT_TOP'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('Maler', Maler(),
                               transitions={'outcome3':'SUB'})

        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['EXIT_SUB'])

        # Open the container
        with sm_sub:

            # Add states to the container
            smach.StateMachine.add('Fas', Fas(), 
                                   transitions={'outcome1':'Ser', 
                                                'outcome2':'EXIT_SUB'})
            smach.StateMachine.add('Ser', Ser(), 
                                   transitions={'outcome1':'Fas'})

        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'EXIT_SUB':'EXIT_TOP'})

    # Execute SMACH plan
    outcome = sm_top.execute()



if __name__ == '__main__':
    main()
