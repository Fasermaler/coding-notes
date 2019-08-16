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
    sm_top = smach.StateMachine(outcomes=['outcome6'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('Maler', Maler(),
                               transitions={'outcome3':'CON'})

        # Create the sub SMACH state machine
        sm_con = smach.Concurrence(outcomes=['outcome4','outcome5'],
                                   default_outcome='outcome4',
                                   outcome_map={'outcome5':
                                       { 'Fas':'outcome2',
                                         'Ser':'outcome1'}})

        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('Fas', Fas())
            smach.Concurrence.add('Ser', Ser())

        smach.StateMachine.add('CON', sm_con,
                               transitions={'outcome4':'CON',
                                            'outcome5':'outcome6'})

    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == '__main__':
    main()
