#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Faser
class Faser(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             input_keys=['faser_counter_in'],
                             output_keys=['faser_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Faser')
        if userdata.faser_counter_in < 3:
            userdata.faser_counter_out = userdata.faser_counter_in + 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Maler
class Maler(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['maler_counter_in'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.loginfo('Counter = %f'%userdata.maler_counter_in)        
        return 'outcome1'
        




def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['EXIT'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Faser', Faser(), 
                               transitions={'outcome1':'Maler', 
                                            'outcome2':'EXIT'},
                               remapping={'faser_counter_in':'sm_counter', 
                                          'faser_counter_out':'sm_counter'})
        smach.StateMachine.add('Maler', Maler(), 
                               transitions={'outcome1':'Faser'},
                               remapping={'maler_counter_in':'sm_counter'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
