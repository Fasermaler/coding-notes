#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define the main state 1 
class main_state_1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['FAILED','COMPLETED_ACTION'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing main state 1')
        if self.counter < 4:
            self.counter += 1
            return 'FAILED'
        else:
            return 'COMPLETED_ACTION'

# define main state 2
class main_state_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['COMPLETED_ACTION'])

    def execute(self, userdata):
        rospy.loginfo('Executing main state 2')
        return 'COMPLETED_ACTION'



# define recovery state
class recovery_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['COMPLETED_ACTION'])

    def execute(self, userdata):
        rospy.loginfo('Executing recovery state 1')
        return 'COMPLETED_ACTION'
        






def main():
    rospy.init_node('smach_example_state_machine')
    # Create the main SMACH state machine
    sm_main = smach.StateMachine(outcomes=['ERROR', 'EXIT'])

    # Open the container
    with sm_main:

        # Add states to the container
        smach.StateMachine.add('STATE1', main_state_1(), 
                               transitions={'FAILED':'ERROR', 
                                            'COMPLETED_ACTION':'STATE2'})
        smach.StateMachine.add('STATE2', main_state_2(), 
                               transitions={'COMPLETED_ACTION':'EXIT'})


    # Create the recovery SMACH state machine
    sm_recv = smach.StateMachine(outcomes=['COMPLETE'])

    # Open the container
    with sm_recv:

        # Add states to the container
        smach.StateMachine.add('RECOVERY_STATE', recovery_state(), 
                               transitions={'COMPLETED_ACTION':'COMPLETE'})



    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['EXIT_TOP'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('RECOVERY', sm_recv,
                               transitions={'COMPLETE':'MAIN'})

        

        smach.StateMachine.add('MAIN', sm_main,
                               transitions={'EXIT':'EXIT_TOP',
                                            'ERROR':'RECOVERY'})

    # Execute SMACH plan
    outcome = sm_top.execute()



if __name__ == '__main__':
    main()
