#!/usr/bin/env python

import rospy
import rostest

import threading

import unittest

from smach import *
from smach_ros import *

from smach_msgs.msg import *

### Custom state classe
class Setter(State):
    """State that sets the key 'a' in its userdata"""
    def __init__(self):
        State.__init__(self,['done'],[],['a'])
    def execute(self,ud):
        rospy.sleep(1.0)
        ud.a = 'A'
        rospy.loginfo("Added key 'a'.")
        return 'done'

class Getter(State):
    """State that grabs the key 'a' from userdata, and sets 'b'"""
    def __init__(self):
        State.__init__(self,['done'],['a'],['b'])
    def execute(self,ud):
        while 'a' not in ud and not rospy.is_shutdown():
            #rospy.loginfo("Waiting for key 'a' to appear. ")
            rospy.sleep(0.1)
        ud.b = ud.a
        return 'done'

def main():
    """Test introspection system."""

    rospy.init_node('viewer_test2',log_level=rospy.DEBUG)

    # Construct state machine
    sm = StateMachine(['done'])
    sm2 = StateMachine(['done'])
    sm3 = StateMachine(['done'])

    with sm:
        # Note: the following "Getter" state should fail
        StateMachine.add('2GETTER1', Getter(), {})
        StateMachine.add('2SM2', sm2, {'done':'2SM3'})
        with sm2:
            StateMachine.add("2SETTER", Setter(), {})
        StateMachine.add('2SM3', sm3, {'done':'done'})
        with sm3:
            StateMachine.add("2SETTER", Setter(), {})
        StateMachine.add('2GETTER2', Getter(), {'done':'2SM2'})

    sm.set_initial_state(['2GETTER1'])
    sm2.set_initial_state(['2SETTER'])
    sm3.set_initial_state(['2SETTER'])

    # Run introspector
    server = IntrospectionServer('viewer_test2',sm,'/viewer_test2')
    server_thread = threading.Thread(target=server.start)
    server_thread.start()

    rospy.spin()


if __name__=="__main__":
    main();
