# ROS SMACH - a simple walkthrough

Author: Fasermaler

This is a simple walkthrough on ROS SMACH, the state machine library for ROS. It references heavily from the SMACH tutorial on the [ROS Wiki](http://wiki.ros.org/smach/Tutorials) and references code from [rhaschke's executive_smach_tutorials repository](https://github.com/rhaschke/executive_smach_tutorials). This is also because the repo on eacousineau is already broken and the ROS wiki has yet to update the links.

I have also included instructions on getting smach visualization to work on Kinetic. This is because the official `smach_viewer` has been discontinued since Hydro.

## Pre-requisites

- Knowledge of ROS and how it works
- Basic Knowledge of State Machines
- Knowledge of Action Servers

______

## Table of Contents <a name="top"></a> 
1. [Introduction](#1)</br>
   1.1	[When to use SMACH?](#1.1)</br>
   1.2	[When not to use SMACH?](#1.2)</br>
   1.3	[What can SMACH do for you?](#1.3)</br>
2. [Simple State Machine Example](#2)</br>
   2.1	[Creating States](#2.1)</br>
   2.2	[Creating the SMACH State Machine](#2.2)</br>
   2.3	[Adding States to the State Machine](#2.3)</br>
   2.4	[Full Example](#2.4)</br>
   2.5	[Running the Example](#2.5)</br>
3. [SMACH with Rospy](#3)</br>
   3.1	[Defining 2 States](#3.1)</br>
   3.2	[Creating the State Machine](#3.2)</br>
   3.3	[Running the Example](#3.3)</br>
   3.4	[Viewing the Rospy Info Log](#3.4)</br>
4. [State Machines and Data Transfer](#4)</br>
   4.1	[Definition of States with Userdata](#4.1)</br>
   4.2	[Definition of State Machine with Userdata](#4.2)</br>
   4.3	[Remapping](#4.3)</br>
   4.4	[Running the Example](#4.4)</br>
5. [State Machine in a State Machine](#5)</br>
   5.1	[Creating the States](#5.1)</br>
   5.2	[Creating the Nested State Machine](#5.2)</br>
   5.3	[Running the Example](#5.3)</br>
6. [Calling ROS Actions from State Machine](#6)</br>
   6.1	[Creating the Test Action Server](#6.1)</br>
   6.2	[SMACH SimpleActionServer](#6.2)</br>
   6.2.1	[Basic Initialization (No Goals)](#6.2.1)</br>
   6.2.2	[Initialization with Fixed Goals](#6.2.2)</br>
   6.2.3	[Goal Callback](#6.2.3)</br>
   6.2.4	[Result Message](#6.2.4)</br>
   6.2.5	[Result Callback](#6.2.5)</br>
   6.3	[Simple Action State Definition in Example](#6.3)</br>
7. [Visualization of SMAC State Machines](#7)</br>
   7.1	[Preparations](#7.1)</br>
   7.2	[Installing rqt_smach](#7.2)</br>
   7.3	[Running SMACH Viewer](#7.3)</br>
   7.4	[Running the Example](#7.4)</br>
8. [Concurrence](#8)</br>
   8.1	[Concurrence Syntax](#8.1)</br>
   8.2	[Running the Example](#8.2)</br>
9. [Sequences](#9)</br>
   9.1	[Defining the ExampleState Class](#9.1)</br>
   9.2	[Defining the State Machine and Sequence](#9.2)</br>
   9.3	[Running the Example](#9.3)</br>
10.	[Iterators](#10)</br>
      10.1	[Imports](#10.1)</br>
      10.2	[Constructing the State Machine](#10.2)</br>
      10.3	[Creating the Iteration Container](#10.3)</br>
      10.4	[Running the Example](#10.4)</br>


## 1 Introduction <a name="1"></a>

[go to top](#top)

SMACH is a ROS independent library for building Hierachical state machines. Due to this, it is extremely useful when working with use cases that has many predefined routines or actions. Or anything that requires structured or orderly process flow.

Additionally, SMACH can be used with ROS Actions to allow for increased code modularity. This can be done by embedding ROS Action Clients as part of the SMACH state machine code. The ROS Action Servers are then only called when they are required. This allows for easier unit testing, more orderly process flow and reduction in unnecessary data transfer.

### 1.1 When to use SMACH? <a name="1.1"></a>

[go to top](#top)

- Fast prototyping: The straightforward Python-based SMACH syntax  makes it easy to quickly prototype a state machine and start running it.  

- Complex  state machines: SMACH allows you to design, maintain and debug large,  complex hierarchical state machines. You can find an example of a  complex hierarchical state machine [here](http://wiki.ros.org/pr2_plugs_executive). 

- Introspection: SMACH gives you full introspection in your state machines, state transitions, data flow, etc. See the [smach_viewer](http://wiki.ros.org/smach_viewer) for more details. 

### 1.2 When not to use SMACH? <a name="1.2"></a>

[go to top](#top)

- Unstructured tasks: SMACH will fall short as the scheduling of your task becomes less structured.  

- Low-level systems: SMACH is not meant to be used as a state machine for *low-level* systems that require high efficiency, SMACH is a *task-level* architecture.  

- Smash: Do not use SMACH when you want to smash something, for that use [smash](https://en.wikipedia.org/wiki/Hulk_and_the_Agents_of_S.M.A.S.H.). 

### 1.3 What can SMACH do for you? <a name="1.3"></a>

[go to top](#top)

Convert state-less systems into state-based ones (depends on whether that is what you are looking for).

![UntitledDiagram2](assets/UntitledDiagram2.jpg)

## 2 Simple State Machine Example <a name="2"></a>

[go to top](#top)

In this section, we will create a very simple state machine example.

### 2.1 Creating States <a name="2.1"></a>

[go to top](#top)

To begin, the idea of a state machine involves having a few states. To create a state class, it needs to have an `__init__` method as well as an `execute` method.

```python
class ExampleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['done'])
    def execute(self, ud):
        return 'done'
```

In `ExampleState`, we define it as a SMACH state that has only one outcome - `done`. If it is executed, it will return the outcome `done`. This means that this simple state will only return the one output `done` if it were to be executed.

### 2.2 Creating the SMACH State Machine <a name="2.2"></a>

[go to top](#top)

Next the SMACH state machine has to be created. Within `outcomes` we will define the possible end states for the state machine (in this case it will be `succeeded` and `aborted`). As we know that our example states are simple enough to be fool-proof, we do not expect them to actually fail. In general `aborted` is a state to allow the an execution to be terminated (as we shall see in later examples).

```python

# Create a SMACH state machine
sm = smach.StateMachine(outcomes=['succeeded','aborted'])
```

### 2.3 Adding States to the State Machine <a name="2.3"></a>

[go to top](#top)

Now that we have our state class as well as the state machine contained. we have to define our actual states. 

```python
with sm:
        # Add states to the container
        smach.StateMachine.add('State1', ExampleState(), {'done':'State2'})
        smach.StateMachine.add('State2', ExampleState(), {'done':'State3'})
        smach.StateMachine.add('State3', ExampleState(), {'done':'succeeded'})
```

In each line, we add a new state, define the state name and what state class we are using (in this case `ExampleState()`) and then we assign the outcome with a new state or in the case of the last state, an outcome of the state machine.
Without going further, you should be able to tell what would happen if we were to execute this state machine. The following is a simple diagram:

![UntitledDiagram](assets/UntitledDiagram.jpg)

### 2.4 Full Example <a name="2.4"></a>

[go to top](#top)

Here is the full example for a brief look on how to fit it all together. In subsequent sections, only relevant portions of the code will be discussed - please refer to the example script itself for the full version/

```python
import smach

class ExampleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['done'])
    def execute(self, ud):
        return 'done'

def main():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('State1', ExampleState(), {'done':'State2'})
        smach.StateMachine.add('State2', ExampleState(), {'done':'State3'})
        smach.StateMachine.add('State3', ExampleState(), {'done':'succeeded'})

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
```

### 2.5 Running the example <a name="2.5"></a>

[go to top](#top)

Because SMACH in itself is a ROS-independent library, the script can actually be run outside a ROS workspace. The script in this example is `state_machine.py`.

## 3 SMACH with Rospy <a name="3"></a>

[go to top](#top)

However, given that we intend to use SMACH with ROS, it would be good to look at an example where SMACH is used with a ROS Node.

Start by importing both `smach` and `rospy`:

```python
import rospy
import smach
```

We will now define 2 state classes in this SMACH example. Additionally, we will be using `rospy.loginfo` to allow us to get information on what our state machine is doing later.

### 3.1 Defining 2 states <a name="3.1"></a>

[go to top](#top)

State `Faser`:

```python
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
```

In state `Faser`, the 2 possible outcomes are `outcome1` and `outcome2`. A `counter` attribute is also initialized at `0` - this would provide us with the ability to create simple logic to swap between the 2 outcomes. 

When `execute` is called, `Faser` will return either `outcome1` or `outcome2` depending on the internal `counter`.

State `Maler`:

```python
class Maler(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Maler')
        return 'outcome2'
```

`Maler` is a much simpler state akin to the one in out simple state machine example. It only has one outcome and will return that outcome when executed. 

### 3.2 Creating the State Machine <a name="3.2"></a>

[go to top](#top)

Now when creating the state machine within `main`, we start off by definition a ROS node called `smach_example_state_machine`:

```python
rospy.init_node('smach_example_state_machine')
```

Then we define the states:

```python
# Create a SMACH state machine
sm = smach.StateMachine(outcomes=['EXIT', 'outcome5'])

# Open the container
with sm:
    # Add states to the container
    smach.StateMachine.add('Faser', Faser(), 
                           transitions={'outcome1':'Maler', 
                                        'outcome2':'EXIT'})
    smach.StateMachine.add('Maler', Maler(), 
                           transitions={'outcome2':'Faser'})

```

Can you guess how this state machine would play out when executed? Here's a simple diagram illustrating the various states:

![simplestate](assets/simplestate.jpg)

### 3.3 Running the Example <a name="3.3"></a>

[go to top](#top)

Now that this is a ROS Node, the example script has to be compiled within a workspace. 

1. Navigate to the example scripts directory (wherever it is saved)

2. Catkin make the packages

   ```shell
   $ catkin_make
   ```

3. Source the `setup.bash` file. Do this in 2 terminals as you will need to start `roscore` as well.

   ```shell
   $ source ./devel/setup.bash
   ```

4. In the other terminal, start `roscore`

   ```shell
   $ roscore
   ```

5. In the main terminal, call the script by bringing it up using `rosrun`:

   ```shell
   $ rosrun smach_tutorials state_machine_simple.py
   ```

### 3.4 Viewing Rospy Info Log <a name="3.4"></a>

[go to top](#top)

Remember the lines with `rospy.loginfo`? It is possible to view them in another terminal by running:

```shell
$ rostopic echo rosout
```

Logging information is useful when debugging especially when the state machines grow too large and begin encompassing various action clients.

## 4 State Machines and Data Transfer <a name="4"></a>

[go to top](#top)

In many cases, the states will not be expected to work in isolation. Instead, some data (be it sensor data, user input, etc) will be passed to the states and that would cause the states to change in some way or another. Whatever this data is, it is referred to as the *userdata* of a state.

### 4.1 Definition of States with Userdata <a name="4.1"></a>

[go to top](#top)

When defining input and output data, the keys are defined within the `__init__` method of the class. These exact key names can be used as-is within the class. Multiple keys can also be defined if there are multiple sources of data.

State `Faser`:

Once again `Faser` has a counter that checks for what outcome to return. However in this case, the `faser_counter_in` variable is a key that obtains userdata. We will see how we can remap a key from the State Machine into `faser_counter_in` such that it could be used within this state.

```python
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
```

State `Maler`:

Nothing much is notable about `Maler` except that we now take an input key `maler_counter_in` and pass it into `rospy.loginfo`.

```python
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
```

### 4.2 Definition of State Machine with UserData <a name="4.2"></a>

[go to top](#top)

Now the state machine is defined with a new variable involved - `sm_counter`. It functions similarly to the `counter` variable within state `Faser` in the previous example.

```python
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['EXIT'])
    sm.userdata.sm_counter = 0
```

### 4.3 Remapping <a name="4.3"></a>

[go to top](#top)

In order for the input and output keys to be part of the userdata of the entire state machine, there is a need to *remap* the names of the keys to the specific userdata (in this case `sm_counter`). This is done with the use of the `remapping` argument when adding states to a state machine.

The syntax for remapping is as follows:

```python
# Syntax
remapping={x,y}
# Example
remapping={'local_key':'sm_userdata'}
```

Where `x` is a input/output key of a state and `y` is always the userdata to be mapped.

It is always recommended to map userdata even if the input/output keys match with the userdata to be mapped. 

The following is a diagram of how remapping looks like in the context of our state machine (Dotted lines indicate remaps):

![SM-1564061452245](assets/SM-1564061452245.jpg)

Thus the states can be added with this new knowledge of remapping:

```python
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
```

Notice that we remapped both `faser_counter_in` and `faser_counter_out` to `sm_counter`. If we were to omit `faser_counter_out` then the `sm_counter` will not be incremented when `faser_counter_out` is incremented. 

### 4.4 Running the Example <a name="4.4"></a>

[go to top](#top)

Proceed to run the example as usual. The full script can be found under the `smach_tutorials` as `user_data.py`.

## 5 State Machine in a State Machine <a name="5"></a>

[go to top](#top)

We now move into adding a state machine into another state machine - statemachineception. Now this is actually more useful than just being a novelty. For instance you might create a state machine for a certain image processing module to process an image, then use that output from that state machine as part of a state in a larger state machine for an entire system.

If this sounds confusing to you don't worry - we'll go through it slowly. The key thing to take away from this is that adding a state machine to a state machine is the same as adding a state to a state machine. The larger state machine doesn't actually care about the smaller state machine - it simply treats it as another state.

### 5.1 Creating the States <a name="5.1"></a>

[go to top](#top)

The states are quite similar to the ones that were covered in the previous section. Except that now state `Faser` will be split into 2 states - `Fas` and `Ser` and will be used as part of the sub state machine.

States `Fas` and `Ser`:

Similar to previously, states `Fas` and `Ser` will execute in alternating order until the internal counter hits 3, whereby the state machine will call the exit condition. Except instead as we will see later, the exit condition has been modified to only exit the sub state machine.

```python
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
```

State `Maler`:

State `Maler` is a simple non-interactive state.

```python
# define state Maler
class Maler(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Maler')
        return 'outcome3'
```

### 5.2 Creating the nested State Machine <a name="5.2"></a>

[go to top](#top)

Now the nested state machine can be created. Pay attention to the outcomes as there are now 2 exit flags for the 2 states. `EXIT_SUB` refers to the exiting of the `SUB` state machine while `EXIT_TOP` refers to the exiting of the main state machine.

```python
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
```

Notice that after defining the `sub` state machine, adding it to the main state machine was relatively simple.

If this definition order grinds your gears, it is also possible to define the sub state machine first, outside the `sm_top` definition, before adding it to the `top` state machine later.

### 5.3 Running the Example <a name="5.3"></a>

[go to top](#top)

Proceed to run the example script. The script name is `state_machine_nesting.py`.

## 6 Calling ROS Actions from State Machine <a name="6"></a>

[go to top](#top)

Here's the actual fun part - being able to use ROS actions from SMACH. This allows for modularized task allocation and generally makes things easier for everyone involved.

### 6.1 Creating the Test Action Server <a name="6.1"></a>

[go to top](#top)

In this example, the action server will be a very simple TestAction. The action file can be found within the `action` folder as `Test.action` and it is evident that this is a very simple action:

```python
float64 goal
---
---
```

Same goes for the action server defined within the example script itself:

```python
# Create a trivial action server
class TestServer:
    def __init__(self,name):
        self._sas = SimpleActionServer(name,
                TestAction,
                execute_cb=self.execute_cb)

    def execute_cb(self, msg):
        if msg.goal == 0:
            self._sas.set_succeeded()
        elif msg.goal == 1:
            self._sas.set_aborted()
        elif msg.goal == 2:
            self._sas.set_preempted()
            
            
def main():
    rospy.init_node('smach_example_actionlib')

    # Start an action server
    server = TestServer('test_action')
```

### 6.2 SMACH SimpleActionState <a name="6.2"></a>

[go to top](#top)

`smach_ros` has support for calling actions, this state class can act as a proxy to an actionlib action. 

```python
from smach_ros import SimpleActionState
```

There are a few ways to initialize the `SimpleActionState`. The adding the state to the state machine would require the topic name, action type, and some policy for generating a goal. The possible outcomes of the simple action state are 'succeeded', 'preempted' and 'aborted'.  

**Note that the following examples are not part of the example script - they are used as examples only**

#### 6.2.1 Basic Initialization (No Goals) <a name="6.2.1"></a>

[go to top](#top)

The basic initialization simply calls the action and does not bother to generate a goal

```python
# In this example, we attempt to call the gripper to do some predefined action
# There is no goal in this case
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    smach.StateMachine.add('TRIGGER_GRIPPER',
                           SimpleActionState('action_server_namespace',
                                             GripperAction),
                           transitions={'succeeded':'next_state'}) # transition to the next state on success (in this case it will always transition as there is no goal)
```

Notice that where once the state class would be called, `SimpleActionState` was called in it's stead.

#### 6.2.2 Initialization with Fixed Goals <a name="6.2.2"></a>

[go to top](#top)

In the case where we would like the action to continue until a specific goal is reached, the goal can be set. In the following example, a predefined class called `Pr2GripperCommandGoal` was called for the express purpose of setting a goal for the gripper action class to reach. If you require a refresher on goals, refer to the [SimpleActionServer Goal Policies Documentation](http://wiki.ros.org/actionlib#SimpleActionServer_Goal_Policies).

```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    gripper_goal = Pr2GripperCommandGoal() # calls the gripper command goal object
    gripper_goal.command.position = 0.07 
    gripper_goal.command.max_effort = 99999
    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        goal=gripper_goal),
                      transitions={'succeeded':'next_state'}) 
```

#### 6.2.3 Initialization with User Goals <a name="6.2.3"></a>

[go to top](#top)

In most cases, the goals will be define more dynamically - perhaps through another ROS Node or even direct user intervention. In those cases, simply use *userdata* input keys for setting the goals and remap them accordingly.

```python
sm = StateMachine(['succeeded','aborted','preempted'])
# In this example the userdata was set static
# But you can opt to set it dynamically in an actual use case
sm.userdata.user_data_max = 99999
sm.userdata.user_data_position = 0.07
with sm:
    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        goal_slots=['max_effort', # Goals take userdata
                                                    'position']),
                      transitions={'succeeded':'next_state'},
                      remapping={'max_effort':'user_data_max',
                                 'position':'user_data_position'})
```

#### 6.2.4 Goal Callback <a name="6.2.4"></a>

[go to top](#top)

Setting a goal callback function means that you will receive a callback if the action requires a goal and the goal message can be set on demand.

```python
sm = StateMachine(['succeeded','aborted','preempted'])
# In this example the userdata was set static
# But you can opt to set it dynamically in an actual use case
sm.userdata.userdata_input = 99999

with sm:
    def gripper_goal_cb(userdata, goal): # Defines the goal callback
       gripper_goal = GripperGoal()
       gripper_goal.position.x = 2.0
       gripper_goal.max_effort = userdata.gripper_input # Get the max effort from the userdata
       return gripper_goal

    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        goal_cb=gripper_goal_cb,
                                        input_keys=['gripper_input'])
                      transitions={'succeeded':'next_state'},
                      remapping={'gripper_input':'userdata_input'})
```

#### 6.2.5 Result Message <a name="6.2.5"></a>

[go to top](#top)

In the case of a certain action servers such as sensors, a return is desired from the action server. These returns can be written into userdata to be passed onto other states and action servers!

```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        result_slots=['max_effort', 
                                                      'position']),
                      transitions={'succeeded':'next_state'},
                      remapping={'max_effort':'user_data_max',
                                 'position':'user_data_position'})
```

#### 6.2.6 Result Callback <a name="6.2.6"></a>

[go to top](#top)

Using a result callback allows you to evaluate the result returned by the action server and define your own outcome (other than the default 3):

```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    def gripper_result_cb(userdata, status, result):
       if status == GoalStatus.SUCCEEDED:
          userdata.gripper_output = result.num_iterations
          return 'custom_outcome1'
       else:
          return 'custom_outcome2'

    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        result_cb=gripper_result_cb,
                                        output_keys=['gripper_output'])
                      transitions={'succeeded':'next_state'},
                      remapping={'gripper_output':'userdata_output'})
```

### 6.3 Simple Action State definition in Example <a name="6.3"></a>

[go to top](#top)

The following is the simple action state definition within the example:

```python
def main():
    rospy.init_node('smach_example_actionlib')

    # Start an action server
    server = TestServer('test_action')

    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Open the container
    with sm0:
        # Add states to the container

        # Add a simple action state. This will use an empty, default goal
        # As seen in TestServer above, an empty goal will always return with
        # GoalStatus.SUCCEEDED, causing this simple action state to return
        # the outcome 'succeeded'
        smach.StateMachine.add('GOAL_DEFAULT',
                               smach_ros.SimpleActionState('test_action', TestAction),
                               {'succeeded':'GOAL_STATIC'})

        # Add another simple action state. This will give a goal
        # that should abort the action state when it is received, so we
        # map 'aborted' for this state onto 'succeeded' for the state machine.
        smach.StateMachine.add('GOAL_STATIC',
                               smach_ros.SimpleActionState('test_action', TestAction,
                                                       goal = TestGoal(goal=1)),
                               {'aborted':'GOAL_CB'})
        
        def goal_callback(userdata, default_goal):
            goal = TestGoal()
            goal.goal = 2
            return goal

        smach.StateMachine.add('GOAL_CB',
                               smach_ros.SimpleActionState('test_action', TestAction,
                                                       goal_cb = goal_callback),
                               {'aborted':'succeeded'})
```

There might be quite a bit to unpack in this example, so go through it slowly. Notice that `goal_callback` has to take in `userdata` and `default_goal` as arguments, but the actual function in this case does not use either, instead it creates it's own `TestGoal()` object and assigns a goal of `2`.

Also, notice that we could map the `aborted` state from `GOAL_CB` state to `succeeded` state of the state machine itself. Effectively, `GOAL_CB` is a callback state.

Feel free to run the example script `actionlib_test.py` to test the code out.

## 7 Visualization of SMACH State Machines <a name="7"></a>

[go to top](#top)

The use of state machines means that `rqt_graph` might not be the most optimal in trying to figure out what's wrong with your code and whether specific action servers or clients have been broken. Instead, it's better to use the `smach_viewer`. 

### 7.1 Preparations <a name="7.1"></a>

[go to top](#top)

Append the following lines of code below the state machine definition to start an introspection server.

```python
# Create and start the introspection server
sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()

# Execute the state machine
outcome = sm.execute()

# Wait for ctrl-c to stop the application
rospy.spin()
sis.stop()
```

The `server_name` can be set to anything desired and will function as the namespace for the ROS introspection topics. `SM_ROOT` is the name of the state machine within the smach viewer. If there are 2 state machines and one is a sub state machine of the other, naming the state machine `SM_TOP/SM_SUB` will get smach viewer to recognize that there are nested state machines.

### 7.2 Installing SMACH Viewer <a name="7.2"></a>

[go to top](#top)

The repository of examples prepared already has the necessary dependencies for `smach_viewer` to work. However, on it's own, `smach_viewer` has actually been discontinued since ROS Hydro. So here is how to get it to work in your own repositories (tested on Kinetic) if you required SMACH viewer.

1. Within your `/catkin_ws/src` folder, git clone [xdot](https://github.com/matt3o/xdot), [executive_smach](https://github.com/jbohren/executive_smach) and [executive smach visualization](https://github.com/matt3o/executive_smach_visualization). Alternatively, copy the folders over from the smach examples workspace.
2. `catkin_make` the workspace and source the `devel.bash` file.
3. Attempt to run smach viewer using `rosrun rqt_smach rqt_smach`

### 7.3 Running SMACH Viewer <a name="7.3"></a>

[go to top](#top)

Once everything is set-up, run SMACH viewer by opening a new bash terminal and calling it using `rosrun`. If you are using the example repository on Kinetic, run `rqt_smach` instead.

```shell
$ rosrun rqt_smach rqt_smach # For Kinetic

$ rosrun smach_viewer smach_viewer.py # For Hydro
```

### 7.4 Example <a name="7.4"></a>

[go to top](#top)

Run the example script `state_machine_simple_introspection.py`.  Then launch `smach_viewer` in another terminal. You should see the following screen:

![1564069347372](assets/1564069347372.png)

## 8 Concurrence <a name="8"></a>

[go to top](#top)

SMACH allows for various child states to affect the final state of a sub state machine. That sounds like a mouthful I know. Here's a diagram that can illustrate what I mean:

![Copy of remap_sm](assets/Copy of remap_sm.jpg)

The yellow diamond is the outcome map - in essence it's an "AND" gate.

### 8.1 Concurrence Syntax <a name="8.1"></a>

[go to top](#top)

I won't show the state definitions as they are pretty simple. The important part is the state machine definitions:

```python
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
```

As mentioned previously, the yellow diamond functions as the `outcome_map`. In this case, it checks if the outcome of state `Fas` and `Ser` are `outcome2` and `outcome1` respectively. If they are, then the `outcome_map` determines `sm_con`'s state to be `outcome5`. Else it's default state remains as `outcome4`.

### 8.2 Running the Example <a name="8.2"></a>

[go to top](#top)

The example script can be found as `concurrence.py`.

Also, if you were wondering, yes you can chain concurrences.

## 9 Sequences <a name="9"></a>

[go to top](#top)

Remember the very first state machine example at the top of this document? Well what if you'd like to run all those 3 states in sequential order very quickly without very much fuss or checking the state outcomes and trying to parse them into the next state?

That's where sequences can be used - sequences assume that you'd like to transition from one state to the next in the sequence as long as the outcome of each state matches the one provided in the sequence constructor.

### 9.1 Defining the ExampleState Class <a name="9.1"></a>

[go to top](#top)

The example state class will be defined to output 'done' as the outcome. All states within this example will use this same class, thus the `connector_outcome` defined in the sequence will be the same.

```python
class ExampleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['done'])
    def execute(self, ud):
        return 'done'
```

### 9.2 Defining the State Machine and the Sequence <a name="9.2"></a>

[go to top](#top)

Sequences require a slightly different definition in the state machine. Thus in most cases they will be defined within a sub state machine. In this example however, the only state machine will be the sequence state machine. 

```python
# Create a SMACH state machine
sq = smach.Sequence(
    outcomes = ['succeeded'],
    connector_outcome = 'done')

# Open the container
with sq:
    # Add states to the container
    smach.Sequence.add('Fas', ExampleState())
    smach.Sequence.add('Ser', ExampleState())
    smach.Sequence.add('Maler', ExampleState(), {'done':'succeeded'})
```

In this case `connector_outcome` refers to the `outcome` needed from each state in the sequence before the sequence advances to the next state. Note the final state in the sequence has it's `outcome` remapped from 'done' to 'succeeded' to match that of the overall state machine.

### 9.3 Running the Example <a name="9.3"></a>

[go to top](#top)

The example can be found as `sequence.py`.

## 10 Iterators <a name="10"></a>

[go to top](#top)

SMACH allows a state machine to be defined as an iterator. What an iterator does is that it keeps iterating through the states until a set of success conditions are met (somewhat like a while loop). In this example, we will make an iterator that can sort numbers into odd and even.

### 10.1 Imports <a name="10.1"></a>

[go to top](#top)

```python
import roslib; roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')
import rospy

import smach
from smach import Iterator, StateMachine, CBState
from smach_ros import ConditionState, IntrospectionServer
```

### 10.2 Constructing the State Machine <a name="10.2"></a>

[go to top](#top)

In this example, we will create a function to construct the state machine. At the start, 3 sets of userdata are created. `nums` refers to the list of numbers to be sorted. `even_nums` and `odd_nums` are empty lists for holding even and odd numbers respectively.

More interesting is the second part, where the iterator is constructed. 

```python
def construct_sm():
    sm = StateMachine(outcomes = ['succeeded','aborted','preempted'])
    sm.userdata.nums = range(25, 88, 3)
    sm.userdata.even_nums = []
    sm.userdata.odd_nums = []
    with sm:
## %Tag(ITERATOR)%
        tutorial_it = Iterator(outcomes = ['succeeded','preempted','aborted'],
                               input_keys = ['nums', 'even_nums', 'odd_nums'],
                               it = lambda: range(0, len(sm.userdata.nums)),
                               output_keys = ['even_nums', 'odd_nums'],
                               it_label = 'index',
                               exhausted_outcome = 'succeeded')
```

- `outcomes`: Possible outcomes of the iterator stat machine
- `input_keys`: Input keys for userdata
- `it`: The data to be iterated upon. In this case a lambda function is used to get the list of data
- `output_keys`: Output keys for userdata
- `it_label`: Key that holds the current item in the list.
- `exhausted_outcome`: Preferred outcome when the iterator finishes iterating through the list

### 10.3 Creating the Iteration Container <a name="10.3"></a>

[go to top](#top)

Next the iteration container is created. This is done by creating a container state machine that now has an extra state: `continue`. The `continue` state is the outcome of one iteration of the iteration container and whether to continue iterating.

`ConditionState` is specially imported from the `smach_ros` library. It allows for a state to be created based on a condition of `true` or `false`. In this case, the condition callback function or `cond_cb` is simply a lambda function that checks if the number is divisible by 2.

The `even_cb` and `odd_cb` functions serve to append the respective even or odd numbers to the userdata. Their respective state machines then map the returned 'succeeded' outcome to the overall 'continue' outcome of the container state machine.

```python
with tutorial_it:
    container_sm = StateMachine(outcomes = ['succeeded','preempted','aborted','continue'],
                                input_keys = ['nums', 'index', 'even_nums', 'odd_nums'],
                                output_keys = ['even_nums', 'odd_nums'])
    with container_sm:
        #test wether even or odd
        StateMachine.add('EVEN_OR_ODD',
                         ConditionState(cond_cb = lambda ud:ud.nums[ud.index]%2, 
                                        input_keys=['nums', 'index']),
                         {'true':'ODD',
                          'false':'EVEN' })
        #add even state
        @smach.cb_interface(input_keys=['nums', 'index', 'even_nums'],
                            output_keys=['odd_nums'], 
                            outcomes=['succeeded'])
        def even_cb(ud):
            ud.even_nums.append(ud.nums[ud.index])
            return 'succeeded'
        StateMachine.add('EVEN', CBState(even_cb), 
                         {'succeeded':'continue'})
        #add odd state
        @smach.cb_interface(input_keys=['nums', 'index', 'odd_nums'], 
                            output_keys=['odd_nums'], 
                            outcomes=['succeeded'])
        def odd_cb(ud):
            ud.odd_nums.append(ud.nums[ud.index])
            return 'succeeded'
        StateMachine.add('ODD', CBState(odd_cb), 
                         {'succeeded':'continue'})
```

### 10.4 Running the Example Code <a name="10.4"></a>

[go to top](#top)

The example code can be found as `iterator_tutorial.py`. It is recommended that SMACH viewer be used to see how the state machine actually functions.