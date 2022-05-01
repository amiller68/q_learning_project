# q_learning_project

## Team Members
Alex Miller and Hamlet Fernandez

## Flex Hours
Alex: 2:06
Hamlet: 2:05

## Writeup

**Objectives description:** 
There are two components to this project. 
The first has to do with computing a decision-making matrix to guide a robot's actions towards an optimal outcome. 
The second is an interface that can translate these decisions to the robot's environment in order to maximize some optimal real world outcome.
The goal is to design a system that is able to use these components in order to make real world decisions about its world.

**High-level description:** 
We simulated an actor taking repeated random actions and receiving reward signals from its environment. 
For each iteration, we used the actors current state, action, and next state in order to update its expected reward at its current state within a matrix.
After many iterations of this, values within the matrix converged on their expected values, which could be used to determine optimal action sequences.
These sequences represent optimal solutions to the problem of moving objects in front of AR tags; if you know them you know which colored object belong in front of each AR tag.

**Q-learning algorithm description:** 
Describe how you accomplished each of the following components of the Q-learning algorithm in 1-3 sentences, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):
_Selecting and executing actions for the robot (or phantom robot) to take_
We implemented this component in the `get_next_action()` method of the `QLearning` class:
 - Get all possible actions from the current state using `action_matrix`
 - Select a random action id from the list of available actions
 - return the random id

_Updating the Q-matrix_
We implemented this component in the `update_q_matrix(reward)` method of the `QLearning` class:
 - On receiving a `reward` value of the `q_learning/reward` topic:
   - determine the next state based on the current state and the last action taken.
   - Calculate the maximum reward of the next state
   - Use this value in order to update the q matrix for the current state and last action taken
   - If we've taken a multiple of 3 steps, then reset the state to 0
   
_Determining when to stop iterating through the Q-learning algorithm_
`update_q_matrix(reward)` returns True once it observes a total of `seq_convergence_target` updates with a value change less than `convergence_bound`.
Once it returns True, the `QLearning` instance stops publishing actions and saves the q-matrix to a csv.

_Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot_
We implemented this component in the `get_next_action` method of the `RobotAction` class.
 - Given the current state, get the index of the action with the highest reward from the q-matrix
 - Publish this action and wait for the next reward from the environment
Once we've published three actions, our `RobotAction` class stops publishing actions and exits.

## Implementation Plan
- Q-learning algorithm
  - Executing the Q-learning algorithm
    - We initialize an empty 64 x 9 Q-learning matrix `Q`
    - After loading the actions-matrix, we can randomly select an action `a` to take from our starting state `s`, and see what reward it yields.
    - We use the reward to calculate `Q[s][a]` and then transition into the next state `s+1`. We repeat this until we reset, at which point we start the process over.
    - We repeat the last step until our matrix converges.
    - Testing:
      - We will test our process for iterating through actions by running the phantom movement node, and see if the actions we dispatch correspond with the actions the phantom robot carries out.
      - Once we're sure our method is correct, we can run it on the virtual reset world and if we don't get errors that's good evidence our implementation is correct.
  - Determining when the Q-matrix has converged
    - We can determine if the Q-matrix has converged if it has gone some significant number `sig` iterations without updating a value, or changing a value by more than some constant `c`.
    - These values will need to be chosen arbitrarily. For now, we'll settle on `sig = 20` and `c = .1`, but we will need to test if that gives us a usable matrix
    - Testing:
      - Whether the Q-Matrix has converged will need to be determined when testing the performance of later parts of the project.
      - If we find that our action finding algorithm is lacking, we will need to update `sig` and `c`, or rethink how we build the Q-matrix
  - Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
    - When we have a converged Q-matrix, for any state the robot is in we can choose the action with the highest expected reward and take that course of action.
    - This should maximize our expected reward.
    - Testing:
      - Whether our algorithm maximizes reward correctly will depend on how well our Q-matrix is converged.
      - If our robot has trouble deciding on the optimal course of action we either need to change our method for updating the Q-matrix or redefine `sig` and `c`
- Robot perception
  - Determining the identities and locations of the three colored objects
    - We can locate object using LiDAR, and identify them by using the robot's camera, comparing their colors to expected ranges which we must define.
    - Testing:
      - We can test this by running the algorithm, printing out what action we want to take, and verifying if the robot is able to identify the object specified by a given action
      - If the robot has trouble finding objects, then something is wrong with our LiDAR implementation
      - If the robot can find objects, but doesn't identify them correctly, then something is wrong with the color ranges we defined for each object.
      - To make this more robust, we can test whether the robot is able to locate object from various starting positions.
  - Determining the identities and locations of the three AR tags
    - We can locate the blocks using LiDAR, and we can identify them by machine reading the numbers on the front.
    - Testing:
      - We can test this by running the algorithm, printing out what action we want to take, and verifying if the robot is able to identify the object specified by a given action, and then move to the correct block
      - If the robot has trouble finding blocks, then something is wrong with our LiDAR implementation
      - If the robot can find blocks, but doesn't identify them correctly, then something is wrong with how we're reading the front of blocks.
      - To make this more robust, we can test whether the robot is able to locate blocks from various starting positions.
      - If the robot is able to move to an object and then to a block, as specified by an action, reliably, then the robot's perception is implemented correctly.
- Robot manipulation & movement
   - Picking up and putting down the colored objects with the OpenMANIPULATOR arm
     - We will need to figure out the height and grip width with which the robot should pick up object, as well as the distance it should be from an object before attempting to pick it up. Once we know that, it should be easy to tell the robot to grab an object.
     - Testing:
       - We will need to arrive at these values through testing, by watching the robot's performance and tuning the values appropriately.
   - Navigating to the appropriate locations to pick up and put down the colored objects 
     - Once the robot is able to perceive objects, we can locate them using odometry, and implement a general method that describes how to drive to a specified location.
     - Testing:
       - We can test this by seeing if the robot is able to zero in on objects correctly.

## Timeline

This Week
[x] Finish Q-Learning algorithm and have a converged Matrix by Sunday
[ ] Finish module for locating objects using LiDAR and Camera data by Wednesday

Next Week
[ ] Finish module for Robot movement by Friday
[ ] Finish model Arm manipulation by sunday
[ ] Test, Tweak, and prepare deliverables until Tuesday