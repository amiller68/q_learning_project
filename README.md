# q_learning_project

## Team Members
Alex Miller and Hamlet Fernandez

## Flex Hours
Alex: 2:06
Hamlet: 2:05

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
[ ] Finish Q-Learning algorithm and have a converged Matrix by Sunday
[ ] Finish module for locating objects using LiDAR and Camera data by Wednesday

Next Week
[ ] Finish module for Robot movement by Friday
[ ] Finish model Arm manipulation by sunday
[ ] Test, Tweak, and prepare deliverables until Tuesday