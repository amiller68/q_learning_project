# q_learning_project

## Team Members
Alex Miller and Hamlet Fernandez

## Flex Hours

### Implementation Plan
Alex: 2:06
<br>
Hamlet: 2:05

### Final Deliverables:
Alex: 0:42
<br>
Hamlet: 0:42

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
<br>

_Selecting and executing actions for the robot (or phantom robot) to take_:
<br>

We implemented this component in the `get_next_action()` method of the `QLearning` class:
 - Get all possible actions from the current state using `action_matrix`
 - Select a random action id from the list of available actions
 - return the random id

_Updating the Q-matrix_:
<br>

We implemented this component in the `update_q_matrix(reward)` method of the `QLearning` class:
 - On receiving a `reward` value of the `q_learning/reward` topic:
   - determine the next state based on the current state and the last action taken.
   - Calculate the maximum reward of the next state
   - Use this value in order to update the q matrix for the current state and last action taken
   - If we've taken a multiple of 3 steps, then reset the state to 0
   
_Determining when to stop iterating through the Q-learning algorithm_:
<br>

`update_q_matrix(reward)` returns True once it observes a total of `seq_convergence_target` updates with a value change less than `convergence_bound`.
Once it returns True, the `QLearning` instance stops publishing actions and saves the q-matrix to a csv.

_Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot_:
<br>

We implemented this component in the `get_next_action` method of the `RobotAction` class.
 - Given the current state, get the index of the action with the highest reward from the q-matrix
 - Publish this action and wait for the next reward from the environment
Once we've published three actions, our `RobotAction` class stops publishing actions and exits.

**Robot perception description:** 
<br>

In our `robot_controller` file, we implemented a class called `RobotPerception` that handled deciding where a specified target was located. 
A class or program with access to a `RobotPerception` instance can set a target using the member function `set_target`.
If the target specifies an object then when the instance receives an image via its `image_callback` method then it calls `locate_object`; if it specifies an AR tag it calls `locate_tag`.
Both of these methods scan the image for the specified target, and, if they can locate it, set a class variable called `target_err`, which describes how far off the observed target is from the center of the image.
Otherwise, if no appropriate target is specified, or the specified target can't be found, then `target_err` is set to `None`.
Concurrently, our `scan_callback` continually updates the distance observed directly in front of the robot and binds this value to the class variable `target_dist`.
A program can access the class instance's guess of where the specified target is by calling the class method `get_target`, which returns a tuple, `target_err, target_dist` if `target_err` is not `None`.

_Identifying the locations and identities of each of the colored objects_:
- Once we implemented our image callback, we implemented and tested `locate_object`. If this method is called then the `RobotPerception` instance scans an image for the color range of the specified object, and then locates the center of the pixels in the image that fall within that range.
- If an object is observed then `locate_object` binds how far off that object is from the center of the image by using the estimated center of its pixels based on its color range.
- We researched an initial estimate of what ranges were appropriate for each color. 
- We then tested our implementation on actual objects using the method `test_color_perception` which only exits if the robot is able to observe all three colors in a row. We used this method to debug and fix our ranges, and see if any of our specified ranges caused objects to be misidentified.

_Identifying the locations and identities of each of the AR tags_:
- Once we implemented our image callback, we implemented and tested `locate_tag`. If this method is called then the `RobotPerception` instance scans a grayscale image for the specified AR tag, specified by its id, using the ARuco library. If the method locates a tag it calculates the center of the tag in the image by averaging the coordinates of its four corners.
- If a tag is observed then `locate_tag` binds how far off the center of that tag is from the center of the image by using the estimated center of its pixels based on its detected corners.
- We researched our implementation by using the AR detection example in the project description. We were able to identify which tag_id should be associated with which id returned by the ARuco `detectMarkers` method by holding up the different tags in front of the robot and reading debugging print statements.
- We then tested our implementation on tags using the method `test_tag_perception` which only exits if the robot is able to observe all tags colors in a row.

**Robot manipulation and movement:**

In our `robot_controller` file, we implemented a class called `RobotManipulator` that handled publishing movement commands the robot and its arm.
A calling program or class can get the robot to move towards an object or tag using in instance of this class by:
- Retrieving the estimated location of a `target` from an instance of `RobotPerception`
- calling the `follow_target(target)` of a `RobotManpipulator` instance, which tells the robot either to:
  - Spin around if target is `None`, which indicates that the perception class instance wasn't able to see the specified target
  - Or turn at the rate specified `target_err` and move forward.
  - if `target_dist` is greater than some specified distance the method returns `True` to indicate that we are still following the target. Otherwise, the method returns `False` to indicate that we are done following the target.
  - Our implementation assumes that if the robot is able to see a target, and measures our set goal distance, that the robot then must be in front of the specified target.
Once the robot has arrived at a target it stops and then proceeds to either pickup a target object, or place an object in front of a target tag. 
These actions are implemented in the `RobotManipulator` class methods `pickup_object` and `put_down_object`, which tell the robot's arm to 
execute defined pickup and put down movements, respectively. We came to these arm positions by testing fiddling with the arm GUI, and testing the transitions on real objects.
  
<br>

_Moving to the right spot in order to pick up a colored object_:
- We implemented a `RobotController` class that receives instructions from a node which publishes the optimal set of actions to take implemented in `robot_action.py`.
- `RobotController` uses these instructions in order to set targets in a perception class instance, and uses the estimated targets to get the robot to move through a manipulator class instance.
- For every object the robot is told to go to, it runs `follow_target` within a while loop until it returns `False`, at which point it signals the robot to stop.
- We tested our implementation by seeing if the robot could accurately carry out the instructions from our action publishing node without actually picking up objects.
- Once we could get the robot to reliably complete an optimal move sequence between objects and AR tags, we knew our implementation was correct.

_Picking up the colored object_:
- Before the robot controller carries out a received instruction, it sets the arm to a `start` position that positions the robot arm such that the gripper is out in front of it, ready to be aligned with a target object
- Once `follow_target` returns false and the controller stops the robot, we know the arm gripper is around the specified object.
- The controller then calls `pickup_object` which closes the grippers and moves the arm into a `lift` position, picking up the object.
- We tested our implementation by seeing if our robot could reliably pick up objects. It made some mistakes; if the arm isn't aligned properly or the robot moves too far, the robot will just knock over the object instead. 

_Moving to the desired destination (AR tag) with the colored object:_
- Similar to what we said above:
- We implemented a `RobotController` class that receives instructions from a node which publishes the optimal set of actions to take implemented in `robot_action.py`.
- `RobotController` uses these instructions in order to set targets in a perception class instance, and uses the estimated targets to get the robot to move through a manipulator class instance.
- For every tag the robot is told to go to, it runs `follow_target` within a while loop until it returns `False`, at which point it signals the robot to stop.
- We tested our implementation by seeing if the robot could accurately carry out the instructions from our action publishing node without actually picking up objects.
- We had to set a larger goal distance for the AR tags as opposed to the objects, due to the camera not being able to pick up the AR tags if the robot was too close.
- Once we could get the robot to reliably complete an optimal move sequence between objects and AR tags, we knew our implementation was correct.

_Putting the colored object back down at the desired destination_:
- Once the robot had picked up an object, it proceeds to the specified tag.
- Once `follow_target` returns `False`, the controller stops the robot and then puts the arm back in the `start` position before opening the grippers.
- At this point the object the robot previously picked up should be back on the ground.
- We tested our implementation by seeing if our robot could reliably put down objects at the correct AR tags. 

**Challenges:**
Implementing this project went pretty smoothly; the code base was easy to break up into distinct classes, and the testing process in Gazebo went quickly.
That being said, it was difficult getting the robot to work as reliably in the real world, with both network lag and lighting proving to be big obstacles to overcome.
We were able to overcome lighting-effects on perception by writing test scripts that helped us configure our color range parameters.
Moreover, we were able to get successful test runs with our implementation once we had configured our colors correctly.
One issue we were not able to fix was that whenever we tried to move the arm back into a position it had already visited, we got an error that crashed our program. 
Therefore, our implementation does not, strictly speaking, work since it can't go through the motion of picking up and putting down an object more than once; we did confirm that our implementation could in fact pick up and put down each object one at a time.
Additionally, when we went to record our robot moving one object correctly, and then moving from object to tag without picking up objects, we experienced a large amount of lag that we didn't get before, keeping us from getting a successful run and recording.

**Future work:**: 
If we had more time we would fix the issues with our implementation; we would fix the error we get when trying to move the arm back to a previous position and try and get a recording of a successful run.
Moreover, though we confirmed our method of picking up object does work, it would be nice if the robot, in the case it isn't correctly aligned with a target object, could use our perception library in order to calculate a good target for the arm's position.
This would make our implementation more resilient to errors and lag by allowing it to correct movement mistakes.

**Takeaways**:
- Breaking code up into objects with distinct responsibilities makes development clear and debugging easier. 
  - Based on how we structured our classes, we were able to look at and make changes to the project independently of one another
  - Because our classes had set responsibilities and well-defined behavior, it was easy to debug our project since distinct functionality was contained within a given class.
- Pair programming is very useful for both developing and debugging
  - When we did work together it was very helpful to pair program so that we could come up with solutions and catch bugs quicker.
  - It also made testing alot easier because we were able to talk through the behavior we observed from our robot and come to reasoned decisions on how to fix bugs together.

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