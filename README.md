# mobilerobot_control

This is a ROS program, that simulates the navigation behaviour of an UGV robot in an environment of 6 rooms. The program asks the user to enter 
a request to specify what he wants the robot to do. The user can choose between 5 requests:

1 -> go to a random target position (out of 6 possible target positions) - using move_base -

2 -> enter a specific target position for the robot to go to  - using move_base -

3 -> make the robot start following the walls

4 -> stop where you are

5 -> go to a specified target position using Bug0 algorithm (which is less efficient than move_base)

In the first choice, the main node sends a service request to the tiserver and gets a random target position. Then, it publishes this position to the /move_base/goal and 
then continuously check the status of the goal by subscribing to /move_base/status topic. Once, the status indicates reaching the target. The interface asks the user to enter
another request.

In the second choice, the main node asks the user to choose one out of 6 possible target positions, and publishes it to /move_base/goal as in the previous choice. 

In the third choice, the main node sends a request to the wall_follower service in order for the robot to start following the walls. The user can enter another request anytime
during following the walls. 

In the fourth choice, the main node stops wall following and publishes zero velocity command to /cmd_vel, and the robot stops moving.

In the fifth choice, the main node asks the user to choose one out of 6 possible target positions. Then, it updates the ROS parameters of target x and y positions. After updating the parameters, the bug0 node detects this change and drives the robot towards the target position using go_to_point and wall_follower services. After reaching the
target, bug0 node sends an empty service request to the main node. The main node, then, indicates that the target has been reached, and asks the user to enter another request.

There is a timeout value of 60 seconds. If the bug0 algorithm couldn't reach the target within this time window, the robot stops and the interface asks the user for another request. 



## To run the simulation:

- Clone the repository into your ROS workspace

- Go inside the root of your workspace and run:

  $ catkin_make

  $ roslaunch final_assignment simulation_gmapping.launch
  
  $ roslaunch final_assignment move_base.launch
  
  $ roslaunch final_assignment bug0.launch
  
  $ rosrun final_assignment user_interface_main.py
  


**This program is composed of two packages: my_srv (containing the service files) and final_assignment (containing the main code of the program and the simulation files)**


## The program is implemented using the following nodes: 

- Gazebo simulator: for simulating the robot's behaviour in the predefined environment

- Slam gmapping node: for simultaneously localizing the robot and mapping the environment

- Move Base node: for path planning and driving the robot towards allocated targets in the environment

- tiserver service node: for generating random integer between min and max integers. (uses custom message of a request of 2 integers: min and max, and a response of a random integer in this range)

- Bug0 node: for driving the robot towards targets using different algorithm than move_base 

- Wall follower service node: for making the robot move following the walls in the environment

- Go to point service node: for driving the robot directly towards a point

- User interface main: for getting input from the user and executing the appropriate action, and displaying robot's status and information

## The rqt_graph of the program: 

![alt text](https://github.com/yaraalaa0/mobilerobot_control/blob/main/graph_2.PNG)




## Limitations

 The /move_base/status updates the status that indicates reaching the target after a relatively long time. I don't know why exactly is this, but it takes about 20 seconds after reaching the target to update its status. Probably, it takes into considerations very small errors in distance that cannot be detected with our eyes. 
 
 I have put the default values of ROS parameters target x and y positions (that are read by bug0) to zeros. I made the convention that if they are zeros, then bug0 knows that it shouldn't do anything. Howerver, if their values are different than 0, bug0 should start reaching this target. Probably, a better service routine could be implemented in order to activate or deactivate bug0 algorithm. 

