# mobilerobot_control

This is a ROS program, that simulates the navigation behaviour of an UGV robot in an environment of 6 rooms. The program asks the user to enter 
a request to specify what he wants the robot to do. The use can choose between 5 requests:

1 -> go to a random target position (out of 6 possible target positions) - using move_base -

2 -> enter a specific target position for the robot to go to  - using move_base -

3 -> make the robot start following the walls

4 -> stop where you are

5 -> go to a specified target position using Bug0 algorithm (which is less efficient than move_base)




## The program is implemented using the following nodes: 

- Gazebo simulator: for simulating the robot's behaviour in the predefined environment

- Slam gmapping node: for simultaneously localizing the robot and mapping the environment

- Move Base node: for path planning and driving the robot towards allocated targets in the environment

- tiserver service node: for generating random integer between min and max integers. (uses custom message of a request of 2 integers: min and max, and a response of a random integer in this range)

- Bug0 node: for driving the robot towards targets using different algorithm than move_base 

- Wall follower service node: for making the robot move following the walls in the environment

- Go to point service node: for driving the robot directly towards a point

- User interface main: for getting input from the user and executing the appropriate action, and displaying robot's status and information

The rqt_graph of the program: 

![alt text](https://github.com/yaraalaa0/mobilerobot_control/blob/main/graph_2.PNG)



## To run the simulation:

- Clone the repository into your ROS workspace

- Go inside the root of your workspace and run:

  $ catkin_make

  $ roslaunch final_assignment simulation_gmapping.launch
  
  $ roslaunch final_assignment move_base.launch
  
  $ roslaunch final_assignment bug0.launch
  
  $ rosrun final_assignment user_interface_main.py






This program is composed of two packages: my_srv and final_assignment




