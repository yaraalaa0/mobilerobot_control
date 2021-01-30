# mobilerobot_control



To run the simulation:

- Clone the repository into your ROS workspace

- Go inside the root of your workspace and run:

  $ catkin_make

  $ roslaunch final_assignment simulation_gmapping.launch
  
  $ roslaunch final_assignment move_base.launch
  
  $ roslaunch final_assignment bug0.launch
  
  $ rosrun final_assignment user_interface_main.py




This program is composed of several nodes: 

- Gazebo simulator: for simulating the robot's behaviour in the predefined environment

- Slam gmapping node: for simultaneously localizing the robot and mapping the environment

- Move Base node: for path planning and driving the robot towards allocated targets in the environment

- Bug0 node: for driving the robot towards targets using different algorithm than move_base 

- Wall follower service node: for making the robot move following the walls in the environment

- Go to point service node: for driving the robot directly towards a point

- User interface main: for getting input from the user and executing the appropriate action, and displaying robot's status and information

The rqt_graph of the program: 

![alt text](https://github.com/yaraalaa0/mobilerobot_control/blob/main/graph_2.PNG)


This program is composed of two packages: my_srv and final_assignment




