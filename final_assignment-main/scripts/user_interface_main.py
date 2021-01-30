#! /usr/bin/env python

# import ros stuff
import rospy
from std_srvs.srv import *
from my_srv.srv import TargetIndex
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from time import sleep

target_pos = [[-4,-3], [-4,2], [-4,7], [5,-7], [5,-3], [5,1]]  # initialize the six possible target positions
goal_flag = 0    # flag to indicate when move_base reach target
goal_flag_bug = 0  # flag to indicate when Bug0 reach target


def reached_pos(req):     # callback function when receiving empty message from bug0
    global goal_flag_bug
    goal_flag_bug=1     # reached the target using Bug0
    print("Target reached!")
    
    # set destination point to 0, to tell Bug0 to stop
    rospy.set_param("des_pos_x", 0)  
    rospy.set_param("des_pos_y", 0)
    
    return []


def clbk_status(msg):    # Callback function executed upon receiving the status message from move_base
    global goal_flag
    if(len(msg.status_list) > 0):
        if(msg.status_list[0].status == 3):     # status=3 indicates reaching the target
            goal_flag = 1
    



def main():
    global goal_flag, goal_flag_bug

    rospy.init_node('user_interface_main')   # initialize the ros node

    srv = rospy.Service('user_interface', Empty, reached_pos)   # initialize the service for knowing when Bug0 algorithm reaches target

    goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)   # publisher on topic move_base/goal

    sub_status = rospy.Subscriber('/move_base/status', GoalStatusArray, clbk_status, queue_size=1)   # subscriber to topic /move_base/status
 
    srv_client_target = rospy.ServiceProxy('/TargetIndex_random', TargetIndex)   # service client for receiving random target index

    srv_client_wall_follower = rospy.ServiceProxy('/wall_follower_switch', SetBool)  # service client for wall follower service

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # publisher on topic /cmd_vel

    

    print("Hello!")
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():        
        
        print("Please choose which request you want: 1, 2, 3, 4, or 5")
        r = float(raw_input('request: '))


        if(r==1):  # get random target index (from 1 to 6) and go to it using move_base

            resp = srv_client_wall_follower(False)        #disable wall_follower (if active)
            resp = srv_client_target(1,6)                 #call the service to get random target
            t_index = resp.t_pos     
            print("The random target index "+str(t_index)) 
            print("The current target is: x: "+str(target_pos[t_index-1][0])+" y: "+ str(target_pos[t_index-1][1]))

            # Send target (x,y) position to move_base
            MoveBase_msg = MoveBaseActionGoal()
            MoveBase_msg.goal.target_pose.header.frame_id = "map"
            MoveBase_msg.goal.target_pose.pose.orientation.w = 1
            MoveBase_msg.goal.target_pose.pose.position.x = target_pos[t_index-1][0]
            MoveBase_msg.goal.target_pose.pose.position.y = target_pos[t_index-1][1]
            goal_pub.publish(MoveBase_msg)

            print("We are currently approaching the target using Dijkstra")
            sleep(8)
            goal_flag = 0
            while(goal_flag == 0):   #Check if target has been reached yet
                sleep(1)
            print("We have reached the target")


        if(r==2):   #Get target from the user and go to it using move_base

            resp = srv_client_wall_follower(False)    # disable wall_follower (if active)
            # get target from user
            print("Please choose one of the six target positions")
            print("1: (-4,-3) , 2: (-4,2) , 3: (-4,7) , 4: (5,-7) , 5: (5,-3) , 6: (5,1)")
            t_index = int(raw_input('target index: '))
            print("The current target is: x: "+str(target_pos[t_index-1][0])+" y: "+ str(target_pos[t_index-1][1]))

            # Send target (x,y) position to move_base
            MoveBase_msg = MoveBaseActionGoal()
            MoveBase_msg.goal.target_pose.header.frame_id = "map"
            MoveBase_msg.goal.target_pose.pose.orientation.w = 1
            MoveBase_msg.goal.target_pose.pose.position.x = target_pos[t_index-1][0]
            MoveBase_msg.goal.target_pose.pose.position.y = target_pos[t_index-1][1]
            goal_pub.publish(MoveBase_msg)

            print("We are currently approaching the target using Dijkstra")
            sleep(8)
            goal_flag = 0
            while(goal_flag == 0):  #check if target has been reached yet
                sleep(1)
            print("We have reached the target")


        if(r==3):   # Start wall following
            resp = srv_client_wall_follower(True)   # call wall follower service and activates it
            print("I am following the wall")



        if(r==4):   # stop where you are 

            resp = srv_client_wall_follower(False)   #disable wall_follower (if active)

            # Set robot velocity to 0
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            pub.publish(twist_msg)
            print("I stopped")
            


        if(r==5):    #Use Bug0 algorithm to go to target

            resp = srv_client_wall_follower(False)   #disable wall_follower (if active)

            print("Changed to Bug0 Algorithm")
            # Get target from user
            print("Please choose one of the six target positions")
            print("1: (-4,-3) , 2: (-4,2) , 3: (-4,7) , 4: (5,-7) , 5: (5,-3) , 6: (5,1)")
            t_index = int(raw_input('target index: '))
            print("The current target is: x: "+str(target_pos[t_index-1][0])+" y: "+ str(target_pos[t_index-1][1]))

            #Set the target (x,y) position in ros parameter server
            rospy.set_param("des_pos_x", target_pos[t_index-1][0])
            rospy.set_param("des_pos_y", target_pos[t_index-1][1])
            print("We are currently approaching the target using Bug0")


            timeout=0
            goal_flag_bug=0
            while(goal_flag_bug == 0):   #check if target has been reached yet
                sleep(1)
                timeout = timeout +1

                #Checking if timeout value has been reached
                if(timeout==60):
                    #terminate and stop the robot
                    goal_flag_bug = 1
                    rospy.set_param("des_pos_x", 0)
                    rospy.set_param("des_pos_y", 0)
                    twist_msg = Twist()
                    twist_msg.linear.x = 0
                    twist_msg.angular.z = 0
                    pub.publish(twist_msg)
                    print("I stopped")
                    print("Timeout expired")

        rate.sleep()


if __name__ == '__main__':
    main()
