#include "ros/ros.h"
#include "my_srv/TargetIndex.h"
#include "time.h"

//selecting a random integer between M and N 
int randMtoN(int M, int N){
int ind = (rand()% N) + M;
srand(time(NULL));
return ind;       
}

bool callback_random(my_srv::TargetIndex::Request &req, my_srv::TargetIndex::Response &res){
    res.t_pos = randMtoN(req.min, req.max);
    return true;
}


int main(int argc, char **argv){
    
    ros::init(argc,argv, "tiserver");

    ros::NodeHandle n;

    ros::ServiceServer tgt_serv = n.advertiseService("/TargetIndex_random", callback_random);


    ros::spin();
    return 0;
}
