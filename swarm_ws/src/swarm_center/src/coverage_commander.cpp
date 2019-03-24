/**
 * This is the commander for multi robot coverage trial
 * Function: 1)wait for planning request; 2)Keep track of every robot covering area
 * Date: 2019.3 Author: Weifan Zhang
 */


#include <ros/ros.h>
#include <iostream>
#include <string>
#include "swarm_center/mCPPReq.h"
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/Twist.h>

#define DEFAULT_RATE 50

using namespace std;
using namespace Eigen;

class CoverageCommander
{
private:
    int robot_number;

    ros::NodeHandle global;
    ros::NodeHandle local;

//    type? map

public:
    CoverageCommander(){
        ROS_INFO("SWARM COMMANDER is activated!");
//        this->robot_number = robot_num;
    }

    bool plan(swarm_center::mCPPReq::Request &req,
              swarm_center::mCPPReq::Response &res)
    {
        return true;
    }
//    void run(){
//        ros::Rate r(DEFAULT_RATE);
//        while(ros::ok()){
//            /*update curr_pos and cmd_sp. Publish them.*/
//        }
//        ros::spinOnce();
//        r.sleep();
//    }

private:
    ros::ServiceServer service = local.advertiseService("mCPP_req",&CoverageCommander::plan,this);


};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "coverage_commander");
    ros::Rate r(1);

    CoverageCommander node;
//    node.run();

    ros::spinOnce();
    r.sleep();
    return 0;
}