/**
 * This is the dispatch center for dynamic coverage trial
 * Date: 2019.4 Author: Weifan Zhang
 */


#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

#include "swarm_center/mCPPReq.h"
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/Twist.h>

#define DEFAULT_RATE 50

using namespace std;

class DispatchCenter
{
private:
    int robot_number;
    string prefix = "swarmbot";

    ros::NodeHandle local;
    ros::NodeHandle global;

    /* subscribe armed signal */
    ros::Subscriber ready_sub;

    /* Publish the velocity command */
    vector<ros::Publisher> flight_state_pub;

    /* client for coverage planning request */
    ros::ServiceClient plan_client;


    bool all_ready;

public:
    DispatchCenter(){
//        this->robot_number = robot_num;
        ros::NodeHandle local("/dispatch_center");
        ros::NodeHandle global("");
        this->local = local;
        this->global = global;

        if(!this->local.getParam("robot_number", this->robot_number)){
            ROS_WARN("Did not set up robot number, using default 1");
            this->robot_number = 1;
        }
        ROS_INFO("DISPATCH CENTER is activated! CENTER has %d robots to dispose", this->robot_number);
        
        this->all_ready = false;
        ready_sub = global.subscribe<std_msgs::Bool>("/setup_ready",1,&DispatchCenter::readyCallback,this);

        flight_state_pub.resize(this->robot_number);
        for (int i = 0; i < this->robot_number; ++i) {
            char msg_name[50];
            sprintf(msg_name,"/%s%d/flight_state",this->prefix.c_str(),i);
            flight_state_pub[i] = global.advertise<std_msgs::UInt8>(msg_name,1);
        }

        plan_client = global.serviceClient<swarm_center::mCPPReq>("mCPP_req");
    }

    void readyCallback(const std_msgs::Bool::ConstPtr &msg) {
        this->all_ready = msg->data;
    }

    void run(){
        ros::Rate r(DEFAULT_RATE);
        while(ros::ok()){
            /*update curr_pos and cmd_sp. Publish them.*/
        }
        ros::spinOnce();
        r.sleep();
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "dispatch_center");

    DispatchCenter node;
    node.run();

    return 0;
}