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

#define DEFAULT_RATE 1

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
    bool plan_ready;
    bool first_run;

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

        this->first_run = true;

        this->all_ready = false;
        ready_sub = global.subscribe<std_msgs::Bool>("/setup_ready",1,&DispatchCenter::armedReadyCallback,this);
        this->plan_ready = false;
        ready_sub = global.subscribe<std_msgs::Bool>("/plan_ready",1,&DispatchCenter::planReadyCallback,this);

        flight_state_pub.resize(this->robot_number);
        for (int i = 0; i < this->robot_number; ++i) {
            char msg_name[50];
            sprintf(msg_name,"/%s%d/flight_state",this->prefix.c_str(),i);
            flight_state_pub[i] = global.advertise<std_msgs::UInt8>(msg_name,1);
            std_msgs::UInt8 test;
            test.data = 0;
            for (int j = 0; j < 10; ++j) {
                flight_state_pub[i].publish(test);
            }
        }

        plan_client = global.serviceClient<swarm_center::mCPPReq>("/mCPP_req");
    }

    void armedReadyCallback(const std_msgs::Bool::ConstPtr &msg) {
        if(!this->all_ready) {
            this->all_ready = msg->data;
            ROS_INFO("vehicles are all armed");
        }
    }

    void planReadyCallback(const std_msgs::Bool::ConstPtr &msg) {
        this->plan_ready = msg->data;
        if (this->plan_ready) {
            ROS_INFO("plan ready!");
        }
    }

    void run(){
        ros::Rate r(DEFAULT_RATE);

        swarm_center::mCPPReq srv;
        srv.request.a = true;
        srv.response.b = false;
        while (ros::ok()&&!srv.response.b) {
            plan_client.call(srv);
            ros::spinOnce();
            r.sleep();
            ROS_INFO("request for first plan, response %d", srv.response.b);
        }

        while(ros::ok()){
            if (!this->plan_ready)//||!this->all_ready)
            {
//                ROS_INFO("wait");
                continue;
            }
//            ROS_INFO("armed ready and plan ready");
            if (first_run) {
                ROS_INFO("send takeoff signal");
                std_msgs::UInt8 flight_state;
                flight_state.data = 2;
                for (int i = 0; i < this->robot_number; ++i) {
                    for (int j = 0; j < 10; ++j) {
                        flight_state_pub[i].publish(flight_state);
                    }
                }
                first_run = false;
            }


            ros::spinOnce();
            r.sleep();
        }
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "dispatch_center");

    DispatchCenter node;
    node.run();

    return 0;
}