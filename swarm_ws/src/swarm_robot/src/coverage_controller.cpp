#include <ros/ros.h>
#include <iostream>
#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#define DEFAULT_RATE 50

using namespace std;

class CoverageController
{
private:
    string robot_name;
    geometry_msgs::Pose curr_pos;
    geometry_msgs::Twist cmd_sp, cmd_vel;

    ros::NodeHandle global;
    ros::NodeHandle local;

    /*Publish the setpoint*/
//    ros::Publisher cmd_sp_pub;

    /*Publish the velocity command*/
//    ros::Publisher cmd_vel_pub;

public:
    CoverageController(const string &robot_name, bool debug = false){
        this->robot_name = robot_name;
        ROS_INFO("ROBOT CONTROLLER for %s is activated!",this->robot_name.c_str());
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
    ros::init(argc, argv, "coverage_controller");


    if(argc<2) {
        ROS_WARN("No robot name has been specified. Shutting down robot controller.");
    } else {
        if(argc<3) {
            CoverageController node(argv[1]);
            node.run();
        } else {
            CoverageController node(argv[1], true);
            node.run();
        }
    }

    return 0;
}