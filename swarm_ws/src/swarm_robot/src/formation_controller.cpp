#include <ros/ros.h>
#include <iostream>
#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <swarm_robot/formation_controller.h>

#define DEFAULT_RATE 50

using namespace std;

FormationController::FormationController(const string &robot_name, bool debug) {
    this->robot_name = robot_name;
    ROS_INFO("FORMATION CONTROL behavior is enacted!");
}

void FormationController::run() {
    ros::Rate r(DEFAULT_RATE);
    while(ros::ok()){
//        ROS_INFO(" run formation~~~");
    }
    ros::spinOnce();
    r.sleep();
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "formation_controller");


    if(argc<2) {
        ROS_WARN("No robot name has been specified. Shutting down robot controller.");
    } else {
        if(argc<3) {
            FormationController node(argv[1]);
            node.run();
        } else {
            FormationController node(argv[1], true);
            node.run();
        }
    }

    return 0;
}