#include <ros/ros.h>
#include <iostream>
#include <string>
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/Twist.h>

#define DEFAULT_RATE 50

using namespace std;

class SwarmCommander
{
private:
    int robot_number;

    ros::NodeHandle global;
    ros::NodeHandle local;

    /*Publish the setpoint*/
//    ros::Publisher cmd_sp_pub;

    /*Publish the velocity command*/
//    ros::Publisher cmd_vel_pub;

public:
    SwarmCommander(){
        ROS_INFO("SWARM COMMANDER is activated!");
//        this->robot_number = robot_num;
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
    ros::init(argc, argv, "swarm_commander");

    SwarmCommander node;
    node.run();

    return 0;
}