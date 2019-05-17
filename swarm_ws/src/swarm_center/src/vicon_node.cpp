/**
 * This is a node for vicon
 * Date: 2019.5 Author: Weifan Zhang
 */


#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>

#define DEFAULT_RATE 20


using namespace std;
using namespace cv;

class ViconNode
{
private:
    int robot_number;

//    ros::NodeHandle global;
//    ros::NodeHandle local;
    vector<ros::Publisher> raw_pub;
    ros::Subscriber vicon_sub;

    vector<float> curr_pos_x;
    vector<float> curr_pos_y;

public:
    ViconNode(ros::NodeHandle& nh){
        ROS_INFO("RECORD CENTER is online!");
        this->robot_number = 5;

        raw_pub.resize(this->robot_number);
        for (int i = 0; i < this->robot_number; ++i) {
            char msg_name[50];
            sprintf(msg_name,"/vicon/minifly%d/",i);
            raw_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(msg_name,1);
        }

        vicon_sub = nh.subscribe<vicon_bridge::Markers>("/vicon/markers",1000,&ViconNode::viconCallback,this);
    }

    void viconCallback(const vicon_bridge::Markers::ConstPtr &msg)
    {


    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "vicon_node");
    ros::NodeHandle n;

    ViconNode node(n);
//    node.run();
    ros::spin();
    return 0;
}