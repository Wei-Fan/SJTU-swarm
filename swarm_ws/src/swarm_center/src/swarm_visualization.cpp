/**
 * This is the recording center
 * Function: 1)visualization
 * Date: 2019.4 Author: Weifan Zhang
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
# include<geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>

using namespace std;

class Visual
{
private:
    int robot_number;
    string prefix = "swarmbot";

    ros::NodeHandle local;
    ros::NodeHandle global;

    /* raw position */
    vector<visualization_msgs::Marker> swarm_position_points;
    vector<visualization_msgs::Marker> swarm_position_line;
    vector<geometry_msgs::Point> old_swarm_position;
    vector<ros::Publisher> swarm_position_pub;
    vector<ros::Publisher> swarm_line_pub;
    vector<ros::Subscriber> swarm_position_sub;

    /* setpoint */
    vector<visualization_msgs::Marker> swarm_setpoint_points;
    vector<visualization_msgs::Marker> swarm_setpoint_line;
    vector<geometry_msgs::Point> old_swarm_setpoint;
    vector<ros::Publisher> swarm_setpoint_position_pub;
    vector<ros::Publisher> swarm_setpoint_line_pub;
    vector<ros::Subscriber> swarm_setpoint_sub;


public:
    Visual(){
        ros::NodeHandle local("/swarm_visualization");
        ros::NodeHandle global("");
        this->local = local;
        this->global = global;

        if(!this->local.getParam("robot_number", this->robot_number)){
            ROS_WARN("Did not set up robot number, using default 4");
            this->robot_number = 4;
        }

        /* raw position */
        swarm_position_points.resize(this->robot_number);
        swarm_position_line.resize(this->robot_number);
        old_swarm_position.resize(this->robot_number);
        swarm_position_pub.resize(this->robot_number);
        swarm_line_pub.resize(this->robot_number);
        swarm_position_sub.resize(this->robot_number);

        for (int i = 0; i < this->robot_number; ++i) {
            char msg_name[50];
            sprintf(msg_name,"/%s%d/raw_position",this->prefix.c_str(),i);
            swarm_position_sub[i] = global.subscribe<geometry_msgs::PoseStamped>(msg_name,1,&Visual::rawpositionCallback,this);
        }

        for (int i = 0; i < this->robot_number; ++i) {
            char msg_name0[50];
            sprintf(msg_name0,"/swarm_%d_position_marker",i);
            char msg_name1[50];
            sprintf(msg_name1,"/swarm_%d_line_marker",i);
            swarm_position_pub[i] = global.advertise<visualization_msgs::Marker>(msg_name0,1);
            swarm_line_pub[i] = global.advertise<visualization_msgs::Marker>(msg_name1,1);
        }

        /* setpoint */
        swarm_setpoint_points.resize(this->robot_number);
        swarm_setpoint_line.resize(this->robot_number);
        old_swarm_setpoint.resize(this->robot_number);
        swarm_setpoint_position_pub.resize(this->robot_number);
        swarm_setpoint_line_pub.resize(this->robot_number);
        swarm_setpoint_sub.resize(this->robot_number);

        for (int i = 0; i < this->robot_number; ++i) {
            char msg_name[50];
            sprintf(msg_name,"/%s%d/set_position",this->prefix.c_str(),i);
            swarm_setpoint_sub[i] = global.subscribe<geometry_msgs::PoseStamped>(msg_name,1,&Visual::setpositionCallback,this);
        }

        for (int i = 0; i < this->robot_number; ++i) {
            char msg_name0[50];
            sprintf(msg_name0,"/swarm_%d_set_position_marker",i);
            char msg_name1[50];
            sprintf(msg_name1,"/swarm_%d_set_line_marker",i);
            swarm_setpoint_position_pub[i] = global.advertise<visualization_msgs::Marker>(msg_name0,1);
            swarm_setpoint_line_pub[i] = global.advertise<visualization_msgs::Marker>(msg_name1,1);
        }

        init_param();

    }

    void rawpositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        int id = stoi(msg->header.frame_id);

        swarm_position_points[id].pose.position = msg->pose.position;
        swarm_position_points[id].pose.position.y = -swarm_position_points[id].pose.position.y;
        swarm_position_points[id].header.stamp = ros::Time::now();
        swarm_position_pub[id].publish(swarm_position_points[id]);

        geometry_msgs::Point  new_swarm_position = msg->pose.position;
        new_swarm_position.y = -new_swarm_position.y;
        double delta_x = old_swarm_position[id].x-new_swarm_position.x;
        double delta_y = old_swarm_position[id].y-new_swarm_position.y;
        if ( delta_x * delta_x + delta_y * delta_y<0.001)
          return;
        swarm_position_line[id].header.stamp = ros::Time::now();
        swarm_position_line[id].points.push_back(new_swarm_position);
        swarm_line_pub[id].publish(swarm_position_line[id]);
        old_swarm_position[id] = new_swarm_position;

    }

    void setpositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        int id = stoi(msg->header.frame_id);

        swarm_setpoint_points[id].pose.position = msg->pose.position;
        swarm_setpoint_points[id].pose.position.y = -swarm_setpoint_points[id].pose.position.y;
        swarm_setpoint_points[id].header.stamp = ros::Time::now();
        swarm_setpoint_position_pub[id].publish(swarm_setpoint_points[id]);

        geometry_msgs::Point  new_swarm_position = msg->pose.position;
        new_swarm_position.y = -new_swarm_position.y;
        double delta_x = old_swarm_setpoint[id].x-new_swarm_position.x;
        double delta_y = old_swarm_setpoint[id].y-new_swarm_position.y;
        if ( delta_x * delta_x + delta_y * delta_y<0.001)
            return;
        swarm_setpoint_line[id].header.stamp = ros::Time::now();
        swarm_setpoint_line[id].points.push_back(new_swarm_position);
        swarm_setpoint_line_pub[id].publish(swarm_setpoint_line[id]);
        old_swarm_position[id] = new_swarm_position;

    }

    void init_param() {

        /* random color */
        default_random_engine e;
        uniform_real_distribution<float> u(0.0,1.0);

        for (int i = 0; i < this->robot_number; ++i) {

            /* marker point of swarm 0 raw position */
            char ns_name0[50];
            sprintf(ns_name0,"swarm_%d_position_points",i);
            swarm_position_points[i].header.frame_id    = "world";
            swarm_position_points[i].ns                 = ns_name0;
            swarm_position_points[i].action             = visualization_msgs::Marker::ADD;
            swarm_position_points[i].lifetime           = ros::Duration(0);
            swarm_position_points[i].id                 = 0;
            swarm_position_points[i].type               = visualization_msgs::Marker::CYLINDER;
            swarm_position_points[i].scale.x            = 0.15;
            swarm_position_points[i].scale.y            = 0.15;
            swarm_position_points[i].scale.z            = 0.03;
            swarm_position_points[i].color.b            = 1.0;
            swarm_position_points[i].color.a            = 1.0;
            swarm_position_points[i].pose.orientation.w = 1.0;
            swarm_position_points[i].pose.position.x    = 0;
            swarm_position_points[i].pose.position.y    = 0;
            swarm_position_points[i].pose.position.z    = 0;

            /* marker line of swarm raw trajectory */
            char ns_name1[50];
            sprintf(ns_name1,"swarm_%d_position_line",i);
            swarm_position_line[i].header.frame_id    = "world";
            swarm_position_line[i].ns                 = ns_name1;
            swarm_position_line[i].action             = visualization_msgs::Marker::ADD;
            swarm_position_line[i].lifetime           = ros::Duration(0);
            swarm_position_line[i].id                 = 1;
            swarm_position_line[i].type               = visualization_msgs::Marker::LINE_STRIP;
            swarm_position_line[i].scale.x            = 0.2;
            swarm_position_line[i].color.r            = 1;
            swarm_position_line[i].color.g            = 0;
            swarm_position_line[i].color.b            = 0;
            swarm_position_line[i].color.a            = 1.0;
            swarm_position_line[i].pose.orientation.w = 1.0;

            /* variable record last position of swarm */
            old_swarm_position[i].x = 0;
            old_swarm_position[i].y = 0;
            old_swarm_position[i].z = 0;

            /* marker point of swarm set position */
            char ns_name2[50];
            sprintf(ns_name2,"swarm_%d_set_position_points",i);
            swarm_setpoint_points[i].header.frame_id    = "world";
            swarm_setpoint_points[i].ns                 = ns_name2;
            swarm_setpoint_points[i].action             = visualization_msgs::Marker::ADD;
            swarm_setpoint_points[i].lifetime           = ros::Duration(0);
            swarm_setpoint_points[i].id                 = 0;
            swarm_setpoint_points[i].type               = visualization_msgs::Marker::CYLINDER;
            swarm_setpoint_points[i].scale.x            = 0.1;
            swarm_setpoint_points[i].scale.y            = 0.1;
            swarm_setpoint_points[i].scale.z            = 0.03;
            swarm_position_line[i].color.r              = u(e);
            swarm_position_line[i].color.g              = u(e);
            swarm_position_line[i].color.b              = u(e);
            swarm_setpoint_points[i].pose.orientation.w = 1.0;
            swarm_setpoint_points[i].pose.position.x    = 0;
            swarm_setpoint_points[i].pose.position.y    = 0;
            swarm_setpoint_points[i].pose.position.z    = 0;

            /* marker line of swarm set trajectory */
            char ns_name3[50];
            sprintf(ns_name3,"swarm_%d_set_position_line",i);
            swarm_setpoint_line[i].header.frame_id    = "world";
            swarm_setpoint_line[i].ns                 = ns_name3;
            swarm_setpoint_line[i].action             = visualization_msgs::Marker::ADD;
            swarm_setpoint_line[i].lifetime           = ros::Duration(0);
            swarm_setpoint_line[i].id                 = 1;
            swarm_setpoint_line[i].type               = visualization_msgs::Marker::LINE_STRIP;
            swarm_setpoint_line[i].scale.x            = 0.2;
            swarm_setpoint_line[i].color.r            = u(e);
            swarm_setpoint_line[i].color.g            = u(e);
            swarm_setpoint_line[i].color.b            = u(e);
            swarm_setpoint_line[i].color.a            = 1.0;
            swarm_setpoint_line[i].pose.orientation.w = 1.0;

            /* variable record last position of swarm */
            old_swarm_position[i].x = 0;
            old_swarm_position[i].y = 0;
            old_swarm_position[i].z = 0;
        }
    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "swarm_visualization");
  ros::NodeHandle nh;

  Visual v;
  std::cout << "Start to visualize swarm !" << std::endl;
  ros::spin();

  return 0;

}

  