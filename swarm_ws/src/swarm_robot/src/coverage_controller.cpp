#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>

#define DEFAULT_RATE 20

using namespace std;

string traj_path = "/home/wade/SJTU-swarm/swarm_ws/src/swarm_center/launch/";

enum FlightState{
    None=0, Hovering=1, Takeoff=2, Commanding=3, Landing=4,
};

class csvdata{
public:
    float xyz[3];
};

class CoverageController
{
private:
    int robot_id = 0;
    string swarm_prefix = "swarmbot";
    string robot_name;
    FlightState m_flight_state;
    bool plan_ready;
    bool state_prepare;
    bool pos_prepare;
    geometry_msgs::Pose curr_pos;
    geometry_msgs::Twist cmd_sp, cmd_vel;

    ros::NodeHandle global;
    ros::NodeHandle local;

    /* subscribe current pos */
    ros::Subscriber pos_sub;

    /* subscribe current states from dispatch center */
    ros::Subscriber state_sub;

    /*Publish the position command*/
    ros::Publisher cmd_pos_pub;

    vector<csvdata> cmd_incsv;
    int cmd_cnt;
    int cmd_limit;

public:
    CoverageController(const string &robot_name, bool debug = false){
        this->robot_name = robot_name;
        ROS_INFO("ROBOT CONTROLLER for %s is activated!",this->robot_name.c_str());
        m_flight_state = None;
        state_prepare = false;
        pos_prepare = false;
        plan_ready = false;

        /* publisher and subscriber */
        // wait for dispatch_center's signal to alter flight state
        char msg_name0[50];
        sprintf(msg_name0,"/%s/flight_state",this->robot_name.c_str());
        state_sub = global.subscribe<std_msgs::UInt8>(msg_name0,1,&CoverageController::flightStateCallback,this);

        // get raw_pos from swarm_driver
        char msg_name1[50];
        sprintf(msg_name1,"/%s/raw_position",this->robot_name.c_str());
        pos_sub = global.subscribe<geometry_msgs::PoseStamped>(msg_name1,1,&CoverageController::positionCallback,this);

        // publish sp to swarm_driver
        char msg_name2[50];
        sprintf(msg_name2,"/%s/set_position",this->robot_name.c_str());
        cmd_pos_pub = global.advertise<geometry_msgs::Pose>(msg_name2,1);

        /* read file (how to respond to replanning???)*/
        int swarm_prefix_size = this->swarm_prefix.size();
        int robot_name_size = this->robot_name.size();
        string index = this->robot_name.substr(swarm_prefix_size,robot_name_size);
        this->robot_id = atoi(index.c_str());
        read_Pos_traj();
        cmd_cnt = 0;
        cmd_limit = cmd_incsv.size();
    }

    void flightStateCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        if (msg->data == 0) {
            ROS_WARN("state not set!");
            return;
        }

        m_flight_state = FlightState(int(msg->data));
        state_prepare = true;
        ROS_INFO("flight state set!");
    }

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        curr_pos = msg->pose;
        pos_prepare = true;
    }

    void read_Pos_traj()
    {
        csvdata intp;
        FILE *fp;
        string filename = traj_path + "cover_robot" + to_string(robot_id) + ".csv";
        fp = fopen(filename.c_str(),"r");
        if(fp){cout<<"open path csv"<< filename << endl;}
        while(1){
            fscanf(fp,"%f,%f,%f",&intp.xyz[0],&intp.xyz[1],&intp.xyz[2]);
            cmd_incsv.push_back(intp);
            // std::cout<<"takeoff_pos: x="<<intp.xyz[0]<<"; y="<<intp.xyz[1]<<"; z="<<intp.xyz[2]<<std::endl;
            if (feof(fp))break;
        }
        fclose(fp);
    }

    void run(){
        int cnt = 0;
        ros::Rate loop_rate(DEFAULT_RATE);
        while(ros::ok()){
            /*update curr_pos and cmd_sp. Publish them.*/
            if (!state_prepare||!pos_prepare)//undo this once we add swarm_driver
            {
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }

            geometry_msgs::Pose pos_sp;
            if (cnt == cmd_limit)
                m_flight_state = Landing;

            switch (m_flight_state) {
                case Hovering: {
                    pos_sp.position.x = cmd_incsv[cmd_cnt].xyz[0];
                    pos_sp.position.y = cmd_incsv[cmd_cnt].xyz[1];
                    pos_sp.position.z = cmd_incsv[cmd_cnt].xyz[2];
                    cmd_pos_pub.publish(pos_sp);
                    break;
                }
                case Takeoff: {
                    pos_sp.position.x = cmd_incsv[0].xyz[0];
                    pos_sp.position.y = cmd_incsv[0].xyz[1];
                    pos_sp.position.z = 0;
                    for (int i = 1; i <= 40; ++i) {
                        pos_sp.position.z += cmd_incsv[0].xyz[2]*i/40;

                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }

                    for (int i = 1; i <= 40; ++i) {
                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                    m_flight_state = Commanding;
                    break;
                }
                case Commanding:{
                    pos_sp.position.x = cmd_incsv[cmd_cnt].xyz[0];
                    pos_sp.position.y = cmd_incsv[cmd_cnt].xyz[1];
                    pos_sp.position.z = cmd_incsv[cmd_cnt].xyz[2];
                    cmd_pos_pub.publish(pos_sp);
                    cmd_cnt++;
                    ROS_INFO("robot%d count : %d",this->robot_id,cmd_cnt);
                    ros::spinOnce();
                    loop_rate.sleep();
                    break;
                }
                case Landing:{
                    pos_sp.position.x = cmd_incsv[cmd_cnt].xyz[0];
                    pos_sp.position.y = cmd_incsv[cmd_cnt].xyz[1];
                    pos_sp.position.z = cmd_incsv[cmd_cnt].xyz[2];
                    for (int i = 1; i <= 80; ++i) {
                        pos_sp.position.z -= cmd_incsv[cmd_cnt].xyz[2]*(1-i/80);
                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                    break;
                }
                default:{
                    pos_sp.position.x = curr_pos.position.x;
                    pos_sp.position.y = curr_pos.position.y;
                    pos_sp.position.z = curr_pos.position.z;
                    cmd_pos_pub.publish(pos_sp);
                    ros::spinOnce();
                    loop_rate.sleep();
                    break;
                }
            }
//            ROS_INFO("set_position for robot%d : %d, %d",this->robot_id,pos_sp.position.x,pos_sp.position.y);
        }
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