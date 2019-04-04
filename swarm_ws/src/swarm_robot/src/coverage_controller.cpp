#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>

#define DEFAULT_RATE 50

using namespace std;

string traj_path = "/home/wade/SJTU-swarm/swarm_ws/src/swarm_center/launch/";

enum FlightState{
    Hovering=1, Takeoff=2, Commanding=3, Landing=4,
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

public:
    CoverageController(const string &robot_name, bool debug = false){
        this->robot_name = robot_name;
        ROS_INFO("ROBOT CONTROLLER for %s is activated!",this->robot_name.c_str());
        m_flight_state = Takeoff;

        /* publisher and subscriber */
        char msg_name0[50];
        sprintf(msg_name0,"/%s/flight_state",this->robot_name.c_str());
        state_sub = global.subscribe<std_msgs::UInt8>(msg_name0,1,&CoverageController::flightStateCallback,this);

        char msg_name1[50];
        sprintf(msg_name1,"/%s/raw_position",this->robot_name.c_str());
        pos_sub = global.subscribe<geometry_msgs::Pose>(msg_name1,1,&CoverageController::positionCallback,this);

        char msg_name2[50];
        sprintf(msg_name2,"/%s/set_position",this->robot_name.c_str());
        cmd_pos_pub = global.advertise<geometry_msgs::Pose>(msg_name2,1);

        /* read file */
        int swarm_prefix_size = this->swarm_prefix.size();
        int robot_name_size = this->robot_name.size();
        string index = this->robot_name.substr(swarm_prefix_size,robot_name_size);
        this->robot_id = atoi(index.c_str());
        read_Pos_traj();
    }

    void flightStateCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        m_flight_state = FlightState(int(msg->data));
    }

    void positionCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        curr_pos = *msg;
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