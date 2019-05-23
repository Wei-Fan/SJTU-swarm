/**
 * This is the robot controller for dynamic coverage trials
 * Function: 1)individual robot controller containing multiple action choices
 * Date: 2019.5 Author: Weifan Zhang
 */

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>

#include "swarm_center/mArmReq.h"

//#include <swarm_robot/p>

#define RUN_RATE 20
#define SLEEP_RATE 2

using namespace std;

string traj_path = "/home/wade/SJTU-swarm/swarm_ws/src/swarm_center/launch/";

float TAKEOFF_POS[2] = {-2,0};
float LANDING_POS[2] = {2,0};

/* action-behavior declaration */
enum FlightState{
    None=0, Hovering=1, Takeoff=2, Commanding=3, Landing=4, Returnback=5, Headout=6 // None and Hovering are also served as waiting states
};

class FlightBehavior{
public:
    int behavior_index;
    vector<FlightState> action_seq;
    FlightBehavior(int a = 0){behavior_index = a;}
    ~FlightBehavior(){}
};


class csvdata{
public:
    float xyz[3];
};

class CoverageController
{
private:
    FlightBehavior EXEC_FROM_BEGNNING;
    FlightBehavior EXEC_FROM_MIDDLE;
    FlightBehavior DISENGAGE;
    FlightBehavior ENGAGE;
    FlightBehavior SLEEP;
    FlightBehavior m_flight_behavior;
    FlightState m_flight_state;
    int m_flight_curr_index;
    bool taskOccupiedLight; //differ sleep mode and task mode
    bool taskPauseLight; //true when need to stay in None or Hovering state
    bool taskIndexLight; //true when plan ready and recieve from dispatch center
    int task_index; //used to read file (different from behavior/task index)
    bool taskAlterLight; //true when recieve a new task
    bool plan_ready;

    int robot_id = 0;
    string swarm_prefix = "swarmbot";
    string robot_name;
    bool state_prepare;
    bool pos_prepare;
    geometry_msgs::Pose curr_pos;
    geometry_msgs::Twist cmd_sp, cmd_vel;

    ros::NodeHandle global;
    ros::NodeHandle local;

    /* subscribe current pos */
    ros::Subscriber pos_sub;

    /* subscribe current states from dispatch center */
    ros::Subscriber task_sub;
    ros::Subscriber index_sub;

    /*Publish the position command*/
    ros::Publisher cmd_pos_pub;

    /* call arm */
    ros::ServiceClient arm_client;

    vector<csvdata> cmd_incsv;
    int cmd_cnt;
    int cmd_limit;


public:
    CoverageController(const string &robot_name, bool debug = false){
        ros::NodeHandle local("/coverage_controller");
        ros::NodeHandle global("");
        this->local = local;
        this->global = global;
        /* define behavior */
        //change behavior only can be allowed when no drone is in disengage mode
        EXEC_FROM_BEGNNING.behavior_index = 1;
        EXEC_FROM_BEGNNING.action_seq = {None,Takeoff,Hovering,Commanding,Landing};
        EXEC_FROM_MIDDLE.behavior_index = 2;
        EXEC_FROM_MIDDLE.action_seq = {Hovering,Commanding,Landing};
        DISENGAGE.behavior_index = 3;
        DISENGAGE.action_seq = {Hovering,Returnback,Landing};
        ENGAGE.behavior_index = 4;
        ENGAGE.action_seq = {None,Takeoff,Headout,Hovering,Commanding,Landing};
        SLEEP.behavior_index = 0;
        SLEEP.action_seq = {None};

        /* flags */
//        m_flight_state = None;
        state_prepare = false;
        this->pos_prepare = false;
        this->plan_ready = false;
        this->taskOccupiedLight = false;
        this->taskPauseLight = true;
        this->taskIndexLight = false;
        this->taskAlterLight = false;

        /* get robot id and initial task id */
        this->robot_name = robot_name;
        ROS_INFO("ROBOT CONTROLLER for %s is activated!",this->robot_name.c_str());
        int swarm_prefix_size = this->swarm_prefix.size();
        int robot_name_size = this->robot_name.size();
        string index = this->robot_name.substr(swarm_prefix_size,robot_name_size);
        this->robot_id = atoi(index.c_str());
        this->task_index = this->robot_id;
        // init m_flight_state
        this->m_flight_behavior = SLEEP;
        this->m_flight_curr_index = 0;
        this->m_flight_state = this->m_flight_behavior.action_seq[m_flight_curr_index];

        /* publisher and subscriber */
        // wait for dispatch_center's signal to alter flight state
        char msg_name0[50];
        sprintf(msg_name0,"/%s/flight_task",this->robot_name.c_str());
        task_sub = global.subscribe<std_msgs::UInt8>(msg_name0,1,&CoverageController::flightTaskCallback,this);

        // get raw_pos from swarm_driver
        char msg_name1[50];
        sprintf(msg_name1,"/%s/raw_position",this->robot_name.c_str());
        pos_sub = global.subscribe<geometry_msgs::PoseStamped>(msg_name1,1,&CoverageController::positionCallback,this);

        // publish sp to swarm_driver
        char msg_name2[50];
        sprintf(msg_name2,"/%s/set_position",this->robot_name.c_str());
        cmd_pos_pub = global.advertise<geometry_msgs::PoseStamped>(msg_name2,1);

        // wait for dispatch_center's signal to alter task number for planning file
        char msg_name3[50];
        sprintf(msg_name3,"/%s/task_index",this->robot_name.c_str());
        index_sub = global.subscribe<std_msgs::UInt8>(msg_name3,1,&CoverageController::taskIndCallback,this);

        // call armed through swarm_driver
        arm_client = global.serviceClient<swarm_center::mArmReq>("/mArm_req");

        curr_pos.position.x = 0.0;
        curr_pos.position.y = 0.0;
    }

    void flightTaskCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        // 1--EXEC_FROM_BEGINNING; 2--EXEC_FROM_MIDDLE; 3--DISENGAGE; 4--ENGAGE
        // 0--init check
        int new_index = (int)msg->data;
        ROS_INFO("robot %d change flight task to %d", this->robot_id, new_index);
        fflush(stdout);
        if (new_index != this->m_flight_behavior.behavior_index) {
            this->taskAlterLight = true;
            switch (new_index) {
                case 1: {
                    this->m_flight_behavior = EXEC_FROM_BEGNNING;
                    this->m_flight_curr_index = 0;
                    break;
                }
                case 2: {
                    this->m_flight_behavior = EXEC_FROM_MIDDLE;
                    this->m_flight_curr_index = 0;
                    break;
                }
                case 3: {
                    this->m_flight_behavior = DISENGAGE;
                    this->m_flight_curr_index = 0;
                    break;
                }
                case 4: {
                    this->m_flight_behavior = ENGAGE;
                    this->m_flight_curr_index = 0;
                    break;
                }
                default: {
                    this->m_flight_behavior = SLEEP;
                    this->m_flight_curr_index = 0;
                    this->taskAlterLight = false;
                    this->taskOccupiedLight = false;
                }
            }
        }
//        state_prepare = true;
    }

    void taskIndCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        ROS_INFO("robot %d alter task Number to %d",this->robot_id, msg->data);
        this->taskIndexLight = true;
        this->task_index = (int)msg->data;
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
        string filename = traj_path + "cover_robot" + to_string(this->task_index) + ".csv";
        fp = fopen(filename.c_str(),"r");
        if(fp){cout<<"open path csv"<< filename << endl;}
        while(true){
            fscanf(fp,"%f,%f,%f",&intp.xyz[0],&intp.xyz[1],&intp.xyz[2]);
            cmd_incsv.push_back(intp);
            // std::cout<<"takeoff_pos: x="<<intp.xyz[0]<<"; y="<<intp.xyz[1]<<"; z="<<intp.xyz[2]<<std::endl;
            if (feof(fp))break;
        }
        fclose(fp);
    }

    void run(){
        ros::Rate run_rate(RUN_RATE);
        ros::Rate sleep_rate(SLEEP_RATE);

        geometry_msgs::PoseStamped pos_sp;
        pos_sp.header.frame_id = to_string(this->robot_id);
        pos_sp.pose.position.x = curr_pos.position.x;
        pos_sp.pose.position.y = curr_pos.position.y;
        pos_sp.pose.position.z = -1;
        /* test link */
        for (int i = 1; i <= 20; ++i) {//*(this->robot_id+1); ++i) {
            cmd_pos_pub.publish(pos_sp);
            ros::spinOnce();
            run_rate.sleep();
        }

        ROS_INFO("robot %d start main loop", this->robot_id);
        /* main loop */
        while (ros::ok()) {
            if (this->taskAlterLight) {
                this->plan_ready = false;
                this->taskPauseLight = true;
                this->taskOccupiedLight = true;
            }

            if (this->taskIndexLight&&this->taskOccupiedLight) {
                /* read file */
                cmd_incsv.clear();
                read_Pos_traj();
                cmd_cnt = 0;
                cmd_limit = cmd_incsv.size();
                this->taskPauseLight = false;
                this->taskIndexLight = false;
            }

            this->m_flight_state = this->m_flight_behavior.action_seq[m_flight_curr_index];
            geometry_msgs::PoseStamped pos_sp;
            pos_sp.header.frame_id = to_string(this->robot_id);
            switch (this->m_flight_state) {
                case None: {
                    if (!this->taskPauseLight&&this->pos_prepare) {
                        ROS_INFO("robot %d not long stay put~", this->robot_id);
                        this->m_flight_curr_index++;
                    }
                    if (this->taskOccupiedLight) {
                        ros::spinOnce();
                        run_rate.sleep();
                    } else {
//                        ROS_INFO("robot %d sleeps", this->robot_id);
                        ros::spinOnce();
                        sleep_rate.sleep();
                    }
                    break;
                }
                case Takeoff: {
                    ROS_INFO("robot %d taking off", this->robot_id);
                    pos_sp.pose.position.x = curr_pos.position.x;
                    pos_sp.pose.position.y = curr_pos.position.y;
                    pos_sp.pose.position.z = -1;

                    /* test link */
                    for (int i = 1; i <= 20; ++i) {//*(this->robot_id+1); ++i) {
                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        run_rate.sleep();
                    }

                    /* call arm */
                    swarm_center::mArmReq srv;
                    srv.request.a = this->robot_id;
                    srv.response.b = false;
                    while (ros::ok()&&!srv.response.b) {
                        arm_client.call(srv);
                        ros::spinOnce();
                        run_rate.sleep();
                    }
                    ROS_INFO("request for arm successfully");

                    for (int i = 1; i <= 60; ++i) {//*(this->robot_id+1); ++i) {
                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        run_rate.sleep();
                    }

                    float height = 1.5;
                    if (this->m_flight_behavior.behavior_index == 4)
                        height = 1.0;
                    pos_sp.pose.position.z = 0;
                    for (int i = 1; i <= 200; ++i) {
                        pos_sp.pose.position.z = height*float(i)/200.0;

                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        run_rate.sleep();
                    }

                    for (int i = 1; i <= 40; ++i) {
                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        run_rate.sleep();
                    }

                    ROS_INFO("robot %d has taken off", this->robot_id);
                    this->m_flight_curr_index++;

                    break;
                }
                case Hovering: {
                    ROS_INFO("robot %d taking off", this->robot_id);
                    pos_sp.pose.position = curr_pos.position;
                    pos_sp.pose.position.z = 1.5;
                    for (int i = 1; i <= 40; ++i) {
                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        run_rate.sleep();
                    }

                    if (!this->taskPauseLight) {
                        this->m_flight_curr_index++;
                    }

                    break;
                }
                case Returnback: {
                    ROS_INFO("robot %d 's returning back to assemble point",this->robot_id);

                    /* descent to return back height */
                    pos_sp.pose = curr_pos;
                    pos_sp.pose.position.z = 1.5;
                    for (int i = 0; i < 60; ++i) {
                        pos_sp.pose.position.z -= 1.0*float(i)/40;
                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        run_rate.sleep();
                    }

                    float start_x = curr_pos.position.x;
                    float start_y = curr_pos.position.y;
                    for (int i = 1; i <= 120; ++i) {
                        pos_sp.pose.position.x = float(i)/120*(LANDING_POS[0]-start_x)+start_x;
                        pos_sp.pose.position.y = float(i)/120*(LANDING_POS[1]-start_y)+start_y;

                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        run_rate.sleep();
                    }

                    this->m_flight_curr_index++;
                    break;
                }
                case Headout: {
                    ROS_INFO("robot %d 's heading out to assigned position",this->robot_id);
                    float start_x = curr_pos.position.x;
                    float start_y = curr_pos.position.y;
                    pos_sp.pose.position.z = 1.0;
                    for (int i = 1; i <= 120; ++i) {
                        pos_sp.pose.position.x = float(i)/120*(cmd_incsv[0].xyz[0]-start_x)+start_x;
                        pos_sp.pose.position.y = float(i)/120*(cmd_incsv[0].xyz[1]-start_y)+start_y;

                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        run_rate.sleep();
                    }

                    /* pull up to normal height */
                    for (int i = 0; i < 40; ++i) {
                        pos_sp.pose.position.z += 0.5*float(i)/40;
                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        run_rate.sleep();
                    }

                    this->m_flight_curr_index++;
                    break;
                }
                case Landing: {
                    pos_sp.pose.position.x = cmd_incsv[cmd_cnt].xyz[0];
                    pos_sp.pose.position.y = cmd_incsv[cmd_cnt].xyz[1];
                    pos_sp.pose.position.z = cmd_incsv[cmd_cnt].xyz[2];
                    for (int i = 1; i <= 80; ++i) {
                        pos_sp.pose.position.z -= 1.5*(1-i/80);
                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        run_rate.sleep();
                    }

                    /* put robot to sleep */
                    pos_sp.pose.position.x = cmd_incsv[cmd_cnt].xyz[0];
                    pos_sp.pose.position.y = cmd_incsv[cmd_cnt].xyz[1];
                    pos_sp.pose.position.z = -1;

                    for (int i = 1; i <= 40; ++i) {//*(this->robot_id+1); ++i) {
                        cmd_pos_pub.publish(pos_sp);
                        ros::spinOnce();
                        run_rate.sleep();
                    }

                    this->taskOccupiedLight = false;
                    this->m_flight_behavior = SLEEP;
                    this->m_flight_curr_index = 0;
                    ROS_INFO("robot %d sleep", this->robot_id);

                    break;
                }
                case Commanding: {
                    pos_sp.pose.position.x = cmd_incsv[cmd_cnt].xyz[0];
                    pos_sp.pose.position.y = cmd_incsv[cmd_cnt].xyz[1];
                    pos_sp.pose.position.z = cmd_incsv[cmd_cnt].xyz[2];
                    cmd_pos_pub.publish(pos_sp);
                    cmd_cnt++;
//                    ROS_INFO("robot%d count : %d",this->robot_id,cmd_cnt);
                    ros::spinOnce();
                    run_rate.sleep();

                    if (cmd_cnt == cmd_limit)
                        this->m_flight_curr_index++;
                    break;
                }
            }
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