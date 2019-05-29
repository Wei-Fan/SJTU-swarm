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
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
//#include <geometry_msgs/Twist.h>

#define DEFAULT_RATE 2

using namespace std;

class DispatchCenter
{
private:
    int robot_number;
    int active_number;
    vector<bool> active_list;
    string prefix = "swarmbot";

    ros::NodeHandle local;
    ros::NodeHandle global;

    /* switch require */
    ros::Subscriber sw_sub;

    /* subscribe raw position */
    vector<ros::Subscriber> raw_sub;
    vector<geometry_msgs::Pose> curr_pos;

    /* Publish the velocity command */
    vector<ros::Publisher> flight_task_pub;
    vector<ros::Publisher> flight_index_pub;

    /* client for coverage planning request */
    ros::ServiceClient plan_client;
    ros::Subscriber ready_sub;
//    bool all_ready;
    bool switch_signal;
    bool plan_ready;
    bool first_run;
    vector<bool> start_first_cnt;

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
        this->active_number = this->robot_number - 1;
        active_list.resize(this->robot_number, true);
        active_list[this->robot_number-1] = false;

        sw_sub = global.subscribe<std_msgs::Empty>("/switch_command",1,&DispatchCenter::switchCallback,this);

        /* coverage commander */
        plan_client = global.serviceClient<swarm_center::mCPPReq>("/mCPP_req");
        this->plan_ready = false;
        ready_sub = global.subscribe<std_msgs::Bool>("/plan_ready",1,&DispatchCenter::planReadyCallback,this);

        /* task allocation */
        flight_task_pub.resize(this->robot_number);
        flight_index_pub.resize(this->robot_number);
        for (int i = 0; i < this->robot_number; ++i) {
            char msg_name0[50];
            sprintf(msg_name0, "/%s%d/flight_task", this->prefix.c_str(), i);
            flight_task_pub[i] = global.advertise<std_msgs::UInt8>(msg_name0, 1);
            std_msgs::UInt8 test;
            test.data = 0;
            for (int j = 0; j < 10; ++j) {
                flight_task_pub[i].publish(test);
            }
        }
        for (int i = 0; i < this->robot_number; ++i) {
            char msg_name1[50];
            sprintf(msg_name1,"/%s%d/task_index",this->prefix.c_str(),i);
            flight_index_pub[i] = global.advertise<std_msgs::UInt8>(msg_name1,1);
        }

        /* obtain raw position for each time planning */
        raw_sub.resize(this->robot_number);
        curr_pos.resize(this->robot_number);
        for (int i = 0; i < this->robot_number; ++i) {
            char msg_name[50];
            sprintf(msg_name, "/%s%d/raw_position", this->prefix.c_str(), i);
            raw_sub[i] = global.subscribe<geometry_msgs::PoseStamped>(msg_name, 1, &DispatchCenter::rawPosCallback, this);
        }

        this->switch_signal = false;
        this->first_run = false;
        this->start_first_cnt.resize(this->robot_number, false);
    }

//    void armedReadyCallback(const std_msgs::Bool::ConstPtr &msg) {
//        if(!this->all_ready) {
//            this->all_ready = msg->data;
//            ROS_INFO("vehicles are all armed");
//        }
//    }
    void switchCallback(const std_msgs::Empty::ConstPtr &msg) {
        this->switch_signal = true;
    }

    void rawPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {

        int id = stoi(msg->header.frame_id);
        curr_pos[id].position = msg->pose.position;
        this->start_first_cnt[id] = true;
    }

    void planReadyCallback(const std_msgs::Bool::ConstPtr &msg) {
        this->plan_ready = msg->data;
        if (this->plan_ready) {
            ROS_INFO("plan ready!");
        }
    }

    void run(){
        ros::Rate r(DEFAULT_RATE);
        bool stop = false;
        while (ros::ok()&&!stop) {
            int cnt_t = 0;
            for (int i = 0; i < this->robot_number; ++i) {
                if (this->start_first_cnt[i])
                    cnt_t++;
            }
            if (cnt_t == this->robot_number)
                stop = true;
            ros::spinOnce();
            r.sleep();
        }

        /* setup first time mission */
        ROS_INFO("start first time mission");
        // send out task to robots
        std_msgs::UInt8 init_task;
//        for (int j = 0; j < 4; ++j) {
        init_task.data = 1;
        for (int i = 0; i < this->active_number; ++i) {
            flight_task_pub[i].publish(init_task);
        }
        init_task.data = 0;
        flight_task_pub[this->robot_number-1].publish(init_task);
        ros::spinOnce();
        r.sleep();
//        }
        ROS_INFO("send out first time mission's task");

        // request coverage planning
        swarm_center::mCPPReq srv;
        for (int i = 0; i < this->active_number; ++i) {
            srv.request.x.push_back(curr_pos[i].position.x);
            srv.request.y.push_back(curr_pos[i].position.y);
        }
        plan_client.call(srv);

        while (ros::ok()&&!this->plan_ready) {
            ros::spinOnce();
            r.sleep();
        }
        this->plan_ready = false;

        for (int i = 0; i < this->active_number; ++i) {
            std_msgs::UInt8 task_id;
            task_id.data = i;
            flight_index_pub[i].publish(task_id);
        }
        ROS_INFO("begin main loop");
        /* main loop */
        while(ros::ok()){
            ros::spinOnce();
            r.sleep();
            if (this->switch_signal)//||!this->all_ready)
            {
                /* dispatch!!! */
                int up_id;
                for (int i = 0; i < this->robot_number; ++i) {
                    if (active_list[i] == false) {
                        up_id = i;
                        active_list[i] = true;
                    }
                }
                int down_id = up_id + 1;
                if (down_id == this->robot_number)
                    down_id = 0;
                active_list[down_id] = false;

                /* request coverage plan */
                // send out task to robots
                std_msgs::UInt8 continue_task;
                continue_task.data = 2; //start from middle
                for (int i = 0; i < this->robot_number; ++i) {
                    if (i==down_id||i==up_id)
                        continue;
                    flight_task_pub[i].publish(continue_task);
                }
                std_msgs::UInt8 return_task;
                return_task.data = 3; //return
                flight_task_pub[down_id].publish(return_task);
                std_msgs::UInt8 go_task;
                go_task.data = 4; //engage
                flight_task_pub[up_id].publish(go_task);

                // request coverage planning
                swarm_center::mCPPReq srv;
                for (int i = 0; i < this->robot_number; ++i) {
                    if (i==down_id)
                        continue;
                    if (i==up_id) {
                        srv.request.x.push_back(curr_pos[down_id].position.x);
                        srv.request.y.push_back(curr_pos[down_id].position.y);
                    } else {
                        srv.request.x.push_back(curr_pos[i].position.x);
                        srv.request.y.push_back(curr_pos[i].position.y);
                    }
                }
                plan_client.call(srv);

                // wait for plan ready signal
                while (ros::ok()&&!this->plan_ready) {
                    ros::spinOnce();
                    r.sleep();
                }
                this->plan_ready = false;

                // send task index
                int tmp = 0;
                for (int i = 0; i < this->robot_number; ++i) {
                    if (i==down_id) {
                        tmp++;
                        continue;
                    }
                    std_msgs::UInt8 task_id;
                    task_id.data = i-tmp;
                    flight_index_pub[i].publish(task_id);
                }
                this->switch_signal = false;
            }
        }
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "dispatch_center");

    DispatchCenter node;
    node.run();

    return 0;
}