/**
 * This is the commander for formation control trial
 * Date: 2019.3 Author: Weifan Zhang
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
//#include <geometry_msgs/Twist.h>

#define DEFAULT_RATE 20

//#define DEFAULT_RATE 50
#define GRID_SIZE 32
#define CORE_SIZE 16 //CORE_SIZE = GRID_SIZE / 2
#define AREA_SIZE 800
#define LEN_COF 200 // real world length * LEN_COF = Grid length

using namespace std;
using namespace cv;

class RecordCenter
{
private:
    int robot_number;
    string prefix = "swarmbot";

//    ros::NodeHandle global;
//    ros::NodeHandle local;

    vector<ros::Subscriber> raw_sub;
    vector<float> curr_pos_x;
    vector<float> curr_pos_y;

    Mat board = Mat(Size(900,900), CV_8UC3, Scalar(0));
    bool first_draw;
    vector<bool> all_ready;
    bool ready;

    /*Publish the setpoint*/
//    ros::Publisher cmd_sp_pub;

    /*Publish the velocity command*/
//    ros::Publisher cmd_vel_pub;

public:
    RecordCenter(){
        ROS_INFO("RECORD CENTER is online!");
        this->robot_number = 4;
        ros::NodeHandle node;

        raw_sub.resize(this->robot_number);
        for (int i = 0; i < this->robot_number; ++i) {
            char msg_name[50];
            sprintf(msg_name,"/%s%d/set_position",prefix.c_str(),i);
            raw_sub[i] = node.subscribe<geometry_msgs::Pose>(msg_name,10,&RecordCenter::rawCallback,this);
        }

//        curr_pos_x.resize(this->robot_number);
//        curr_pos_y.resize(this->robot_number);
        this->first_draw = true;
//        all_ready.resize(this->robot_number,false);
        this->ready = false;
    }

    void rawCallback(const geometry_msgs::Pose::ConstPtr &msg)
    {
        float pos_x = msg->position.x;
        float pos_y = msg->position.y;
//        int id = stoi(msg->header.frame_id);
//        all_ready[id] = true;

        curr_pos_x.push_back(pos_x*LEN_COF + AREA_SIZE/2);
        curr_pos_y.push_back(pos_y*LEN_COF + AREA_SIZE/2);
        this->ready = true;

    }

    void run(){
        ros::Rate r(DEFAULT_RATE);
        int cnt = 0;
        while(ros::ok()&&cnt<4000)
        {
            if (this->ready) {
//                int tmp = 0;
//                for (int k = 0; k < this->robot_number; ++k) {
//                    if (all_ready[k])
//                        tmp++;
//                }
//                if (tmp==this->robot_number)
//                    this->ready = true;
//            } else {
                cnt++;
                cout << cnt << endl;
            }
            ros::spinOnce();
            r.sleep();
        }

        cout << "finish record" << endl;
//        waitKey();
        int cnt_limit = curr_pos_x.size();
//        for (int k = 0; k < this->robot_number; ++k) {
//            if (cnt_limit<0||cnt_limit>curr_pos_x[k].size())
//                cnt_limit = curr_pos_x[k].size();
//        }
        int cnt_now = 0;
        while(ros::ok()&&cnt_now<=cnt_limit){
            display_area(cnt_now);
            cnt_now++;
//            ros::spinOnce();
//            r.sleep();
        }
    }

    void display_area(int ind){
        /*draw an empty area*/
//        ROS_INFO("debug 2");
        namedWindow("monitor");
        int stepSize = (int)800/GRID_SIZE;
        if (this->first_draw)
        {
            Point p1 = Point(50,50);
            Point p2 = Point(AREA_SIZE+50,AREA_SIZE+50);
            rectangle(board, p1, p2, CV_RGB(245, 245, 245), -1);
//            imshow("monitor", board);

            /*draw grids*/
//            ROS_INFO("debug 3");

            for (int i = stepSize; i < AREA_SIZE; i += stepSize) {
                line(board, Point(50,50+i),Point(AREA_SIZE+50,50+i),Scalar(0,0,0));
            }

            for (int i = stepSize; i < AREA_SIZE; i += stepSize) {
                line(board, Point(50+i,50),Point(50+i,AREA_SIZE+50),Scalar(0,0,0));
            }
//            ROS_INFO("draw grids complete");
            /*draw result*/
//            for (int i = 0; i < CORE_SIZE; i += 1) { //cols
//                for (int j = 0; j < CORE_SIZE; j += 1) { //rows
//                    if (K(j,i)==0)
//                        continue;
//                    Point t1 = Point(j*stepSize*2+50,i*stepSize*2+50);
//                    Point t2 = Point((j+1)*stepSize*2+50,(i+1)*stepSize*2+50);
//                    rectangle(board, t1, t2, CV_RGB(0,0,255), -1);
//                }
//            }
//            ROS_INFO("draw K complete");
            this->first_draw = false;
            imshow("monitor", board);
            waitKey();
        } else {

//        for (int k = 0; k < this->robot_number; ++k) {
            circle(board,Point(curr_pos_y[ind]+50,curr_pos_x[ind]+50),15,CV_RGB(100,100,255),-1);
//        }

//        ROS_INFO("size of board : %d, %d", board.size().height, board.size().width);
            imshow("monitor", board);
            waitKey(10);
        }



//        destroyWindow("monitor");
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "record_center");

    RecordCenter node;
    node.run();

    return 0;
}