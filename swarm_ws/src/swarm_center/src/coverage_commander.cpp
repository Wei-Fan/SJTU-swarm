/**
 * This is the commander for multi robot coverage trial
 * Function: 1)wait for planning request; 2)Keep track of every robot covering area
 * Date: 2019.3 Author: Weifan Zhang
 */


#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"

#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <sstream>
#include <fstream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include "swarm_center/mCPPReq.h"
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/Twist.h>

//#define DEFAULT_RATE 50
#define GRID_SIZE 24
#define CORE_SIZE 12 //CORE_SIZE = GRID_SIZE / 2
#define AREA_SIZE 800
#define LEN_COF 200 // real world length * LEN_COF = Grid length

using namespace std;
using namespace Eigen;
using namespace cv;

string project_path = "/home/wade/SJTU-swarm/swarm_ws/src/swarm_center/";
//float position_bias_x[4] = {0.5, -0.5, -0.5, 0.5};
//float position_bias_y[4] = {0.5, 0.5, -0.5, -0.5};
//int agents_number = 0;
//vector<int> x_init;
//vector<int> y_init;

class CoverageCommander
{
private:
    int robot_number;
    int active_number;
    string prefix = "swarmbot";

    ros::NodeHandle global;
    ros::NodeHandle local;

    ros::ServiceServer service;
    ros::Publisher ready_pub;
    vector<ros::Subscriber> raw_sub;

//    std::chrono::high_resolution_clock ::time_point recieve_time;

    Mat board = Mat(Size(900,900), CV_8UC3, Scalar(0));

    Matrix<int,CORE_SIZE,CORE_SIZE> K;
//    Matrix<int,GRID_SIZE,GRID_SIZE> E; How to define a 3d matrix?
//    MatrixXi E;
    Matrix<double,Dynamic,CORE_SIZE> C;
    VectorXi S; // record grid number for each robot
    vector<vector<Vector2i>> P; // record trajectory in the core map
    vector<vector<Vector2i>> P_grid; // planning result
    vector<Matrix<int,CORE_SIZE,CORE_SIZE>> T; // record the minimal spanning tree for each robot
    VectorXd m;
    vector<int> robot_core_x;
    vector<int> robot_core_y;
    vector<int> robot_grid_x;
    vector<int> robot_grid_y;
//    bool get_init_pos;
//    int get_init_cnt;
    vector<bool> get_init_bool;
    vector<float> robot_init_x;
    vector<float> robot_init_y;
    vector<float> robot_curr_x;
    vector<float> robot_curr_y;
    bool first_draw;


public:
    bool plan_ready;

    CoverageCommander(){
        ros::NodeHandle local("/coverage_commander");
        ros::NodeHandle global("");
        this->local = local;
        this->global = global;
        if(!this->local.getParam("robot_number", this->robot_number)){
            ROS_WARN("Did not set up robot number, using default 2");
            this->robot_number = 2;
        }
        this->active_number = this->robot_number;
        ROS_INFO("COVERAGE COMMANDER is activated! COMMANDER has %d robots to dispose", this->robot_number);
//        robot_init_x.resize(this->robot_number);
//        robot_init_y.resize(this->robot_number);

        service = global.advertiseService("/mCPP_req",&CoverageCommander::plan,this);

        /* respond only one time per 5s period */
//        recieve_time = std::chrono::high_resolution_clock::now();

        /* tell dispatch center that plan ready */
        ready_pub = global.advertise<std_msgs::Bool>("/plan_ready",1);

        /* obtain raw position for each time planning */
//        raw_sub.resize(this->robot_number);
//        for (int i = 0; i < this->robot_number; ++i) {
//            char msg_name[50];
//            sprintf(msg_name, "/%s%d/raw_position", this->prefix.c_str(), i);
//            raw_sub[i] = global.subscribe<geometry_msgs::PoseStamped>(msg_name, 1, &CoverageCommander::rawPosCallback, this);
//        }

//        robot_core_x.resize(this->robot_number);
//        robot_core_y.resize(this->robot_number);
//        robot_grid_x.resize(this->robot_number);
//        robot_grid_y.resize(this->robot_number);
//        get_init_bool.resize(this->robot_number, false);
//
//        robot_curr_x.resize(this->robot_number);
//        robot_curr_y.resize(this->robot_number);
        first_draw = true;
        plan_ready = false;
        /* initial matrix*/
        K.setZero(); //set zeros
    }

//    void rawPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
//
//        int id = stoi(msg->header.frame_id);
//
//        if (!get_init_bool[id])
//        {
//            robot_init_x[id] = msg->pose.position.x;
//            robot_init_y[id] = msg->pose.position.y;
//            get_init_bool[id] = true;
//            ROS_INFO("get robot %d 's init_pos : %f, %f",id,robot_init_x[id],robot_init_y[id]);
//        } else {
//            robot_curr_x[id] = msg->pose.position.x;
//            robot_curr_y[id] = msg->pose.position.y;
//        }
//    }

    bool plan(swarm_center::mCPPReq::Request &req,
              swarm_center::mCPPReq::Response &res)
//    void run()
    {
        ROS_INFO("request recieved!");
        this->active_number = req.x.size();
        robot_init_x.resize(this->robot_number);
        robot_init_y.resize(this->robot_number);
        robot_core_x.resize(this->active_number);
        robot_core_y.resize(this->active_number);
        robot_grid_x.resize(this->active_number);
        robot_grid_y.resize(this->active_number);
        for (int i = 0; i < this->active_number; ++i) {
            robot_init_x[i] = req.x[i];
            robot_init_y[i] = req.y[i];
        }
        /* respond only one time per period */
//        auto cur_time = std::chrono::high_resolution_clock::now();
//        std::chrono::duration<double> lock_time = cur_time-recieve_time;
//        if (lock_time.count() > 5.0) {
//            res.b = true;
//            recieve_time = cur_time;
//        } else {
//            ROS_WARN("not now");
//            res.b = false;
//            return false;
//        }

        /* check if init positions are obtained */
//        int get_init_cnt = 0;
//        while (get_init_cnt != this->active_number) {
//            get_init_cnt = 0;
//            for (int i = 0; i < this->active_number; ++i) {
//                if (get_init_bool[i]) {
//                    get_init_cnt++;
//                }
//            }
////            ROS_INFO("check init_position");
//            ros::spinOnce();
//        }
//        ROS_INFO("finish init_position collection");
//        robot_init_x = {0.5, -0.5, -0.5, 0.5};
//        robot_init_y = {0.5, 0.5, -0.5, -0.5};

        /*
        * initial phase
        * Area size: 40*40
        * */
//        display_area();

        /*initialize robot positions by clicking (testing code)*/
//        namedWindow("monitor");
//        imshow("monitor",board);
//        setMouseCallback("monitor", onMouse, &board);
//        waitKey();
//        ROS_INFO("robot number : %d", agents_number);
//        destroyWindow("monitor");
//        for (int i = 0; i < this->robot_number; ++i) {
//            robot_core_x.push_back((x_init[i] - 50)*CORE_SIZE/AREA_SIZE);
//            robot_core_y.push_back((y_init[i] - 50)*CORE_SIZE/AREA_SIZE);
//            robot_grid_x.push_back((x_init[i] - 50)*GRID_SIZE/AREA_SIZE);
//            robot_grid_y.push_back((y_init[i] - 50)*GRID_SIZE/AREA_SIZE);
//            ROS_INFO("robot : %d,%d",robot_core_x[i],robot_core_y[i]);
//        }
        for (int i = 0; i < this->active_number; ++i) {
            robot_core_x[i] = (robot_init_x[i]*LEN_COF+AREA_SIZE/2)*CORE_SIZE/AREA_SIZE;
            robot_core_y[i] = (robot_init_y[i]*LEN_COF+AREA_SIZE/2)*CORE_SIZE/AREA_SIZE;
            robot_grid_x[i] = (robot_init_x[i]*LEN_COF+AREA_SIZE/2)*GRID_SIZE/AREA_SIZE;
            robot_grid_y[i] = (robot_init_y[i]*LEN_COF+AREA_SIZE/2)*GRID_SIZE/AREA_SIZE;
            ROS_INFO("robot : %d,%d (core)",robot_core_x[i],robot_core_y[i]);
        }


        /*these codes need to be moved if the robot number comes from elsewhere*/
        C.setOnes(this->active_number*CORE_SIZE,CORE_SIZE);
        m.setOnes(this->active_number); //size : (agents_number,1)

        /*
         * divide area
         * */
        divide_area();

        /*
         * STC planning
         * */
        generate_tree();
        path_planning();

        /*
         * generate planning document
         * */
        generate_path();

        std_msgs::Bool ready_msg;
        ready_msg.data = true;
        for (int i = 0; i < 10; ++i) {
            ready_pub.publish(ready_msg);
            plan_ready = true;
        }
//        this->get_init_pos = false;
        ROS_INFO("send ready signal from coverage center");

        res.b = true;
        return true;
    }

    void display_area(){
        /*draw an empty area*/
//        ROS_INFO("debug 2");
        namedWindow("monitor");
        int stepSize = (int)800/GRID_SIZE;
        if (first_draw)
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
            for (int i = 0; i < CORE_SIZE; i += 1) { //cols
                for (int j = 0; j < CORE_SIZE; j += 1) { //rows
                    if (K(j,i)==0)
                        continue;
                    Point t1 = Point(j*stepSize*2+50,i*stepSize*2+50);
                    Point t2 = Point((j+1)*stepSize*2+50,(i+1)*stepSize*2+50);
                    rectangle(board, t1, t2, CV_RGB(0,0,255), -1);
                }
            }
//            ROS_INFO("draw K complete");
        }

        for (int k = 0; k < this->active_number; ++k) {
            int x_t = AREA_SIZE/2 - robot_curr_y[k]*LEN_COF + 50;
            int y_t = AREA_SIZE/2 + robot_curr_x[k]*LEN_COF + 50;
            circle(board,Point(x_t,y_t),10,CV_RGB(10*k,20*k,30*k),-1);
        }

//        ROS_INFO("size of board : %d, %d", board.size().height, board.size().width);
        imshow("monitor", board);
        waitKey();

//        destroyWindow("monitor");
    }

//    static void onMouse(int event, int x, int y, int, void* userInput)
//    {
//        if (event != EVENT_LBUTTONDOWN) return;
////        printf("###########onMouse x : %d\n", x);
////        printf("###########onMouse y : %d\n", y);
//        int x_world = x - 500;
//        int y_world = 500 - y;
//        Mat *img = (Mat*)userInput;
//
//        circle(*img, Point(x, y), 6, Scalar(255, 0, 255), 2);
//        imshow("monitor", *img);
//        x_init.push_back(x);//x_world);
//        y_init.push_back(y);//y_world);
//        agents_number++;
//
//    }

    bool divide_area(){
        /*Generate the first matrix*/
        S.setZero(this->active_number);
        for (int i = 0; i < CORE_SIZE; ++i) {
            for (int j = 0; j < CORE_SIZE; ++j) {
                double tmp = -1;
                for (int k = 0; k < this->active_number; ++k) {
//                        MatrixXi E_t = E.block<CORE_SIZE,CORE_SIZE>(k*CORE_SIZE,0);
//                        MatrixXi C_t = C.block<CORE_SIZE,CORE_SIZE>(k*CORE_SIZE,0);
                    double Ekji = C(j+k*CORE_SIZE,i) * m(k) * sqrt((i-robot_core_x[k])*(i-robot_core_x[k])+(j-robot_core_y[k])*(j-robot_core_y[k]));
//                        double E_t = ;
                    if (tmp < 0 || tmp > Ekji)
                    {
                        tmp = Ekji;
                        K(j,i) = k;
                    }
                }
                S(K(j,i))++;
            }
        }


        /*main loop*/
        bool stop = false;
        int iteration_count = 0;
        while (!stop&&ros::ok())
        {
            iteration_count++;

            S.setZero(this->active_number); // size : (agents_number,1)
            int count = 0;

            /*check every grid*/
//            ROS_INFO("E size : %d, %d",E.rows(),E.cols());
            for (int i = 0; i < CORE_SIZE; ++i) { //cols
                for (int j = 0; j < CORE_SIZE; ++j) { //rows
                    double tmp = -1;
                    for (int k = 0; k < this->active_number; ++k) {
//                        MatrixXi E_t = E.block<CORE_SIZE,CORE_SIZE>(k*CORE_SIZE,0);
//                        MatrixXi C_t = C.block<CORE_SIZE,CORE_SIZE>(k*CORE_SIZE,0);
                        double Ekji = C(j+k*CORE_SIZE,i) * m(k) * sqrt((i-robot_core_x[k])*(i-robot_core_x[k])+(j-robot_core_y[k])*(j-robot_core_y[k]));
//                        double E_t = ;
                        if (tmp < 0 || tmp > Ekji)
                        {
                            tmp = Ekji;
                            K(j,i) = k;
                        }
                    }
                    S(K(j,i))++;
                }
            }
//            cout << S <<  endl;
//            stop = true;

            /*update C*/
            // obtain the assignment matrix for every robot
            Matrix<int,Dynamic,CORE_SIZE> Kd;
            Kd.resize(this->active_number*CORE_SIZE,CORE_SIZE);
            for (int k = 0; k < this->active_number; ++k) {
                for (int i = 0; i < CORE_SIZE; ++i) {
                    for (int j = 0; j < CORE_SIZE; ++j) {
                        if (K(j,i)==k)
                            Kd(j+k*CORE_SIZE,i) = 1;
                        else
                            Kd(j+k*CORE_SIZE,i) = 0;

                    }
                }
            }
            // obtain the connected sets for every robot
//            cout << isConnect(1,1,K,1,2) << endl;
            vector<vector<Vector2i>> con_set;
            vector<vector<Vector2i>> dcon_set;
            for (int k = 0; k < this->active_number; ++k) {
                vector<Vector2i> con_t;
                vector<Vector2i> dcon_t;
                MatrixXi K_t = Kd.block<CORE_SIZE,CORE_SIZE>(k*CORE_SIZE,0);
//                cout << K_t << endl;
                for (int i = 0; i < CORE_SIZE; ++i) {
                    for (int j = 0; j < CORE_SIZE; ++j) {
                        if (K_t(j,i)!=1)
                            continue;

                        Vector2i c_t(j,i);
                        if (isConnect(j,i,K_t,robot_core_x[k],robot_core_y[k]))
                        {
//                            ROS_INFO("con : %d,%d",j,i);
                            con_t.push_back(c_t);
                        } else {
//                            ROS_INFO("dcon : %d,%d",j,i);
                            dcon_t.push_back(c_t);
                        }
                    }
                }
                con_set.push_back(con_t);
                dcon_set.push_back(dcon_t);
            }

            for (int k = 0; k < this->active_number; ++k) {
                vector<Vector2i> con_t = con_set[k];
                vector<Vector2i> dcon_t = dcon_set[k];
                MatrixXd C_t;
                if (con_t.size()==0||dcon_t.size()==0){
                    C_t.setOnes(CORE_SIZE,CORE_SIZE);
                    C.block<CORE_SIZE,CORE_SIZE>(k*CORE_SIZE,0) = C_t;
                } else {
                    for (int i = 0; i < CORE_SIZE; ++i) {
                        for (int j = 0; j < CORE_SIZE; ++j) {
                            double mdist_con = -1;
                            double mdist_dcon = -1;
                            for (int t = 0; t < con_t.size(); ++t) {
                                double m_t = sqrt((con_t[t](0)-j)*(con_t[t](0)-j)+(con_t[t](1)-i)*(con_t[t](1)-i));
                                if (mdist_con<0||mdist_con>m_t)
                                    mdist_con = m_t;
                            }
                            for (int t = 0; t < dcon_t.size(); ++t) {
                                double m_t = sqrt((dcon_t[t](0)-j)*(dcon_t[t](0)-j)+(dcon_t[t](1)-i)*(dcon_t[t](1)-i));
                                if (mdist_dcon<0||mdist_dcon>m_t)
                                    mdist_dcon = m_t;
                            }

                            if (mdist_con<0.002)
                                C(k*CORE_SIZE+j,i) = 1;
                            else if (mdist_dcon<0.002)
                                C(k*CORE_SIZE+j,i) = 1.3;
                            else
                                C(k*CORE_SIZE+j,i) = 0.3*mdist_con/(mdist_con+mdist_dcon)+1;
                        }
                    }
                }
            }

            /*recalcuate S*/
            for (int k = 0; k < this->active_number; ++k) {
                S(k) -= dcon_set[k].size();
            }

            /*update m*/
            double threshold = CORE_SIZE*CORE_SIZE/30.0;
            if (iteration_count>100)
                threshold = threshold * (iteration_count/50.0)*(iteration_count/50.0);
            for (int k = 0; k < this->active_number; ++k) {
                double dm = S(k) - CORE_SIZE*CORE_SIZE/this->active_number;
                if (dm>threshold || dm<(-1)*threshold)
                    m(k) += 0.002*dm;
                else
                    count++;
            }

            /*stop condition check*/
            bool stop1 = false;
            if (count==this->active_number)
                stop1 = true;

            bool stop2 = true;
            for (int k = 0; k < this->active_number; ++k) {
                if (dcon_set[k].size()!=0)
                {
                    stop2 = false;
                    break;
                }
            }

            if ((stop1 && stop2) || iteration_count==300){
                cout << "S : \n" << S << endl;
                stop = true;
            }
        }
    }

    bool isConnect(int j0, int i0, Matrix<int,CORE_SIZE,CORE_SIZE> K_t, int rx, int ry){
        int pace = -1;
        Matrix<double,4,2> coordinate;
        coordinate << 0, -1,
                1, 0,
                0, 1,
                -1, 0;
        Vector2i curr_p(j0,i0);// size : (2,1)
        Vector2i robot(ry,rx);
//        cout << curr_p << endl << robot << endl;

        bool stop = false;
        int count = 0;
        while (!stop&&ros::ok())
        {
            /*see if the condition were met*/
            Vector2i dir_i = robot-curr_p;
            double dir_l = sqrt(dir_i(0)*dir_i(0)+dir_i(1)*dir_i(1));
//            cout << dir_l << endl;
            if (dir_l<0.002)
            {
//                cout << dir_i << endl;
//                ROS_INFO("(%d,%d) -> (%d,%d) start at stop",j0,i0,ry,rx);
                return true;
            }

            /*sort the direction*/
            double dir_d[2];
            dir_d[0] = dir_i[0] / dir_l;
            dir_d[1] = dir_i[1] / dir_l;
            double dir_cost[4];
            int cost_i[4] = {0,1,2,3};
            for (int t = 0; t < 4; ++t) {
                dir_cost[t] = sqrt((coordinate(t,0)-dir_d[0])*(coordinate(t,0)-dir_d[0])+(coordinate(t,1)-dir_d[1])*(coordinate(t,1)-dir_d[1]));
            }
            if (pace>=0)
            {
                dir_cost[pace] = dir_cost[pace] + 2;
            }
            for (int j = 0; j < 4 - 1; ++j) {
                for (int i = 0; i < 4 - 1 - j; ++i) {
                    if (dir_cost[i] > dir_cost[i+1]){
                        Swap(dir_cost,i,i+1);
                        Swap(cost_i,i,i+1);
                    }
                }
            }
            for (int i = 0; i < 4; ++i) {
//                cout << "sorted dir_cost : " << dir_cost[i] << endl;
            }
            vector<Vector2i> sorted_cd;
            for (int i = 0; i < 4; ++i) {
                Vector2i tmp(coordinate(cost_i[i],0),coordinate(cost_i[i],1));
                sorted_cd.push_back(tmp);
            }

            /*choose the nearest apprachable way*/
            bool move = false;
            for (auto sd : sorted_cd)
            {
//                cout << sd << endl;
                Vector2i next_p = curr_p + sd;
                if (next_p(0)==-1||next_p(0)==CORE_SIZE||next_p(1)==-1||next_p(1)==CORE_SIZE)
                    continue;

                if (K_t(next_p(0),next_p(1))==1)
                {
                    curr_p = next_p;
                    move = true;
                    if (sd(0)==1)
                        pace = 3;
                    else if (sd(0)==-1)
                        pace = 1;
                    else if (sd(1)==-1)
                        pace = 2;
                    else
                        pace = 0;
                    break;
                }
            }
            if (!move)
            {
                return false;
            } else {
                count++;
                if (count==CORE_SIZE*CORE_SIZE)
                {
                    return false;
                }
            }
        }

    }

    template<typename T>
    void Swap(T A[], int i, int j)
    {
        T tmp = A[i];
        A[i] = A[j];
        A[j] = tmp;
    }


    void generate_tree(){
        /*prepare input*/
        // obtain the assignment matrix for every robot
        Matrix<int,Dynamic,CORE_SIZE> Kd;
        Kd.resize(this->active_number*CORE_SIZE,CORE_SIZE);
        for (int k = 0; k < this->active_number; ++k) {
            for (int i = 0; i < CORE_SIZE; ++i) {
                for (int j = 0; j < CORE_SIZE; ++j) {
                    if (K(j,i)==k)
                        Kd(j+k*CORE_SIZE,i) = 1;
                    else
                        Kd(j+k*CORE_SIZE,i) = 0;

                }
            }
        }

        P.clear();

        /*prepare output*/
        vector<Vector2i> direction;
        Vector2i dir_t(0,1);
        direction.push_back(dir_t);
        dir_t(0) = 1;dir_t(1) = 0;
        direction.push_back(dir_t);
        dir_t(0) = 0;dir_t(1) = -1;
        direction.push_back(dir_t);
        dir_t(0) = -1;dir_t(1) = 0;
        direction.push_back(dir_t);

        /*main proccess*/
        for (int k = 0; k < this->active_number; ++k) {


            /*neccessary preparation*/
            Vector2i start_p(robot_core_y[k], robot_core_x[k]);
            vector<Vector2i> P_t;
            P_t.push_back(start_p);

            MatrixXi K_t = Kd.block<CORE_SIZE,CORE_SIZE>(k*CORE_SIZE,0);
            int total_num = S(k);
            Matrix<int,CORE_SIZE,CORE_SIZE> T_t;
            int num = 1;
            T_t.setZero();
            T_t(start_p(0),start_p(1)) = num;
            Vector2i curr_p(0,0);

            /*first move*/
            for (auto d : direction){
                Vector2i tem_p = start_p + d;
                if (tem_p(0)==-1||tem_p(0)==CORE_SIZE||tem_p(1)==-1||tem_p(1)==CORE_SIZE)
                    continue;
                if (K_t(tem_p(0),tem_p(1))==1)
                {
                    P_t.push_back(tem_p);
                    T_t(tem_p(0),tem_p(1)) = T_t(start_p(0),start_p(1))+1;
                    curr_p = tem_p;
                    num++;
                    break;
                }
            }

            /*auto generate*/
            while (num!=total_num&&ros::ok())
            {
                /*move to free space*/
                bool move = false;
                for (auto d : direction){
                    Vector2i tem_p = curr_p + d;
                    if (tem_p(0)==-1||tem_p(0)==CORE_SIZE||tem_p(1)==-1||tem_p(1)==CORE_SIZE)
                        continue;
                    if (K_t(tem_p(0),tem_p(1))==1 && T_t(tem_p(0),tem_p(1))==0)
                    {
                        P_t.push_back(tem_p);
                        T_t(tem_p(0),tem_p(1)) = T_t(curr_p(0),curr_p(1))+1;
                        curr_p = tem_p;
                        num++;
                        move = true;
                        break;
                    }
                }

                /*back to an old cell*/
                if (move)
                    continue;

                for (auto d : direction){
                    Vector2i tem_p = curr_p + d;
                    if (tem_p(0)==-1||tem_p(0)==CORE_SIZE||tem_p(1)==-1||tem_p(1)==CORE_SIZE)
                        continue;
                    if (K_t(tem_p(0),tem_p(1))==1 && T_t(tem_p(0),tem_p(1))==T_t(curr_p(0),curr_p(1))-1)
                    {
                        curr_p = tem_p;
                        break;
                    }
                }
            }
            T.push_back(T_t);
            P.push_back(P_t);
        }
    }

    void path_planning(){
        for (int k = 0; k < this->active_number; ++k) {
            /*clockwise planning method*/
            Matrix<int,CORE_SIZE,CORE_SIZE> T_t = T[k]; // load spanning tree

//            cout << "T"<<k<<" : "<<endl<<T_t<<endl<<endl;
            vector<Vector2i> P_t = P[k]; // load spanning tree
            vector<Vector2i> P_grid_t;

            Vector2i curr_core = P_t[0];

            Vector2i curr_grid(robot_grid_y[k],robot_grid_x[k]);
            P_grid_t.push_back(curr_grid);

            bool stop = false;//false
            while (!stop&&ros::ok()) {
                Vector2i next_grid;
                int grid_where = where_am_I(curr_grid);
                /*cross or not? cross: move to next core; not cross: continue*/
                bool cross = false;
                switch(grid_where){
                    case 1 : {
                        if (curr_core(1)-1 == -1) {
                            cross = false;
                            break;
                        }
                        int in_t = T_t(curr_core(0), curr_core(1)) - T_t(curr_core(0), curr_core(1)-1);
                        if (abs(in_t) == 1) {
                            cross = true;
                            curr_core(1) -= 1;
                        }
                        break;
                    }
                    case 2 : {
                        if (curr_core(0)+1 == CORE_SIZE) {
                            cross = false;
                            break;
                        }
                        int in_t = T_t(curr_core(0), curr_core(1)) - T_t(curr_core(0)+1, curr_core(1));
                        if (abs(in_t) == 1) {
                            cross = true;
                            curr_core(0) += 1;
                        }
                        break;
                    }
                    case 3 : {
                        if (curr_core(1)+1 == CORE_SIZE) {
                            cross = false;
                            break;
                        }
                        int in_t = T_t(curr_core(0), curr_core(1)) - T_t(curr_core(0), curr_core(1)+1);
                        if (abs(in_t) == 1) {
                            cross = true;
                            curr_core(1) += 1;
                        }
                        break;
                    }
                    case 4 : {
                        if (curr_core(0)-1 == -1) {
                            cross = false;
                            break;
                        }
                        int in_t = T_t(curr_core(0), curr_core(1)) - T_t(curr_core(0)-1, curr_core(1));
                        if (abs(in_t) == 1) {
                            cross = true;
                            curr_core(0) -= 1;
                        }
                        break;
                    }
                }

                if (!cross) {
                    next_grid = move_to_next_grid(curr_grid);
                    curr_grid = next_grid;
                } else {
                    next_grid = move_to_next_core(curr_grid);
                    curr_grid = next_grid;
                }

                P_grid_t.push_back(curr_grid);

                /*complete condition*/
                if (next_grid(0)==robot_grid_y[k]&&next_grid(1)==robot_grid_x[k])
                    stop = true;

//                cout << "curr_core : "<< curr_core(0) << "," << curr_core(1) << " ---- curr_grid : " << curr_grid(0) << "," << curr_grid(1) << endl;

            }

            P_grid.push_back(P_grid_t);
        }
    }

    int where_am_I(Vector2i curr){
        if (curr(0)%2==0){
            if (curr(1)%2==0){
                return 1;
            } else {
                return 4;
            }
        } else {
            if (curr(1)%2==0){
                return 2;
            } else {
                return 3;
            }
        }
    }

    Vector2i move_to_next_grid(Vector2i curr){
        Vector2i next = curr;
        if (curr(0)%2==0){
            if (curr(1)%2==0){
                next(0)+=1;
            } else {
                next(1)-=1;
            }
        } else {
            if (curr(1)%2==0){
                next(1)+=1;
            } else {
                next(0)-=1;
            }
        }
        return next;
    }

    Vector2i move_to_next_core(Vector2i curr){
        Vector2i next = curr;
        if (curr(0)%2==0){
            if (curr(1)%2==0){
                next(1)-=1;
            } else {
                next(0)-=1;
            }
        } else {
            if (curr(1)%2==0){
                next(0)+=1;
            } else {
                next(1)+=1;
            }
        }
        return next;
    }

    void generate_path(){
        for (int k = 0; k < this->active_number; ++k) {
            /* path name */
            vector<Vector2i> P_grid_t = P_grid[k];
            string filename = project_path + "launch/cover_robot" + to_string(k) + ".csv";

            /* velocity condition*/
            double vmax = 1.2;
            double amax = 1;
            double s_c = vmax*vmax/amax*LEN_COF*GRID_SIZE/AREA_SIZE;
            double dt = 0.05;//s
            double turn_blank = 8;

            /* write csv file */
            ofstream outfile;
            outfile.open(filename,ios::out);

            /* init pos to first grid center */
            float len_x = (P_grid_t[0](1)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF - robot_init_x[k];
            float len_y = (P_grid_t[0](0)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF - robot_init_y[k];
            for (int i = 1; i <= 10; ++i) {
                outfile << robot_init_x[k]+len_x*i/10 << ',' << robot_init_y[k]+len_y*i/10 << ',' << 1.5 << endl;
            }

            for (int i = 0; i < P_grid_t.size()-1; ++i) {
                /* FIND NEXT TURN POINT*/
                int xoy;
                double s;
                if (P_grid_t[i](0)==P_grid_t[i+1](0)) {
                    xoy = 1;
                } else {
                    xoy = 0;
                }
                bool find_next = false;
                int ind = i+2;
                while (!find_next) {
                    if (xoy==1) {
                        if (P_grid_t[ind](0)==P_grid_t[i](0))
                            ind++;
                        else {
                            ind--;
                            find_next = true;
                        }
                    } else {
                        if (P_grid_t[ind](1)==P_grid_t[i](1))
                            ind++;
                        else {
                            ind--;
                            find_next = true;
                        }
                    }
                }

                s = ind - i;


                for (int j = 0; j < turn_blank/2; ++j) {
                    outfile << (P_grid_t[i](1)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << (P_grid_t[i](0)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << 1.5 << endl;
                }

                if (s > s_c) {
                    double x = 0;
                    double dx = 0.5*amax*dt*dt;
                    double line_time = 2*sqrt(s_c/amax)+(s-s_c)/vmax;
                    int line_num = (int) (line_time / dt);
                    for (int j = 0; j < line_num; ++j) {
                        if (x<s_c/2)
                        {
                            dx += amax*dt*dt;
                            x += dx;
                        } else if (x<s-s_c/2) {
                            x += vmax*dt;
                        } else {
                            dx -= amax*dt*dt;
                            x += dx;
                        }
                        if (xoy==1){
                            if (P_grid_t[i](1)<P_grid_t[i+1](1))
                                outfile << (x+P_grid_t[i](1)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << (P_grid_t[i](0)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << 1.5 << endl;
                            else
                                outfile << (-x+P_grid_t[i](1)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << (P_grid_t[i](0)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << 1.5 << endl;
                        } else{
                            if (P_grid_t[i](0)<P_grid_t[i+1](0))
                                outfile << (P_grid_t[i](1)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << (x+P_grid_t[i](0)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << 1.5 << endl;
                            else
                                outfile << (P_grid_t[i](1)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << (-x+P_grid_t[i](0)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << 1.5 << endl;
                        }
                    }

                } else {
                    double line_time = 2*sqrt(s/amax);
                    int line_num = (int) (line_time / dt);
                    double x = 0;
                    double dx = 0.5*amax*dt*dt;
                    for (int j = 0; j < line_num; ++j) {
                        if (j<line_num/2) {
                            dx += amax*dt*dt;
                            x += dx;
                        } else {
                            dx -= amax*dt*dt;
                            x += dx;
                        }
                        if (xoy==1) {
                            if (P_grid_t[i](1)<P_grid_t[i+1](1))
                                outfile << (x+P_grid_t[i](1)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << (P_grid_t[i](0)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << 1.5 << endl;
                            else
                                outfile << (-x+P_grid_t[i](1)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << (P_grid_t[i](0)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << 1.5 << endl;
                        } else {
                            if (P_grid_t[i](0)<P_grid_t[i+1](0))
                                outfile << (P_grid_t[i](1)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << (x+P_grid_t[i](0)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << 1.5 << endl;
                            else
                                outfile << (P_grid_t[i](1)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << (-x+P_grid_t[i](0)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << 1.5 << endl;
                        }
                    }
                }

                i = ind - 1;
                for (int j = 0; j < turn_blank/2; ++j) {
                    outfile << (P_grid_t[i+1](1)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << (P_grid_t[i+1](0)-GRID_SIZE/2+0.5)*AREA_SIZE/GRID_SIZE/LEN_COF << ',' << 1 << endl;
                }
            }
        }

    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "coverage_commander");
    ros::Time::init();
    ros::Rate r(5);

    CoverageCommander node;
//    node.run();
    while (ros::ok()) {
//        if (node.plan_ready) {
//            node.display_area();
//        }
        ros::spin();
        r.sleep();
    }
    return 0;
}