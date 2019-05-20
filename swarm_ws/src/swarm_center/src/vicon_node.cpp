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

vector<float> x_marker_init;
vector<float> y_marker_init;
vector<float> x_init_pos;
vector<float> y_init_pos;
vector<int> index_sequence;

vector<float> yaw_manuel;
float AMP_COEFF;

void give_index(int index)
{
    index_sequence.push_back(index);
}
void clear_index()
{
    index_sequence.clear();
}


class ViconNode
{
private:
    int robot_number;

//    ros::NodeHandle global;
//    ros::NodeHandle local;
    vector<ros::Publisher> raw_pub;
    ros::Subscriber vicon_sub;
    vector<vicon_bridge::Marker> m_markers;
    bool isFirstVicon;

    float z_ground;
    vector<geometry_msgs::Pose> curr_pos;
    vector<geometry_msgs::Pose> swarm_pos_predict;
    vector<geometry_msgs::Pose> swarm_pos_err;
    vector<geometry_msgs::Pose> swarm_pos_step;
    ros::Time m_last_time_vicon;
    ros::Time m_this_time_vicon;


    Mat background = Mat(Size(1000,1000),CV_8UC3,Scalar(0));

public:
    ViconNode(ros::NodeHandle& nh)
    :isFirstVicon(true)
    ,z_ground(0.0)
    {
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
        m_markers = msg->markers;

        if(isFirstVicon && msg->markers.size() != 0)
        {
            int cnt_tmp = 0;
            for (auto& Marker : m_markers)
            {
                float pos[3];
                pos[0] = Marker.translation.x/1000.0f;
                pos[1] = Marker.translation.y/1000.0f;
                pos[2] = Marker.translation.z/1000.0f;

                x_marker_init.push_back(pos(0));
                y_marker_init.push_back(pos(1));
                if (pos[2] < 0.2)
                {
                    z_ground += pos[2];
                    cnt_tmp++;
                }
            }
            z_ground = z_ground / cnt_tmp;


            unite(x_init_pos,y_init_pos,x_marker_init,y_marker_init);//identify miniflies and get their position into swarm_pos
            bool sequenceIsOk = false;
            while(!sequenceIsOk && ros::ok())//use mouse to rearrange index of swarm_pos
            {
                displayFunc();
                setMouseCallback("vicon_test", onMouse, &background);
                waitKey();
                destroyWindow("vicon_test");

                /*check the click times and exit the initialization*/
                if(index_sequence.size()==this->robot_number){
                    sequenceIsOk = true;
                }else{
                    printf("Initialization fails!! Please click again!!\n");
                    clear_index();
                }

            }

            for (int i=0; i<index_sequence.size();i++)
            {
                geometry_msgs::Pose tmp;
                tmp.position.x = x_init_pos[index_sequence[i]];
                tmp.position.y = y_init_pos[index_sequence[i]];
                tmp.position.z = z_ground;
                curr_pos.push_back(tmp);

                geometry_msgs::Pose tmp_zero;
                tmp_zero.position.x = 0.0;
                tmp_zero.position.y = 0.0;
                tmp_zero.position.z = 0.0;
                swarm_pos_step.push_back(tmp_zero);
                swarm_pos_predict.push_back(tmp_zero);
                swarm_pos_err.push_back(tmp_zero);
            }

            if(curr_pos.size()==this->robot_number)
                isFirstVicon = false;
        }
        else if(!isFirstVicon && msg->markers.size() != 0)
        {
            std::vector<Vector3f> consider_pos;//markers_pos;
            for (auto& Marker : m_markers)
            {
                Vector3f pos;
                pos(0) = Marker.translation.x/1000.0f;
                pos(1) = Marker.translation.y/1000.0f;
                pos(2) = Marker.translation.z/1000.0f;
                consider_pos.push_back(pos);
            }

            /*grand wipe out*/
            /*std::vector<Vector3f> consider_pos;
            for (int i=0;i<markers_pos.size();i++)
            {
                float norm;
                vec3f_norm(&markers_pos[i], &norm);
                bool isInside = true;
                for (int j=0;j<swarm_pos.size();j++)
                {
                    float swarm_norm;
                    vec3f_norm(&swarm_pos[j], &swarm_norm);

                    if(fabs(norm-swarm_norm) > RADIUS_SQUARE) //max circle
                        isInside = false;
                }
                if (isInside)
                    consider_pos.push_back(markers_pos[i]);
            }*/
            //printf("*******consider_pos size : %d\n", consider_pos.size());
            /*find vehicles*/
            for (int i = 0; i < g_vehicle_num; ++i)//for every vehicles
            {
                /*prediction*/
                swarm_pos_predict[i] = swarm_pos[i] + swarm_pos_step[i] + REVISE_WEIGHT*swarm_pos_err[i];
                /*small wipe out*/
                //printf("swarm_pos_predict %d: %f %f %f\n",i, swarm_pos_predict[i](0), swarm_pos_predict[i](1), swarm_pos_predict[i](2));
                std::vector<Vector3f> close_points;
                for (int j = 0; j < consider_pos.size(); ++j)//for every considered points
                {
                    Vector3f tmp_diff;
                    float tmp_norm;
                    tmp_diff(0) = consider_pos[j](0) - swarm_pos_predict[i](0);
                    tmp_diff(1) = consider_pos[j](1) - swarm_pos_predict[i](1);
                    tmp_diff(2) = consider_pos[j](2) - swarm_pos_predict[i](2);
                    vec3f_norm(&tmp_diff, &tmp_norm);

                    //printf("consider_pos: %f %f %f\n", consider_pos[j](0), consider_pos[j](1), consider_pos[j](2));
                    //printf("consider_pos: %f %f %f\n", tmp_diff(0), tmp_diff(1), tmp_diff(2));
                    //printf("*****inside radius : %f\n", tmp_norm);
                    if (tmp_norm < VEHICLE_SIZE)
                        close_points.push_back(consider_pos[j]);
                }//j
                //printf("**********close_points : %d\n", close_points.size());
                /*condition 1 to 4*/
                if (close_points.size() >= 4 && close_points.size() <= 4*g_vehicle_num) //condition 4
                {
                    bool FoundVehicle_i = false;
                    /*find vehicle center from close_points*/
                    for (int j = 0; j < close_points.size(); ++j)
                    {
                        //printf("********** j : %d\n", j);
                        //printf("FoundVehicle_i : %d\n", FoundVehicle_i);
                        if (FoundVehicle_i)
                            break;
                        /*record all the vectors that based on points j*/
                        std::vector<Vector3f> consider_vec;
                        for (int k = 0; k < close_points.size(); ++k)
                        {
                            if (k != j)
                            {
                                Vector3f tmp_vec;
                                tmp_vec(0) = close_points[k](0) - close_points[j](0);
                                tmp_vec(1) = close_points[k](1) - close_points[j](1);
                                tmp_vec(2) = close_points[k](2) - close_points[j](2);
                                consider_vec.push_back(tmp_vec);
                            }
                        }
                        /*count the number of right pairs*/
                        for (int p = 0; p < consider_vec.size(); ++p)
                        {
                            //printf("********** p : %d\n", p);
                            std::vector<Vector3f> swarm_pos_p;
                            int count_p = 0;
                            float len_p;
                            vec3f_norm(&consider_vec[p], &len_p);
                            for (int q = 0; q < consider_vec.size(); ++q)
                            {
                                if (q != p)
                                {
                                    //printf("********** q : %d\n", q);
                                    float len_q;
                                    vec3f_norm(&consider_vec[q], &len_q);
                                    float ctheta = (consider_vec[p](0)*consider_vec[q](0)+consider_vec[p](1)*consider_vec[q](1)+consider_vec[p](2)*consider_vec[q](2))/(len_p*len_q);
                                    if (ctheta < 0.2 && len_q/len_p < 1.05 && len_q/len_p > 0.95)
                                    {
                                        //printf("condition 1\n");
                                        Vector3f tmp_pos;
                                        tmp_pos(0) = 0.5*(consider_vec[p](0) + consider_vec[q](0)) + close_points[j](0);
                                        tmp_pos(1) = 0.5*(consider_vec[p](1) + consider_vec[q](1)) + close_points[j](1);
                                        tmp_pos(2) = 0.5*(consider_vec[p](2) + consider_vec[q](2)) + close_points[j](2);
                                        swarm_pos_p.push_back(tmp_pos);
                                        count_p++;
                                    } else if (ctheta < 0.75 && ctheta > 0.65 && len_q/len_p < 1.45 && len_q/len_p > 1.35)
                                    {
                                        //printf("condition 2\n");
                                        Vector3f tmp_pos;
                                        tmp_pos(0) = 0.5*consider_vec[q](0) + close_points[j](0);
                                        tmp_pos(1) = 0.5*consider_vec[q](1) + close_points[j](1);
                                        tmp_pos(2) = 0.5*consider_vec[q](2) + close_points[j](2);
                                        swarm_pos_p.push_back(tmp_pos);
                                        count_p++;
                                    } else if (ctheta < 0.75 && ctheta > 0.65 && len_p/len_q < 1.45 && len_p/len_q > 1.35)
                                    {
                                        //printf("condition 3\n");
                                        Vector3f tmp_pos;
                                        tmp_pos(0) = 0.5*consider_vec[p](0) + close_points[j](0);
                                        tmp_pos(1) = 0.5*consider_vec[p](1) + close_points[j](1);
                                        tmp_pos(2) = 0.5*consider_vec[p](2) + close_points[j](2);
                                        swarm_pos_p.push_back(tmp_pos);
                                        count_p++;
                                    }
                                }//if
                            }//for q
                            if (count_p == 2)
                            {
                                m_swarm_pos[i](0) = (swarm_pos_p[0](0) + swarm_pos_p[1](0))/2;
                                m_swarm_pos[i](1) = (swarm_pos_p[0](1) + swarm_pos_p[1](1))/2;
                                m_swarm_pos[i](2) = (swarm_pos_p[0](2) + swarm_pos_p[1](2))/2;
                                FoundVehicle_i = true;
                                break;
                            } else if (count_p == 1)
                            {
                                m_swarm_pos[i] = swarm_pos_p[0];
                                FoundVehicle_i = true;
                                break;
                            }
                        }//for p
                    }//for j
                    if (!FoundVehicle_i)
                    {
                        printf("*****condition 4 failed! failure number : 1\n");
                    }
                }else if (close_points.size() == 3) //condition 3
                {
                    //printf("*****condition 3\n");
                    Vector3f tmp_vec_1;
                    float tmp_len_1;
                    tmp_vec_1(0) = close_points[1](0) - close_points[0](0);
                    tmp_vec_1(1) = close_points[1](1) - close_points[0](1);
                    tmp_vec_1(2) = close_points[1](2) - close_points[0](2);
                    vec3f_norm(&tmp_vec_1, &tmp_len_1);

                    Vector3f tmp_vec_2;
                    float tmp_len_2;
                    tmp_vec_2(0) = close_points[2](0) - close_points[0](0);
                    tmp_vec_2(1) = close_points[2](1) - close_points[0](1);
                    tmp_vec_2(2) = close_points[2](2) - close_points[0](2);
                    vec3f_norm(&tmp_vec_2, &tmp_len_2);

                    float ctheta = (tmp_vec_1(0)*tmp_vec_2(0)+tmp_vec_1(1)*tmp_vec_2(1)+tmp_vec_1(2)*tmp_vec_2(2))/(tmp_len_1*tmp_len_2);
                    if (ctheta < 0.75 && ctheta > 0.65)
                    {
                        if (tmp_len_2/tmp_len_1 < 1.5 && tmp_len_2/tmp_len_1 > 1.3)
                        {
                            Vector3f tmp;
                            tmp(0) = 0.5*(close_points[2](0) + close_points[0](0));
                            tmp(1) = 0.5*(close_points[2](1) + close_points[0](1));
                            tmp(2) = 0.5*(close_points[2](2) + close_points[0](2));
                            m_swarm_pos[i] = tmp;
                        } else if (tmp_len_1/tmp_len_2 < 1.5 && tmp_len_1/tmp_len_2 > 1.3)
                        {
                            Vector3f tmp;
                            tmp(0) = 0.5*(close_points[1](0) + close_points[0](0));
                            tmp(1) = 0.5*(close_points[1](1) + close_points[0](1));
                            tmp(2) = 0.5*(close_points[1](2) + close_points[0](2));
                            m_swarm_pos[i] = tmp;
                        } else{
                            printf("*****condition 3 failed! failure number : 1\n");
                        }
                    } else if (ctheta < 0.1 && fabs(tmp_len_2-tmp_len_1)<0.2)
                    {
                        Vector3f tmp;
                        tmp(0) = 0.5*(close_points[2](0) + close_points[1](0));
                        tmp(1) = 0.5*(close_points[2](1) + close_points[1](1));
                        tmp(2) = 0.5*(close_points[2](2) + close_points[1](2));
                        m_swarm_pos[i] = tmp;
                    } else {
                        printf("*****condition 3 failed! failure number : 2\n");
                    }
                } else if (close_points.size() == 2) //condition 2
                {
                    //printf("*****condition 2\n");
                    Vector3f tmp_vec;
                    float tmp_len;
                    tmp_vec(0) = close_points[1](0) - close_points[0](0);
                    tmp_vec(1) = close_points[1](1) - close_points[0](1);
                    tmp_vec(2) = close_points[1](2) - close_points[0](2);
                    vec3f_norm(&tmp_vec, &tmp_len);

                    if (tmp_len > VEHICLE_EDGE_THRESHOLD && tmp_len < VEHICLE_EDGE_THRESHOLD+0.02)
                    {
                        Vector3f tmp;
                        tmp(0) = 0.5*(close_points[1](0) + close_points[0](0));
                        tmp(1) = 0.5*(close_points[1](1) + close_points[0](1));
                        tmp(2) = 0.5*(close_points[1](2) + close_points[0](2));
                        m_swarm_pos[i] = tmp;
                    } else if (tmp_len < VEHICLE_EDGE_THRESHOLD && tmp_len > VEHICLE_EDGE_THRESHOLD-0.02)
                    {
                        Vector3f center_pos;
                        center_pos(0) = 0.5*(close_points[1](0) + close_points[0](0));
                        center_pos(1) = 0.5*(close_points[1](1) + close_points[0](1));
                        center_pos(2) = 0.5*(close_points[1](2) + close_points[0](2));
                        Vector3f predict_diff;
                        predict_diff(0) = swarm_pos_predict[i](0) - center_pos(0);
                        predict_diff(1) = swarm_pos_predict[i](1) - center_pos(1);
                        predict_diff(2) = swarm_pos_predict[i](2) - center_pos(2);
                        float tmp_dist;
                        vec3f_norm(&predict_diff, &tmp_dist);

                        float ratio = 0.035/tmp_dist;
                        Vector3f tmp;
                        tmp(0) = (1-ratio)*center_pos(0) + ratio*swarm_pos_predict[0](0);
                        tmp(1) = (1-ratio)*center_pos(1) + ratio*swarm_pos_predict[0](1);
                        tmp(2) = (1-ratio)*center_pos(2) + ratio*swarm_pos_predict[0](2);
                        m_swarm_pos[i] = tmp;
                    } else {
                        printf("*****condition 2 failed! failure number : 0\n");
                    }

                } else if (close_points.size() == 1)
                {
                    //printf("*****condition 1\n");
                    Vector3f predict_diff;
                    predict_diff(0) = swarm_pos_predict[i](0) - close_points[0](0);
                    predict_diff(1) = swarm_pos_predict[i](1) - close_points[0](1);
                    predict_diff(2) = swarm_pos_predict[i](2) - close_points[0](2);
                    float tmp_dist;
                    vec3f_norm(&predict_diff, &tmp_dist);

                    float ratio = 0.035/tmp_dist;
                    Vector3f tmp;
                    tmp(0) = (1-ratio)*close_points[0](0) + ratio*swarm_pos_predict[0](0);
                    tmp(1) = (1-ratio)*close_points[0](1) + ratio*swarm_pos_predict[0](1);
                    tmp(2) = (1-ratio)*close_points[0](2) + ratio*swarm_pos_predict[0](2);
                    m_swarm_pos[i] = tmp;
                } else {
                    printf("*****Cannot find vehicle%d\n", i);
                    continue;
                }

                /*renew error*/
                swarm_pos_err[i](0) = m_swarm_pos[i](0) - swarm_pos_predict[i](0);
                swarm_pos_err[i](1) = m_swarm_pos[i](1) - swarm_pos_predict[i](1);
                swarm_pos_err[i](2) = m_swarm_pos[i](2) - swarm_pos_predict[i](2);

            }//i

            /*renew and publish*/
            for (int i = 0; i < swarm_pos.size(); ++i)
            {
                swarm_pos_step[i](0) = m_swarm_pos[i](0) - swarm_pos[i](0);
                swarm_pos_step[i](1) = m_swarm_pos[i](1) - swarm_pos[i](1);
                swarm_pos_step[i](2) = m_swarm_pos[i](2) - swarm_pos[i](2);

                swarm_pos[i] = m_swarm_pos[i];
                m_pos_estmsg.pos_est.x = swarm_pos[i](0);
                m_pos_estmsg.pos_est.y = swarm_pos[i](1);
                m_pos_estmsg.pos_est.z = swarm_pos[i](2);
                m_pos_estmsg.vehicle_index = i;
                m_pos_est_v[i].publish(m_pos_estmsg);

                //printf("*****vehicle%d: %f  %f  %f\n", i, swarm_pos[i](0), swarm_pos[i](1), swarm_pos[i](2));
            }
            if(isHovering)
            {
                for(int i=0;i<g_vehicle_num;++i)
                {
                    m_hover_pos[i](0) = swarm_pos[i](0);
                    m_hover_pos[i](1) = swarm_pos[i](1);
                    m_hover_pos[i](2) = swarm_pos[i](2);
                }
                isHovering = false;
            }
        }// else if
    }

    void displayFunc()
    {
        float tmp_max = 0;
        for (int i = 0; i < x_init_pos.size(); ++i)
        {
            float tmp = sqrt(x_init_pos[i]*x_init_pos[i]+y_init_pos[i]*y_init_pos[i]);
            if (tmp_max < tmp)
                tmp_max = tmp;
        }
        AMP_COEFF = 400.0f/tmp_max;
        printf("tmp_max : %f***********AMP_COEFF : %f\n", tmp_max, AMP_COEFF);

        namedWindow("vicon_test");
        Point p1 = Point(50,50);
        Point p2 = Point(950,950);
        rectangle(src, p1, p2, CV_RGB(0, 0, 255), -1);


        for(int i=0;i<x_marker_init.size();i++){
            circle(src, Point(500+x_marker_init[i]*AMP_COEFF, 500-y_marker_init[i]*AMP_COEFF), 2, Scalar(0, 255, 0));
            printf("x1 %d: %f\n", i, x_marker_init[i]);
            printf("y1 %d: %f\n", i, y_marker_init[i]);
        }


        for(int i=0;i<x_init_pos.size();i++){
            circle(src, Point(500+x_init_pos[i]*AMP_COEFF, 500-y_init_pos[i]*AMP_COEFF), 2, Scalar(0, 255, 0));
            printf("x%d: %f\n", i, x_init_pos[i]);
            printf("y%d: %f\n", i, y_init_pos[i]);
        }
        imshow("vicon_test", src);
    }

    //For sequence intialization
    static void onMouse(int event, int x, int y, int, void* userInput)
    {
        if (event != EVENT_LBUTTONDOWN && event != EVENT_LBUTTONUP) return;
        //printf("###########onMouse x : %d\n", x);
        //printf("###########onMouse y : %d\n", y);
        int x_world = x - 500;
        int y_world = 500 - y;
        Mat *img = (Mat*)userInput;
        if (event == EVENT_LBUTTONDOWN)
        {
            circle(*img, Point(x, y), 10, Scalar(0, 0, 255));
            imshow("vicon_test", *img);

            float nearest_dist=-1.0f;
            int nearest_index=0;
            for(int i=0;i<x_init_pos.size();i++){
                float sq_dist=sqrt((x_world-x_init_pos[i]*AMP_COEFF)*(x_world-x_init_pos[i]*AMP_COEFF)+(y_world-y_init_pos[i]*AMP_COEFF)*(y_world-y_init_pos[i]*AMP_COEFF));
                //printf("############# sq_dist : %f\n", sq_dist);
                if(sq_dist<nearest_dist||nearest_dist<0){
                    nearest_dist=sq_dist;
                    nearest_index=i;
                }
            }
            give_index(nearest_index);
        } else if (event == EVENT_LBUTTONUP)
        {
            float nearest_dist=-1.0f;
            int nearest_index=0;
            for(int i=0;i<x_init_pos.size();i++){
                float sq_dist=sqrt((x_world-x_init_pos[i]*AMP_COEFF)*(x_world-x_init_pos[i]*AMP_COEFF)+(y_world-y_init_pos[i]*AMP_COEFF)*(y_world-y_init_pos[i]*AMP_COEFF));
                //printf("############# sq_dist : %f\n", sq_dist);
                if(sq_dist<nearest_dist||nearest_dist<0){
                    nearest_dist=sq_dist;
                    nearest_index=i;
                }
            }
            //printf("********1\n");
            float close_len = -1.0f;
            int close_index = -1;//index in x_marker_init
            for (int j = 0; j < x_marker_init.size(); ++j)//for every x_marker_init
            {
                Vector3f tmp_diff;
                float tmp_len;
                tmp_diff(0) = x_init_pos[nearest_index] - x_marker_init[j];
                tmp_diff(1) = y_init_pos[nearest_index] - y_marker_init[j];
                tmp_diff(2) = 0;
                tmp_len = sqrt(tmp_diff(0)*tmp_diff(0)+tmp_diff(1)*tmp_diff(1));
                printf("**********%d\n", j);
                if (tmp_len > VEHICLE_SIZE)
                    continue;

                float tmp = sqrt((x_marker_init[j]*AMP_COEFF-x_world)*(x_marker_init[j]*AMP_COEFF-x_world)+(y_marker_init[j]*AMP_COEFF-y_world)*(y_marker_init[j]*AMP_COEFF-y_world));
                if (tmp < close_len || close_len < 0)
                {
                    close_len = tmp;
                    close_index = j;
                    printf("*******%d\n", j);
                }
            }
            //printf("********2\n");
            float x_arrow = x_marker_init[close_index] - x_init_pos[nearest_index];
            float y_arrow = y_marker_init[close_index] - y_init_pos[nearest_index];
            line(*img,Point(500+x_init_pos[nearest_index]*AMP_COEFF, 500-y_init_pos[nearest_index]*AMP_COEFF),Point(500+x_marker_init[close_index]*AMP_COEFF, 500-y_marker_init[close_index]*AMP_COEFF),Scalar(0,0,255),5,CV_AA);
            yaw_manuel.push_back(atan2(y_arrow, x_arrow));
            imshow("vicon_test", *img);
        }
    }

    void unite(vector<float> &x_init_pos,vector<float> &y_init_pos,vector<float> &x_marker_pos,vector<float> &y_marker_pos)
    {
        vector<bool> all_union(x_marker_pos.size(),0);

        for(int i=0;i<x_marker_pos.size();++i)//kick out noise
        {
            int within_circle = 0;
            for(int j=0;j<x_marker_pos.size();++j)
            {
                if(sqrt((x_marker_pos[i]-x_marker_pos[j])*(x_marker_pos[i]-x_marker_pos[j])+(y_marker_pos[i]-y_marker_pos[j])*(y_marker_pos[i]-y_marker_pos[j]))<ABOUT_EDGE)
                    ++within_circle;
            }
            if(within_circle<3)
                all_union[i]=1;
        }
        for(int i=0;i<x_marker_pos.size();++i)//choose the first point
        {
            if(all_union[i])continue;
            all_union[i]=1;
            vector<int> num_of_point(2,-1);
            vector<float> min_dstc(3,-1);

            float temp_dstc;
            for(int j=1;j<x_marker_pos.size();++j)//find the nearest point
            {
                if(all_union[j])continue;
                temp_dstc=(x_marker_pos[i]-x_marker_pos[j])*(x_marker_pos[i]-x_marker_pos[j])+(y_marker_pos[i]-y_marker_pos[j])*(y_marker_pos[i]-y_marker_pos[j]);
                if(min_dstc[0]>temp_dstc||min_dstc[0]<0)
                {
                    num_of_point[0]=j;
                    min_dstc[0]=temp_dstc;
                }
            }
            all_union[num_of_point[0]]=1;
            for(int j=1;j<x_marker_pos.size();++j)//find the second nearest point
            {
                if(all_union[j])continue;
                temp_dstc=(x_marker_pos[i]-x_marker_pos[j])*(x_marker_pos[i]-x_marker_pos[j])+(y_marker_pos[i]-y_marker_pos[j])*(y_marker_pos[i]-y_marker_pos[j]);
                if(min_dstc[1]>temp_dstc||min_dstc[1]<0)
                {
                    num_of_point[1]=j;
                    min_dstc[1]=temp_dstc;
                }
            }
            all_union[num_of_point[1]]=1;
            min_dstc[2]=(x_marker_pos[num_of_point[1]]-x_marker_pos[num_of_point[0]])*(x_marker_pos[num_of_point[1]]-x_marker_pos[num_of_point[0]])+(y_marker_pos[num_of_point[1]]-y_marker_pos[num_of_point[0]])*(y_marker_pos[num_of_point[1]]-y_marker_pos[num_of_point[0]]);

            float max_dstc;
            int num_of_max_dstc1,num_of_max_dstc2;
            if(min_dstc[0]>min_dstc[1])//find the farthest two points
            {
                num_of_max_dstc1=i;
                num_of_max_dstc2=num_of_point[0];
                max_dstc=min_dstc[0];
            }
            else{
                num_of_max_dstc1=i;
                num_of_max_dstc2=num_of_point[1];
                max_dstc=min_dstc[1];
            }
            if(max_dstc<min_dstc[2])
            {
                num_of_max_dstc1=num_of_point[0];
                num_of_max_dstc2=num_of_point[1];
            }

            temp_dstc=x_marker_pos[num_of_max_dstc1]+x_marker_pos[num_of_max_dstc2];//calculate the centre point
            x_init_pos.push_back(temp_dstc/2);
            temp_dstc=y_marker_pos[num_of_max_dstc1]+y_marker_pos[num_of_max_dstc2];
            y_init_pos.push_back(temp_dstc/2);

            for(int j=1;j<x_marker_pos.size();++j)//find and kick out the forth point, if cant find, it doesnt matter
            {
                if(all_union[j])continue;
                float cross_product,len_1,len_2;
                len_1=sqrt((x_marker_pos[num_of_max_dstc1]-x_marker_pos[j])*(x_marker_pos[num_of_max_dstc1]-x_marker_pos[j])+(y_marker_pos[num_of_max_dstc1]-y_marker_pos[j])*(y_marker_pos[num_of_max_dstc1]-y_marker_pos[j]));
                len_2=sqrt((x_marker_pos[num_of_max_dstc2]-x_marker_pos[j])*(x_marker_pos[num_of_max_dstc2]-x_marker_pos[j])+(y_marker_pos[num_of_max_dstc2]-y_marker_pos[j])*(y_marker_pos[num_of_max_dstc2]-y_marker_pos[j]));
                cross_product=(x_marker_pos[num_of_max_dstc2]-x_marker_pos[j])*(x_marker_pos[num_of_max_dstc1]-x_marker_pos[j])+(y_marker_pos[num_of_max_dstc2]-y_marker_pos[j])*(y_marker_pos[num_of_max_dstc1]-y_marker_pos[j]);
                float cos_degree=cross_product/len_1/len_2;
                if(cos_degree<0.08 && cos_degree>-0.08)
                {
                    all_union[j]=1;
                    break;
                }
            }
        }
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