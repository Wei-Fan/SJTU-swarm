//
// Created by zm on 18-12-1.
//

#include "PID.h"
#include <cstdlib>
#include <utility>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cmath>
#include "string"
#include <time.h>
#include <queue>
#include <vector>
#include <algorithm>

using namespace std;

PID::PID() {
    error_list.push_back(make_pair(0.0f, 0.0f));
    filter_list.push_back(make_pair(0.0f, 0.0f));
    filter_data = 0;
    error = 0;
    P_Out = 0;
    I_Out = 0;
    D_Out = 0;
    Output = 0;
    Output_filter = 0;
    start_intergrate_flag = false;
}


float PID::satfunc(float data, float Max, float Thres)
{
    if (fabs(data)<Thres)
        return 0;
    else if(fabs(data)>Max){
        return (data>0)?Max:-Max;
    }
    else{
        return data;
    }
}

void PID::setPID(float p_value, float i_value, float d_value)
{
    Kp = p_value;
    Ki = i_value;
    Kd = d_value;
}

void PID::set_sat(float i_max, float con_max, float thres)
{
    Output_max = con_max;
    Imax = i_max;
    errThres = thres;
}

bool PID::add_error(float input_error, float curtime)
{
    error = input_error;
    // delta_time = 0.05; /////////////////////////////////////////////////////////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!1
    if(error_list.size() == 1)
    {
        delta_time = curtime;
    }
    else{
        delta_time = curtime - error_list.rbegin()->first;
    }
//    cout<<"dt="<<delta_time<<endl;
    if(error_list.size()<10){
        error_list.push_back(make_pair(curtime, error));
    }
    else{
        vector<pair<float, float> >::iterator k_beg = error_list.begin();
        error_list.erase(k_beg);
        std::pair<float,float > p1(curtime, error);
        error_list.push_back(p1);
    }

    return true;
}

void PID::pid_output(void)
{
    P_Out = Kp * error;                          //P环节输出值
    I_Out = I_Out + Ki *error*delta_time;        //I环节输出值
    I_Out = satfunc(I_Out, Imax, 0);             //I环节限幅[I_Out<=Imax]
    if(start_intergrate_flag == 0)
    {
        I_Out = 0;
    }

    D_Out = 0;

    if (error_list.size() < 3 || Kd == 0)
    {
        D_Out = 0; //initiral process
    }
    else
    {
        vector<pair<float, float> >::reverse_iterator error_k = error_list.rbegin();
        vector<pair<float, float> >::reverse_iterator error_k_1 = error_k + 1;
        D_Out = (error_k->second - error_k_1->second)/delta_time * Kd;
    }

    Output = P_Out + I_Out + D_Out;
    Output = satfunc(Output, Output_max, errThres);
}

bool PID::filter_input(float data2fliter, float curtime)
{
    filter_data = data2fliter;
    if(filter_list.size() < 10){
        filter_list.push_back(make_pair(curtime, filter_data));
    }
    else{
        vector<pair<float, float > > ::iterator fil_iter = filter_list.begin();
        filter_list.erase(fil_iter);
        std::pair<float, float > temp_iter(curtime, filter_data);
        filter_list.push_back(temp_iter);
    }
    return true;
}

void PID::filter_output()
{
    if(filter_list.size() < 10 || ! start_intergrate_flag){
        Output_filter = 0;
    }
    else{
        vector<pair<float, float> >::iterator filter_k;
        float filter_sum = 0;
        for(filter_k = filter_list.begin(); filter_k != filter_list.end(); ++ filter_k){
            filter_sum = filter_sum + filter_k->second;
        }
        Output_filter = filter_sum * delta_time/(filter_list.back().first - filter_list.front().first);
    }
}
