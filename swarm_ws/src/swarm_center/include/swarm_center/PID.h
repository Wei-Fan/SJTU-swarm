//
// Created by zm on 18-12-1.
//

// #ifndef OFFB_POSCTL_PID_H
// #define OFFB_POSCTL_PID_H

#include <vector>

class PID {

public:
    //构造函数
    PID();
    float Kp;                          //参数P
    float Ki;                          //参数I
    float Kd;                          //参数D

    float error;                       //误差量 = 实际值 - 期望值
    float filter_data;                 //待滤波的数据
    float delta_time;                  //时间间隔dt

    std::vector <std::pair<float, float> > error_list; //误差表,用作计算微分项 平滑窗口 [2nd data, 1st time]
    std::vector <std::pair<float, float> > filter_list; //积分平均表

    float P_Out;                       //P环节输出
    float I_Out;                       //I环节输出
    float D_Out;                       //D环节输出
    float Output;                      //输出
    float Output_filter;               //积分滤波的输出

    bool start_intergrate_flag;        //是否积分标志[进入offboard(启控)后,才开始积分]
    float Imax;                        //积分上限
    float Output_max;                  //输出最大值
    float errThres;                    //误差死区(if error<errThres, error<0)

    //设置PID参数函数[Kp Ki Kd]
    void setPID(float p_value, float i_value, float d_value);
    //设置积分上限 控制量最大值 误差死区
    void set_sat(float i_max, float con_max, float thres);
    //输入 误差 和 当前时间
    bool add_error(float input_error, float curtime);
    void pid_output(void);

    //积分平均滤波器
    bool filter_input(float data2fliter, float curtime);
    void filter_output(void);

    //饱和函数
    float satfunc(float data, float Max, float Thres);
};


// #endif //OFFB_POSCTL_PID_H
