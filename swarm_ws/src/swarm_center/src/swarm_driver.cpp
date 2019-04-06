#include "ros/ros.h"
#include <swarm_center/PID.h>
#include <swarm_center/Parameter.h>
#include "serial/serial.h"
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//#define USING_OPTFLOW
//#define USING_OFFLINE_TRAJECTORY

#define NUM_OF_AGENTS	2

#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )

#define THRUST_BASE  		(40000)
#define PIDVX_pid_output_LIMIT	120.0f
#define PIDVY_pid_output_LIMIT	120.0f
#define PIDVZ_pid_output_LIMIT	(20000)

#define Int_max					20.0f
//#define Int_max					500.0f

#define PIDX_pid_output_LIMIT	1200.0f
#define PIDY_pid_output_LIMIT	1200.0f
#define PIDZ_pid_output_LIMIT	120.0f

using namespace std;

string parameter_file = "/home/wade/SJTU-swarm/swarm_ws/src/swarm_config/include/swarm_config/Minifly";
/* pre-defined position */
float position_bias_x[4] = {0.5, -0.5, -0.5, 0.5};
float position_bias_y[4] = {0.5, 0.5, -0.5, -0.5};

class MiniflyRos
{
public:
    uint8_t id = 0;
    float init_position[2];
    float pos_cmd[3] = {0.0,0.0,1.0};
    float rpyt_cmd[4] = {0.0,0.0,0.0,10.0};
    float current_pos[3] = {0.0,0.0,0.0};
    float current_vel[3] = {0.0,0.0,0.0};
    float current_acc[3] = {0.0,0.0,0.0};
    float u_i[2];
    PID PxPid,PyPid,PzPid;
    PID VxPid,VyPid,VzPid;
    float rpy_trim[3] = {0.0,0.0,0.0};
    geometry_msgs::PoseStamped err_pos;
    geometry_msgs::Vector3 err_pos_vec;

    // MiniflyRos(const std::string &dev_name, const std::string &parampath, int baudrate);
    MiniflyRos(uint8_t id_input, float *init_pos);//const std::string &parampath, uint8_t id_input);
    ~MiniflyRos();
//
//#ifdef USING_OFFLINE_TRAJECTORY
//    void read_Pos_traj(const std::string &filename);
//#endif
//
//private:
    std::string prefix;
//#ifndef USING_OPTFLOW
    // ros::Subscriber Mf_pos_sub;
	// ros::Subscriber Mf_vel_sub;
//	ros::Publisher thrust_pub;
//	ros::Publisher err_pub;
//	ros::Publisher err_vec_pub;
//	void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
//	void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
	void offb_pos_ctrl(float cur_time);

	ros::Subscriber pos_sub;
	void spCallback(const geometry_msgs::Pose::ConstPtr &msg);

//#endif
};
//
// MiniflyRos::MiniflyRos(const std::string &dev_name, const std::string &parampath, int baudrate):
// flydata(dev_name,baudrate)
MiniflyRos::MiniflyRos(uint8_t id_input, float *init_pos)://const std::string &parampath, uint8_t id_input):
        id(id_input)
{
    prefix = "swarmbot" + std::to_string(id);
    ros::NodeHandle n(prefix);
    // n.setCallbackQueue(&queue);
//#ifndef USING_OPTFLOW
	// Mf_pos_sub = n.subscribe<geometry_msgs::PoseStamped>("/mocap/pose", 10, &MiniflyRos::pos_cb,this);
	// Mf_vel_sub = n.subscribe<geometry_msgs::TwistStamped>("/mocap/vel", 10, &MiniflyRos::vel_cb,this);
//	thrust_pub=n.advertise<std_msgs::Float64>(prefix + "/thrust",100);
//	err_pub=n.advertise<geometry_msgs::PoseStamped>(prefix + "/err_pos",100);
//	err_vec_pub=n.advertise<geometry_msgs::Vector3>(prefix + "/err_pos_vec",100);

    init_position[0] = init_pos[0];
    init_position[1] = init_pos[1];

    // get sp from coverage_controllers
    char msg_name0[50];
    sprintf(msg_name0,"/%s/set_position",prefix.c_str());
    pos_sub = n.subscribe<geometry_msgs::Pose>(msg_name0,1,&MiniflyRos::spCallback,this);

    /* read PID parameter from file */
	Parameter param;
	if (param.readParam(parameter_file.c_str()) == 0){
	    throw std::runtime_error("read config file error!");
	    // return -1;
	}
	PxPid.setPID(param.x_p,param.x_i,param.x_d);
	PyPid.setPID(param.y_p,param.y_i,param.y_d);
	PzPid.setPID(param.z_p,param.z_i,param.z_d);
	VxPid.setPID(param.vx_p,param.vx_i,param.vx_d);
	VyPid.setPID(param.vy_p,param.vy_i,param.vy_d);
	VzPid.setPID(param.vz_p,param.vz_i,param.vz_d);
	PxPid.set_sat(Int_max,PIDX_pid_output_LIMIT,0.0);
	PyPid.set_sat(Int_max,PIDY_pid_output_LIMIT,0.0);
	PzPid.set_sat(Int_max,PIDZ_pid_output_LIMIT,0.0);
	VxPid.set_sat(Int_max,PIDVX_pid_output_LIMIT,0.0);
	VyPid.set_sat(Int_max,PIDVY_pid_output_LIMIT,0.0);
	VzPid.set_sat(Int_max,PIDVZ_pid_output_LIMIT,0.0);
	rpy_trim[0] = param.rpy_trim[0];
	rpy_trim[1] = param.rpy_trim[1];
	rpy_trim[2] = param.rpy_trim[2];
	rpyt_cmd[0] = 0.0 + rpy_trim[0];
	rpyt_cmd[1] = 0.0 + rpy_trim[1];
	rpyt_cmd[2] = 0.0 + rpy_trim[2];
	cout<<"MF id : "<<id<<endl;
	cout<<"X_pid: "<<param.x_p<<" "<<param.x_i<<" "<<param.x_d<<endl;
	cout<<"VX_pid: "<<param.vx_p<<" "<<param.vx_i<<" "<<param.vx_d<<endl;
	cout<<"Y_pid: "<<param.y_p<<" "<<param.y_i<<" "<<param.y_d<<endl;
	cout<<"VY_pid: "<<param.vy_p<<" "<<param.vy_i<<" "<<param.vy_d<<endl;
	cout<<"Z_pid: "<<param.z_p<<" "<<param.z_i<<" "<<param.z_d<<endl;
	cout<<"VZ_pid: "<<param.vz_p<<" "<<param.vz_i<<" "<<param.vz_d<<endl;
//#endif
    // float tmp[] = {0.0,0.0,1.0};
    // memcpy(pos_cmd,tmp,3);
    // float tmp1[] = {0.0,0.0,0.0,1000.0};
    // memcpy(rpyt_cmd,tmp,4);
}

void MiniflyRos::spCallback(const geometry_msgs::Pose::ConstPtr &msg) {
    pos_cmd[0] = msg->position.x;
    pos_cmd[1] = msg->position.y;
    pos_cmd[2] = msg->position.z;
}

MiniflyRos::~MiniflyRos()
{
//#ifdef USING_OFFLINE_TRAJECTORY
//    cmd_incsv.clear();
//#endif
}

//#ifndef USING_OPTFLOW
//void MiniflyRos::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
//{
//	geometry_msgs::PoseStamped tmp = *msg;
//	current_pos[0] = tmp.pose.position.x;
//	current_pos[1] = tmp.pose.position.y;
//	current_pos[2] = tmp.pose.position.z;
//}
//
//void MiniflyRos::vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
//    geometry_msgs::TwistStamped tmp = *msg;
//    current_vel[0] = tmp.twist.linear.x;
//    current_vel[1] = tmp.twist.linear.y;
//    current_vel[2] = tmp.twist.linear.z;
//}

void MiniflyRos::offb_pos_ctrl(float cur_time) // current_pos not finish!!!!!!
{

	PxPid.add_error(pos_cmd[0]-current_pos[0],cur_time);PxPid.pid_output();
	PyPid.add_error(pos_cmd[1]-current_pos[1],cur_time);PyPid.pid_output();
	PzPid.add_error(pos_cmd[2]-current_pos[2],cur_time);PzPid.pid_output();
//	cout<<"errP_xyz\t"<<pos_cmd[0]-current_pos[0]<<" "<<pos_cmd[1]-current_pos[1]<<" "<<pos_cmd[2]-current_pos[2]<<endl;
	VxPid.add_error(0.1*PxPid.Output - current_vel[0],cur_time);VxPid.pid_output();
	VyPid.add_error(0.1*PyPid.Output - current_vel[1],cur_time);VyPid.pid_output();
	VzPid.add_error(PzPid.Output - current_vel[2],cur_time);VzPid.pid_output();
	err_pos.pose.position.x = pos_cmd[0]-current_pos[0];
	err_pos.pose.position.y = pos_cmd[1]-current_pos[1];
	err_pos.pose.position.z = pos_cmd[2]-current_pos[2];
	err_pos_vec.x = pos_cmd[0]-current_pos[0];
	err_pos_vec.y = pos_cmd[1]-current_pos[1];
	err_pos_vec.z = pos_cmd[2]-current_pos[2];
	// cout<<"errV_xyz\t"<<0.1*PxPid.Output - current_vel[0]<<" "<<0.1*PyPid.Output - current_vel[1]<<" "<<PzPid.Output - current_vel[2]<<endl;
	rpyt_cmd[1] = 0.15*VxPid.Output + rpy_trim[1];
	rpyt_cmd[0] = - 0.15*VyPid.Output + rpy_trim[0];
	rpyt_cmd[3] = (VzPid.Output + THRUST_BASE)/655.35f;
	rpyt_cmd[2] = 0.0 + rpy_trim[3];
	// cout<<"current_time="<<cur_time<<endl;
//	cout<<"pid z vz:"<<PzPid.Output<<" "<<VzPid.Output<<endl;
	// cout<<"rpyt = "<<rpyt_cmd[0] <<" "<<rpyt_cmd[1] <<" "<<rpyt_cmd[2] <<" "<<rpyt_cmd[3] <<endl;
}
//#endif
//
//#ifdef USING_OFFLINE_TRAJECTORY
//// void MiniflyRos::read_Pos_traj(const std::string &filename)
//void MiniflyRos::read_Pos_traj(const std::string &filename)
//{
//    csvdata intp;
//    FILE *fp;
//    std::string pathaddr("/home/sanmu/Minifly/Minifly_ros_ws/src/mini_swarm/data/"+filename);
//    fp=fopen(pathaddr.c_str(),"r");//Äã×Ô¼ºµÄÎÄ¼þÂ·¾¶
//    if(fp){std::cout<<"open path csv"<<std::endl;}
//    while(1){
//        fscanf(fp,"%f,%f,%f",&intp.xyz[0],&intp.xyz[1],&intp.xyz[2]);
//        cmd_incsv.push_back(intp);
//        // std::cout<<"takeoff_pos: x="<<intp.xyz[0]<<"; y="<<intp.xyz[1]<<"; z="<<intp.xyz[2]<<std::endl;
//        if (feof(fp))break;
//    }
//    fclose(fp);
//    // for(int i=0;i<incsv.size();i++)
//    // {
//    //     std::cout<<incsv[i].xyz[0]<<" "<<incsv[i].xyz[1]<<" "<<incsv[i].xyz[2]<<std::endl;
//    // }
//}
//#endif
//
class MiniSwarm
{
private:
    ros::Publisher ready_pub;
    vector<ros::Publisher> raw_pub;
    string prefix = "swarmbot";
//    vector<float> position_bias_x;
//    vector<float> position_bias_y;

public:
    std::vector<MiniflyRos*> Mfs;
    serial::Serial ros_ser;
    MiniSwarm(const std::string &dev_name, uint8_t num_of_agents, int baudrate);
    ~MiniSwarm();
    void addMf(uint8_t id, float *init_pos);//const std::string &parampath, uint8_t id);
    void update_power(std::string &pkg_tmp);
    void update_state(std::string &pkg_tmp);
    void raw_data_decoding();
    void take_off(uint8_t id);
    void send_pos_sp(uint8_t id, float *xyz);
    void send_att_sp(uint8_t id, float *rpyt);
    void run();
    float get_ros_time(ros::Time time_begin);
};
//
MiniSwarm::MiniSwarm(const std::string &dev_name, uint8_t num_of_agents, int baudrate)
{
    ros::NodeHandle node("");
    // tell dispatch_center that all vehecles armed
    ready_pub = node.advertise<std_msgs::Bool>("/setup_ready",100);

    // publish raw pos for coverage_controllers and coverage_commander
    raw_pub.resize(NUM_OF_AGENTS);
    for (int i = 0; i < NUM_OF_AGENTS; ++i) {
        char msg_name[50];
        sprintf(msg_name,"/%s%d/raw_position",prefix.c_str(),i);
        raw_pub[i] = node.advertise<geometry_msgs::PoseStamped>(msg_name,1);
    }

    try
    {
        ros_ser.setPort(dev_name);
        // ros_ser.setPort("/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_COM_Port_6D87155F5056-if00");
        ros_ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_ser.setTimeout(to);
        ros_ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
    }

    if(ros_ser.isOpen()){
        ROS_INFO_STREAM("Serial Port opened");
    }else{
        ROS_ERROR_STREAM("No port opened");
    }

    for(uint8_t i=0; i < num_of_agents; ++i)
    {
        float init_p[2];
        init_p[0] = position_bias_x[i];
        init_p[1] = position_bias_y[i];
        addMf(i,init_p);
    }
}
MiniSwarm::~MiniSwarm()
{
    for (auto mf: Mfs)
    {
        delete mf;
    }
}
void MiniSwarm::addMf(uint8_t id, float *init_pos)//const std::string &parampath, uint8_t id)
{
    MiniflyRos* mf = new MiniflyRos(id,init_pos);//parampath,id);
//    mf->read_Pos_traj("test8.csv");
    Mfs.push_back(mf);
}

void MiniSwarm::update_power(std::string &pkg_tmp)
{
    uint16_t tmp;
    uint8_t id = pkg_tmp[1];
    for (int i=0; i<2; ++i){
        tmp = uint16_t((unsigned char)pkg_tmp[2*i+2]*256+ (unsigned char)pkg_tmp[2*i+3]);
        Mfs[id]->u_i[i] = float(tmp)/100;
    }

    printf("MF%02X: U = %2.2f, I = %2.2f\n",id,Mfs[id]->u_i[0],Mfs[id]->u_i[1]);
    // std::cout<<"MF"<<id<<": U = "<<Mfs[id]->u_i[0]<<"V, I = "<<Mfs[id]->u_i[1]<<"A"<<std::endl;
}

void MiniSwarm::update_state(std::string &pkg_tmp)
{
    uint16_t tmp;
    uint8_t id = pkg_tmp[1];
    for (int i=0; i<3; ++i){
        tmp = uint16_t((unsigned char)pkg_tmp[2*i+2]*256+ (unsigned char)pkg_tmp[2*i+3]);
        Mfs[id]->current_acc[i] = float(tmp)/100;
    }
    for (int i=3; i<6; ++i){
        tmp = uint16_t((unsigned char)pkg_tmp[2*i+2]*256+ (unsigned char)pkg_tmp[2*i+3]);
        Mfs[id]->current_vel[i-3] = float(tmp)/100;
    }
    for (int i=6; i<9; ++i){
        tmp = uint16_t((unsigned char)pkg_tmp[2*i+2]*256+ (unsigned char)pkg_tmp[2*i+3]);
        Mfs[id]->current_pos[i-6] = float(tmp)/100;
    }
    Mfs[id]->current_pos[0] += position_bias_x[id];
    Mfs[id]->current_pos[1] += position_bias_y[id];

    /* publish optflow's position estimation */
    geometry_msgs::PoseStamped msg;
    msg.header.seq = id;
    msg.pose.position.x = Mfs[id]->current_pos[0];
    msg.pose.position.y = Mfs[id]->current_pos[1];
    msg.pose.position.z = Mfs[id]->current_pos[2];
    char msg_name[50];
    sprintf(msg_name,"/%s%d/raw_position",this->prefix.c_str(),id);
    raw_pub[id].publish(msg);

    printf("MF_cp%02X: x = %2.2f, y = %2.2f, z = %2.2f\n",id,Mfs[id]->current_pos[0],Mfs[id]->current_pos[1],Mfs[id]->current_pos[2]);

}

void MiniSwarm::raw_data_decoding()
{
    if(ros_ser.available()){
        std::string buffer;
        buffer = ros_ser.read(ros_ser.available());
        int cnt = 0;
        int msg_len = 0;
        int msg_type = 0;
        int i = 0;
        std::string pkg_tmp;
        while (i < buffer.length()){
            if ((unsigned char)buffer[i] == 0xAA && (unsigned char)buffer[i+1] == 0xAA && (unsigned char)buffer[i+2] != 0xAA ){
                msg_type = buffer[i+2];
                msg_len = buffer[i+3];
                //check the pkg
                std::cout<<"msg_type:"<<msg_type<<std::endl;
                if (msg_type ==1 || msg_type ==2 || msg_type ==3 || msg_type ==5 ||msg_type ==-15){
                    uint8_t sum = 0;
                    for (int j = 0; j < msg_len+4; ++j){
                        sum = sum + buffer[i+j];
                    }
                    if ((sum-buffer[i+msg_len+4])%256==0){
                        pkg_tmp = buffer.substr(i+4,msg_len);
                        switch(msg_type){
                            // case 1 :
                            //     update_state(pkg_tmp);
                            //     break;
                            // case 2 :
                            //     update_senser(pkg_tmp);
                            //     break;
                            // case 3 :
                            //     update_ctrl_cmd(pkg_tmp);
                            //     break;
                            case 5 :
                                update_power(pkg_tmp);
                                break;
                            case -15 :
                                update_state(pkg_tmp);
                            default :
                                break;
                        }
                    }
                }
                i = i + msg_len + 5;
            }else{
                ++i;
            }
        }
    }
}

void MiniSwarm::take_off(uint8_t id){
    uint8_t takeoff[] = {0xAA,0xAF,0x50,0x03,0x00,0x03,0x00,0xAF};
    takeoff[2] = takeoff[2] + id;
    takeoff[7] = takeoff[7] + id;
    ros_ser.write(takeoff,8);
}

void MiniSwarm::send_pos_sp(uint8_t id, float *xyz)
{
    uint8_t cmd[]={0xAA,0xAF,0x60,0x0D,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t cksum = 0xC7 + id;
    cmd[2] = cmd[2] + id;
    int cnt = 5;
    float tmp;
    for(int i = 0; i<3; ++i){
        tmp = xyz[i];
        cmd[cnt++] = BYTE0(tmp);
        cmd[cnt++] = BYTE1(tmp);
        cmd[cnt++] = BYTE2(tmp);
        cmd[cnt++] = BYTE3(tmp);
        cksum += BYTE3(tmp) + BYTE2(tmp) + BYTE1(tmp) + BYTE0(tmp);
        // printf("%02hhX %02hhX %02hhX %02hhX ", BYTE0(tmp), BYTE1(tmp), BYTE2(tmp), BYTE3(tmp));
    }
    cmd[cnt] = cksum;
//    printf("msgid: %02hhX, xyz: %5.5f %5.5f %5.5f \n",cmd[2],xyz[0],xyz[1],xyz[2]);
    ros_ser.write(cmd,18);
}

void MiniSwarm::send_att_sp(uint8_t id, float *rpyt)
{
    uint8_t cmd[]={0xAA,0xAF,0x60,0x11,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t cksum = 0xCA + id;
    cmd[2] = cmd[2] + id;
    int cnt = 5;
    float tmp;
    for(int i = 0; i<4; ++i){
        tmp = rpyt[i];
        cmd[cnt++] = BYTE0(tmp);
        cmd[cnt++] = BYTE1(tmp);
        cmd[cnt++] = BYTE2(tmp);
        cmd[cnt++] = BYTE3(tmp);
        cksum += BYTE3(tmp) + BYTE2(tmp) + BYTE1(tmp) + BYTE0(tmp);
        // printf("%02hhX %02hhX %02hhX %02hhX ", BYTE0(tmp), BYTE1(tmp), BYTE2(tmp), BYTE3(tmp));
    }
    cmd[cnt] = cksum;
    // printf("%02hhX\n",cksum);
    ros_ser.write(cmd,22);
}

void MiniSwarm::run()
{
    ros::Rate loop_rate(20);
//#ifdef USING_OFFLINE_TRAJECTORY
//    int cnt = 0;
//    int limit = 0;
//    for(auto mf:Mfs)
//    {
//        if(mf->cmd_incsv.size()>limit){limit = mf->cmd_incsv.size();}
//    }
//#endif
//#ifdef USING_OPTFLOW
    for(int i = 0; i<50; ++i)
    {
        for(auto mf:Mfs){
            send_pos_sp(mf->id,mf->pos_cmd);
            // cout<<mf->id<<endl;
            // printf("%02X\n",mf->id);
        }
        loop_rate.sleep();
    }
    for(auto mf:Mfs){
        take_off(mf->id);
    }
    /* armed */
    std_msgs::Bool ready;
    ready.data = true;
    ready_pub.publish(ready);
    loop_rate.sleep();


//    cout<<"take off!"<<endl;
//    for(int i = 0; i<50; ++i)
//    {
//        for(auto mf:Mfs){
//            send_pos_sp(mf->id,mf->pos_cmd);
//        }
//        loop_rate.sleep();
//    }
//#else
    float cur_time;
	ros::Time begin_time = ros::Time::now();
//	for(int i = 0; i<50; ++i)
//	{
//		for(auto mf:Mfs){
//			send_att_sp(mf->id,mf->rpyt_cmd);
//		}
//		loop_rate.sleep();
//	}
//	for(auto mf:Mfs){
//		take_off(mf->id);
//	}
//	loop_rate.sleep();
//	for(int i = 0; i<50; ++i)
//	{
//		ros::spinOnce();
//		for(auto mf:Mfs){
//			cur_time = get_ros_time(begin_time);
//			mf->offb_pos_ctrl(cur_time);
//			send_pos_sp(mf->id,mf->rpyt_cmd);
//		}
//		loop_rate.sleep();
//	}
//#endif
//    std::cout<<ros::ok()<<std::endl;
    while(ros::ok())
    {
        raw_data_decoding();
//#ifdef USING_OFFLINE_TRAJECTORY
//#ifndef USING_OPTFLOW
        for(auto mf:Mfs){
			cur_time = get_ros_time(begin_time);
//			memcpy(mf->pos_cmd,mf->cmd_incsv[cnt].xyz,12);
			mf->offb_pos_ctrl(cur_time);
			send_att_sp(mf->id,mf->rpyt_cmd);
//			ROS_INFO("robot %d send out sp: %f,%f",mf->id,mf->current_pos[0],mf->current_pos[1]);
		}
//#else
//        for(auto mf:Mfs){
//            // send_pos_sp(mf->id,mf->pos_cmd);
//            send_pos_sp(mf->id,mf->cmd_incsv[cnt].xyz);
//        }
//#endif
//        cnt++;
//        if(cnt == limit){break;}
//#endif
        ros::spinOnce();
        loop_rate.sleep();
    }
//#ifndef USING_OPTFLOW
//    float det_alt_tmp[6];
//	float x_tmp[6];
//	float y_tmp[6];
//	for(int i=0; i< Mfs.size();++i)
//	{
//		x_tmp[i] = Mfs[i]->current_pos[0];
//		y_tmp[i] = Mfs[i]->current_pos[1];
//		det_alt_tmp[i] = Mfs[i]->current_pos[2];
//	}
//	for(int i = 0; i < 100; +i)
//	{
//		for(int j=0; j< Mfs.size(); ++j){
//			cur_time = get_ros_time(begin_time);
//			Mfs[j]->pos_cmd[0] = x_tmp[j];
//			Mfs[j]->pos_cmd[1] = y_tmp[j];
//			Mfs[j]->pos_cmd[2] = det_alt_tmp[j]*(1-i/100);
//			Mfs[j]->offb_pos_ctrl(cur_time);
//			send_att_sp(Mfs[j]->id,Mfs[j]->pos_cmd);
//		}
//		loop_rate.sleep();
//	}
//#endif
}

float MiniSwarm::get_ros_time(ros::Time time_begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-time_begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - time_begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}



int main(int argc, char **argv)
{
    // raise(SIGSTOP);
    ros::init(argc, argv, "swarm_driver");
    ros::NodeHandle Global;
    MiniSwarm Miniflys("/dev/ttyACM0",NUM_OF_AGENTS,500000);
//    // std::cout<<"fuck you!"<<std::endl;
//    // aMinifly.read_Pos_traj("test8.csv");
    Miniflys.run();
    return 0;
}
