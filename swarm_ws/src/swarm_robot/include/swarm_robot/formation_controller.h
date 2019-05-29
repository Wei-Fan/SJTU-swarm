#ifndef SWARM_ROBOT_Formation_Controller_H
#define SWARM_ROBOT_Formation_Controller_H
#include <ros/ros.h>
#include <vector>
#include <utility>
#include <Eigen/Dense>
#include <eiquadprog.h>
#include <Hungarian.h>
#include <std_msgs/Int32MultiArray.h>

/**
 * The ROS wrapper around a controller which will take in message and adapt them to the controllers input.
 */
class FormationController {
private:
    ros::NodeHandle global;
    ros::NodeHandle local;

    /**
     * Publish the robots command velocity to lower level control
     */
    ros::Publisher cmd_vel_pub;

    ros::Publisher neighbours_pub;


    /**
     * This flag indiciates whether or not the main control loop is actively running.
     */
    bool active;

    /**
     * The lower level controller we will use to do actual calculations
     */
    std::string robot_name;
    int robot_id;
    int number_of_robots;

    /**
     * Parameters of role assignment
     */
    // std::vector<int> assignment;
    bool enable_assign;
    double start_time;

    /*
     * Parameters of formation
     */
    std::string formation_type = "circle";
    double formation_center[2] = {0,0}; 
    double circle_radius = 0;
    double square_length = 0; 
    
    double connectivity_radius;

    /*
     * Necessary parmeters for decentralized role assignmnent
     */
    std::vector<int> assignment;

    ros::Publisher assignment_request_pub;
    ros::Subscriber assignment_sub;

public:

    /**
     * Initialize the controller.
     * @param robot_name the name of the robot (excluding swarm prefix).
     */
    FormationController(const std::string &robot_name, bool debug=false);

    /**
     * Default starting function. Just call this to begin running the nodes main loop.
     */
    void run();

};

#endif //SWARM_ROBOT_Formation_Controller_H
