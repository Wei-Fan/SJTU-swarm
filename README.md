# SJTU-swarm
This is a package created for my diploma project<br>


---------------------------------
* **Framework test enter:**<br>
`$roslaunch swarm_config test_one.launch`<br>
using VICON:
`$roslaunch viconros miniswarm_cap.launch`<br>
using rviz for visualization:
`$rviz`<br>
using terminal to trigger dispatch process:
`$rostopic pub /switch_command std_msgs/Empty`<br>

* **Instructions for use:**<br>
-> Generate multiple robots through launch multiple one_robot.launch file.<br>
-> Each robot has a robot_controller to communicate with low-level controller.<br>
-> Each robot perform behavior according to parameters in launch file.

* **How to automatically generate launch file with multiple robots?**<br>
(coming up soon)<br>
`$rosrun swarm_config setup_swarm "/home/$USER/SJTU-swarm/swarm_ws/src/swarm_config/src/" "/home/$USER/SJTU-swarm/swarm_ws/src/swarm_config/launch/" "test.yaml"`

* **How to add a new behavior?**<br>
-> Create a new behavior launch file with the name of the new behavior. Please refer to formation_control.launch.<br>
-> Write a new ros node and realize the new behavior in the package of swarm_robot.<br>
-> The new behaviour name won't be passed to the commander node. So, if you want to do something in the swarm_center part, remember modify the commander.launch file.<br>
-> Declare a new behavior name (coming up soon)<br>

* **What does Swarm Center include?**<br>
coverage_commander: implement DARP+STC algorithm <br>
dispatch_center: auto-recharging dispatch algorithm<br>
swarm_driver: low-level controller<br>
swarm_visualization: use rviz to visualization<br>

* **Note**<br>
1 There are two branch. The master branch is to use optical flow and the dynamic branch uses VICON. Also, the dynamic branch enables the swarm to change the flight during the executing process.