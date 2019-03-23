# SJTU-swarm
This is a package created for my diploma project<br>


---------------------------------
* **Framework test enter:**<br>
`$roslaunch swarm_config test_one.launch`

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
-> Declare a new behavior name (coming up soon)<br>

* **What does Swarm Center include?**<br>
Commander node and performance summary (coming up soon)<br>