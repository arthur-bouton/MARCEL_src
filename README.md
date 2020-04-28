# MARCEL_src

Source code for MARCEL (Mobile Active Rover Chassis for Enhanced Locomotion)


**Build the ROS package on the rover:**

Let's say your catkin workspace is *~/catkin_ws* for example:<br />
`CATKIN_WS=~/catkin_ws`

Put the *rover_ctrl* package among the catkin workspace sources:<br />
`ln -s $(readlink -f MARCEL_src/RaspberryPi/rover_ctrl) $CATKIN_WS/src`

Downloads the sources for the force-torque sensor package:<br />
`cd ~/Downloads`<br />
`git clone --depth=1 https://github.com/ros-industrial/robotiq.git`<br />
`ln -s ~/Downloads/robotiq/robotiq_ft_sensor $CATKIN_WS/src`

Download the C++ sources for the model trees:<br />
`cd $CATKIN_WS/src/rover_ctrl/src`<br />
`git clone --depth=1 https://github.com/Bouty92/ModelTree`

To autonomously start the low-level control node (nav_node) at boot:<br />
`sudo ln -s $(readlink -f MARCEL_src/RaspberryPi/nav_node.service) /etc/systemd/system`<br />
`sudo systemctl enable nav_node.service`


**Nodes for the operator's remote computer:**

To operate the rover from a remote computer, you only need the nodes in python:<br />
`mkdir -p $CATKIN_WS/src/rover_ctrl`<br />
`ln -s $(readlink -f MARCEL_src/RaspberryPi/rover_ctrl/scripts) $CATKIN_WS/src/rover_ctrl`
