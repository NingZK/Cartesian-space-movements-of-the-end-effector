THe ROS package description:

This ROS package will automatically generate Cartesian space movements of the end-effector
of the Panda robot manipulator: the end-effector will have to "draw" squares of different
sizes on the x-y Cartesian plane, starting from a given robot configuration.

To run the package:

0) Download and install the following packages:
     - git clone -b ROS-DISTRO-devel https://github.com/ros-planning/moveit_tutorials.git
     - git clone -b ROS-DISTRO-devel https://github.com/ros-planning/panda_moveit_config.git
     where ROS-DISTRO  is the name of your ROS distribution(e.g. melodic)

1) Unzip the "ar_week10_test.zip" folder in your carkin workspace.
2) There are five folders named launch,msg,scripts,src and srv and one CMakeLists.txt and one README.txt and a package.xml in the folder.
3) Build the catkin workspace.
4) When the workspace is ready,enter the following command:
     - catkin_make
     - source devel/setup.bash
5) Run the following command to each new terminals:
     - roslaunch panda_moveit_config demo.launch
     - rosrun ar_week10_test square_size_generator.py
     - rosrun ar_week10_test move_panda_square.py
     - rosrun rqt_plot rqt_plot

