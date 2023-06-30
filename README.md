# RobotTaskTeaching3DHoloLens

## This project uses hand tracking of the HoloLens 2 to attach the end effector to the user's hand and teach a task with it to a robot

This application was built as a bachelor thesis at the pdZ lab at ETH Zurich. The user has a choice between a time and equidistance mode. Time mode records poses of the end effector at fixed intervals and checks them through ROS every few seconds, the result gets then visualized by the digital twin. In equidistance mode are points recorded at fixed distances from each other and the user has the choice to create a spline, which he can edit through the control points. It is built with Unity 3D v2021.3.19f1, MRTK v2.8.3.0 and uses URDF importer v0.5.2,  ROS-TCP-Connector v0.7.0 and, Bezier Solution v2.3.3.


## Launching ROS host computer
Launch each of the commands in a new terminal:
1. roslaunch ros_tcp_endpoint endpoint.launch
2. roslaunch panda_moveit_config demo.launch (you can minimize RViz)
3. rosrun goal_state_publisher unity

## Known Bugs / Issues /Shortfallings
* Reattachin directly at the start/end does not work on the HoloLens (works on the PC)
* The implemented hand gestures create many false positives and are in general a bit unsatisfactory
* After the robot has been replaces
