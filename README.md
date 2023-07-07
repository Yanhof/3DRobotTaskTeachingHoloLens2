# 3DRobotTaskTeachingHoloLens

## This project uses hand tracking of the HoloLens 2 to attach the end effector to the user's hand and teach a task with it to a robot

This application was built as a bachelor thesis at the pdZ lab at ETH Zurich. The user has a choice between a time and equidistance mode. Time mode records poses of the end effector at fixed intervals and checks them through ROS every few seconds, the result gets visualized by the digital twin of the Franka Research 3. In equidistance mode are points recorded at fixed distances from each other and the user has the choice to create a spline, which he can edit through the control points. It is built with Unity 3D v2021.3.19f1, MRTK v2.8.3.0 and uses URDF importer v0.5.2,  ROS-TCP-Connector v0.7.0 and, Bezier Solution v2.3.3.

![Image of hand tracking active in time mode](titleimage_final.png)

### Using the app
After dragging the robot base to the desired position with one hand and scaling and rotating it with two hands, use the hand menu on the left to place the robot on the base. Select the desired mode in the main menu and start tracking as described. In a nutshell, place your right hand where you want to record and make a thumbs-up gesture with your left hand to start and stop a recording. Along the way, you can create pick-up and place events by making a pinch gesture (left hand) or empty events by making a victory pose (left hand, palm to eyes). Events are intended to be used later for various end-effector manipulations (e.g. starting a welder), to change certain settings of the robot (e.g. speed), or to be used for human-robot interaction, visualised by showing the position and orientation of the end effector was in when they were recorded. If the hand tracking loses the user's hand, or the pose is out of reach for the robot, the end effector snaps back to the last verified position. The user can place their hand in the same position and orientation to reattach the end effector.


![A recorded trajectory with three events](trajectory_events.jpg)
This image shows a recorded trajectory with three events

![A recorded trajectory with a spline which was checked for reachability by the IK](red_green_check_points.jpg)
A recorded trajectory with a spline which was checked for reachability by the IK


### Overview of scripts
* The script ROS handles all the ROS related stuff (built with ROS 1) (found at ROS --> MyROS)
* The scripts VisualizeRobot and SetRobot are used to handle the digital twin of the robot (also at ROS --> MyROS)
* HandPose is needed for the detection of the different hand gestures (attached at HandTracking game object)
* GripperAtHand handles the attaching of the end effector to the user hand (attached at HandTracking game object)
* The script MainController handles all the recording of the trajectory and events as well as all the general behaviour (attached at Manager game object)

### How the Unity scene is built up and where to find things
The main manager can be found as a child of the RecordingTrajectory game object. It holds the trajectory as well as the total saved workflow and all the menus. To use the hand menu on the PC go to Menus --> HandMenu and activate the game object MenuContent. Under the game object HandTracking can you find the end effector which will be attached to the user's hand and a holder for the last valid end effector pose. Everything related to ROS can be found under ROS --> MyROS, ROS_Server_Imitation_Old hold a script which can be used to simulate the feedback received from ROS using a random check. Finally, under the Robot game object is the Franka Research 3 as well as  the base used for positioning placed.

## Launching ROS host computer
Launch each of the commands in a new terminal:
1. roslaunch ros_tcp_endpoint endpoint.launch (Maybe you need to first download the ROS-TCP-Endpoint package)
2. roslaunch panda_moveit_config demo.launch (you can minimize RViz)
3. rosrun goal_state_publisher unity (Thanks to <a href="https://github.com/LucasG2001"> LucasG2001  <img src="https://contrib.rocks/image?repo=LucasG2001/Android-APP" width="20" />
</a>  for coding the file)
   
![Setup ROS host computer and HoloLens 2](Setup_additional.png)


## Known Bugs / Issues / Shortfallings
* Rendering of workflow trajectory has holes in the path
* Reattachin directly at the start/end does not work on the HoloLens (works on the PC)
* The implemented hand gestures create many false positives and are in general a bit unsatisfactory
* After the robot has been replaced at another position is the visualization off (angles received back from IK of ROS let it point somewhere else, also end effector fingers are no longer at the correct position)

