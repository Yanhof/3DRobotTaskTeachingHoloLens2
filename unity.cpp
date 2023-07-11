#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "utility.h"
#include <iostream>
#include <vector>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8MultiArray.h>

//we get pose_msgs
//we publish multiarray int8
//publish jointpose
void singleCallback(const geometry_msgs::Pose::ConstPtr& p, moveit::planning_interface::MoveGroupInterface* move_group_ptr, ros::NodeHandle& nh)
{
    //get: pose
    //send: true/false (boolMSg)
    //send: JointStateMsg (q 7x1)

    ros::Publisher reachability_pub = nh.advertise<std_msgs::Bool>("reachability_single_pose", 10);
    ros::Publisher q_pub = nh.advertise<sensor_msgs::JointState>("joint_angles", 10);

    ROS_INFO("got new reference pose");
    // Get a pointer to the current robot state
    moveit::core::RobotState start_state(*move_group_ptr->getCurrentState());
    start_state.setToDefaultValues();
    // Update the planning scene with the current robot state
    move_group_ptr->setStartState(start_state);

    geometry_msgs::Pose target_pose;
    target_pose.orientation = p->orientation;
    target_pose.position = p->position;
    move_group_ptr->setPoseTarget(target_pose);

    std_msgs::Bool is_reachable;
    sensor_msgs::JointState q;
    q.header.stamp = ros::Time::now();
    q.name = move_group_ptr->getJointNames();
    q.position.resize(q.name.size());
    std::cout << "size of q is  " << q.position.size() << std::endl;
    std::cout << "Joint names are  " << std::endl;
    for (int i = 0; i <q.position.size(); i++){
        std::cout << q.name[i] << std::endl;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //---------------------------------------------
    ros::Duration(0.1).sleep();

    ROS_INFO("Plan was computed");
    if (success) {
        ROS_INFO("success");
        // Get the joint trajectory from the plan
        const auto& joint_values = plan.trajectory_.joint_trajectory.points.back().positions;
        ROS_INFO("got positions");
        // Update the joint state message with the last joint positions
        std::copy(joint_values.begin(), joint_values.end(), q.position.begin());
        ROS_INFO("copied positions");
        is_reachable.data = true;
        ROS_INFO("set reachable");
//--------------------------------------------
        ros::Duration(0.1).sleep();

        q_pub.publish(q);

    }
    else{
        ROS_INFO("No Plan found");
        is_reachable.data = false;
        ROS_INFO("set unreachable");
        //don't publish q
    }

    //----------------------------------
    ros::Duration(0.1).sleep();

    reachability_pub.publish(is_reachable);
    ros::Duration(0.1).sleep();
    ROS_INFO("published reachable");
}

 //hier nur zurückschicken: letzter Zustand bevor unfeasible
    //get: POseArrayMsg
    //get: length of pose array int8
    //send: int8MultiArrayMsg (1/0 feasible/unfeasible)
    //send: JointPoseMsg q für den letzten feasible Zustand (wenn es 2 gibt für den ersten)
    //update state of the robot if new point is feasible, if unfeasible is encountered plan for the next from last reachable

void fullCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array_msg, moveit::planning_interface::MoveGroupInterface* move_group_ptr, ros::NodeHandle& nh)
{
    ROS_INFO("Got Full Path Message");
    //create publishers with node handle
    ros::Publisher q_pub = nh.advertise<sensor_msgs::JointState>("joint_angles", 10);
    ros::Publisher reachability_pub = nh.advertise<std_msgs::Int8MultiArray>("reachability_full_path", 10);
    // Create a JointState message to publish the joint positions
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name = move_group_ptr->getJointNames();
    joint_state_msg.position.resize(joint_state_msg.name.size());
    std::cout << "size of q is  " << joint_state_msg.position.size() << std::endl;

    std_msgs::Int8MultiArray reachable_msg;
    reachable_msg.data.resize(pose_array_msg->poses.size());

    moveit::core::RobotState start_state(*move_group_ptr->getCurrentState());
    start_state.setToDefaultValues();
    // Update the planning scene with the current robot state
    move_group_ptr->setStartState(start_state);

    // Loop through all poses in the PoseArray message
    int i = 0;
    bool isLastReachable = false;
    for (const auto& pose : pose_array_msg->poses)
    {
        // Set the target pose for the planning scene
        move_group_ptr->setPoseTarget(pose);
        // Compute a plan to reach the target pose
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            ROS_INFO("Found solution");
            if (!isLastReachable){
                // Get the last joint positions from the plan
                const auto& joint_values = plan.trajectory_.joint_trajectory.points.back().positions;
                // Update the joint state message with the last joint positions
                std::copy(joint_values.begin(), joint_values.end(), joint_state_msg.position.begin());
            }
            reachable_msg.data[i] = 1;
            start_state.setVariableValues(joint_state_msg);
        }
        else
        {   // Publish a joint state message full of -1 if the plan fails
            ROS_INFO("Failed to find path");
            isLastReachable = true;
            reachable_msg.data[i] = 0;
        }
        i++;

    } //for loop
    // Publish the joint state message
    //---------------------------------------------------------------
    ROS_INFO("at position 0 of arra is " + reachable_msg.data[0]);
    ros::Duration(0.5).sleep();
    ROS_INFO("Before joint_angles publish");
    q_pub.publish(joint_state_msg);
    ros::Duration(0.5).sleep();
    ROS_INFO("After joint_angles publish and before sending reachable");
    reachability_pub.publish(reachable_msg);
    ROS_INFO("after publish reachable matrix");
    ros::Duration(0.1).sleep();
}


int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "unity_handler");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    auto* move_group = new moveit::planning_interface::MoveGroupInterface("panda_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //--------------------------------------------------------------------------------
    move_group->setPlanningTime(0.2);

    ros::Subscriber single_point = nh.subscribe<geometry_msgs::Pose>("check_single_pose", 1000, boost::bind(&singleCallback, _1, move_group, nh));
    ros::Subscriber full_path = nh.subscribe<geometry_msgs::PoseArray>("check_full_path", 1000, boost::bind(&fullCallback, _1, move_group, nh));

    ros::waitForShutdown();

    return 0;
}