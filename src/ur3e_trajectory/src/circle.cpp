#include "../include/circle.hpp"
#include <math.h>

#define PI 3.14159265

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "circle");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
   
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    float r = 0.45;
    int sides = 20;
   
    //Write your code for following the circle trajectory here.
   
    //horizontal
    /*moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = 1.29154;
    joint_targets["shoulder_lift_joint"] = -0.855211;
    joint_targets["shoulder_pan_joint"] = 0.366519;
    joint_targets["wrist_1_joint"] = -2.00713;
    joint_targets["wrist_2_joint"] = -1.55334;
    joint_targets["wrist_3_joint"] = -1.3439;

    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }

    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;

    geometry_msgs::Pose pose_target = arm_move_group.getCurrentPose().pose;
    pose_target.position.x = r;
    pose_target.position.y = 0.0;
    pose_target.position.z = 0.898;

    bool pose_plan_success;
    std::string reference_frame = arm_move_group.getPlanningFrame();
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target, reference_frame, pose_plan);

    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan);
    }

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;

    waypoints.push_back(start_pose);

    start_pose.position.x = 0.0;
    start_pose.position.y = 0.0;

    for(int m = 1; m < 20; m++){
        geometry_msgs::Pose end_pose = start_pose;
        end_pose.position.x += r * cos((2*m*PI)/sides);
        end_pose.position.y += r * sin((2*m*PI)/sides);
        end_pose.position.z += 0.0;

        waypoints.push_back(end_pose);
    }

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(pose_target, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);*/

    //vertical
     moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = -0.10472;
    joint_targets["shoulder_lift_joint"] = -1.98968;
    joint_targets["shoulder_pan_joint"] = -0.139626;
    joint_targets["wrist_1_joint"] = -1.02974;
    joint_targets["wrist_2_joint"] = 1.79769;
    joint_targets["wrist_3_joint"] = 0.645772;

    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }

    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;

    geometry_msgs::Pose pose_target = arm_move_group.getCurrentPose().pose;
    pose_target.position.x = -0.1;
    pose_target.position.y = 0.0;
    pose_target.position.z = 0.898 + r;

    bool pose_plan_success;
    std::string reference_frame = arm_move_group.getPlanningFrame();
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target, reference_frame, pose_plan);

    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan);
    }

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;

    waypoints.push_back(start_pose);

    start_pose.position.z = 0.0;
    start_pose.position.y = 0.0;

    for(int m = 1; m < 20; m++){
        geometry_msgs::Pose end_pose = start_pose;
        end_pose.position.z += r * cos((2*m*PI)/sides);
        end_pose.position.y += r * sin((2*m*PI)/sides);
        end_pose.position.x += 0.0;

        waypoints.push_back(end_pose);
    }

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(pose_target, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);

}

