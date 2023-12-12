#include "../include/square.hpp"
#include <math.h>

#define PI 3.14159265

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "square");
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
    planning_options.acceleration_scaling_factor = 0.15;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    //Vertical x-z plane
    /*moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = -1.8326;
    joint_targets["shoulder_lift_joint"] = -1.39626;
    joint_targets["shoulder_pan_joint"] = -1.93732;
    joint_targets["wrist_1_joint"] = 0.0;
    joint_targets["wrist_2_joint"] = 2.0;
    joint_targets["wrist_3_joint"] = 0.0;

    bool joint_plan_success;
    std::string reference_frame = "world";
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }


    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_1;

    // Get the start Pose
    geometry_msgs::Pose start_pose_1 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_1 = start_pose_1;
    end_pose_1.position.x += 0;
    end_pose_1.position.y -= 0;
    end_pose_1.position.z += 0.1;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(end_pose_1);

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose_1, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    //n.setParam("/record_pose", false);
   

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_2;

    // Get the start Pose
    geometry_msgs::Pose start_pose_2 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_2 = start_pose_2;
    end_pose_2.position.x += 0.1;
    end_pose_2.position.y -= 0;
    end_pose_2.position.z -= 0;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints2;
    waypoints2.push_back(end_pose_2);

    moveit_msgs::RobotTrajectory trajectory2;
    trajectory2 = ArmController::planCartesianPath(start_pose_2, waypoints2, reference_frame, arm_move_group);

    //n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory2);
    //n.setParam("/record_pose", false);


    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_3;

    // Get the start Pose
    geometry_msgs::Pose start_pose_3 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_3 = start_pose_3;
    end_pose_3.position.x += 0;
    end_pose_3.position.y -= 0;
    end_pose_3.position.z -= 0.1;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints3;
    waypoints3.push_back(end_pose_3);

    moveit_msgs::RobotTrajectory trajectory3;
    trajectory3 = ArmController::planCartesianPath(start_pose_3, waypoints3, reference_frame, arm_move_group);

    //n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory3);
    //n.setParam("/record_pose", false);


    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_4;

    // Get the start Pose
    geometry_msgs::Pose start_pose_4 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_4 = start_pose_4;
    end_pose_4.position.x -= 0.1;
    end_pose_4.position.y -= 0;
    end_pose_4.position.z -= 0;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints4;
    waypoints4.push_back(end_pose_4);

    moveit_msgs::RobotTrajectory trajectory4;
    trajectory4 = ArmController::planCartesianPath(start_pose_4, waypoints4, reference_frame, arm_move_group);

    //n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory4);
    n.setParam("/record_pose", false);*/

    //Horizontal x-y plane
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = 1.5708;
    joint_targets["shoulder_lift_joint"] = -0.785398;
    joint_targets["shoulder_pan_joint"] = 0.837758;
    joint_targets["wrist_1_joint"] = 3.90954;
    joint_targets["wrist_2_joint"] = -1.6057;
    joint_targets["wrist_3_joint"] = -3.89208;

    bool joint_plan_success;
    std::string reference_frame = "world";
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }


    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_1;

    // Get the start Pose
    geometry_msgs::Pose start_pose_1 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_1 = start_pose_1;
    end_pose_1.position.x += 0;
    end_pose_1.position.y += 0.1;
    end_pose_1.position.z -= 0;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(end_pose_1);

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose_1, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    //n.setParam("/record_pose", false);
   

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_2;

    // Get the start Pose
    geometry_msgs::Pose start_pose_2 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_2 = start_pose_2;
    end_pose_2.position.x += 0.1;
    end_pose_2.position.y -= 0;
    end_pose_2.position.z -= 0;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints2;
    waypoints2.push_back(end_pose_2);

    moveit_msgs::RobotTrajectory trajectory2;
    trajectory2 = ArmController::planCartesianPath(start_pose_2, waypoints2, reference_frame, arm_move_group);

    //n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory2);
    //n.setParam("/record_pose", false);


    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_3;

    // Get the start Pose
    geometry_msgs::Pose start_pose_3 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_3 = start_pose_3;
    end_pose_3.position.x += 0;
    end_pose_3.position.y -= 0.1;
    end_pose_3.position.z -= 0;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints3;
    waypoints3.push_back(end_pose_3);

    moveit_msgs::RobotTrajectory trajectory3;
    trajectory3 = ArmController::planCartesianPath(start_pose_3, waypoints3, reference_frame, arm_move_group);

    //n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory3);
    //n.setParam("/record_pose", false);


    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_4;

    // Get the start Pose
    geometry_msgs::Pose start_pose_4 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_4 = start_pose_4;
    end_pose_4.position.x -= 0.1;
    end_pose_4.position.y -= 0;
    end_pose_4.position.z -= 0;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints4;
    waypoints4.push_back(end_pose_4);

    moveit_msgs::RobotTrajectory trajectory4;
    trajectory4 = ArmController::planCartesianPath(start_pose_4, waypoints4, reference_frame, arm_move_group);

    //n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory4);
    n.setParam("/record_pose", false);


    //Largest not horizontal square
    /*float angle = PI/2;
    float r = 0.45;

    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;

    geometry_msgs::Pose pose_target = arm_move_group.getCurrentPose().pose;
    pose_target.position.x = 0.0;
    pose_target.position.y = r * cos(angle);
    pose_target.position.z = 0.898 + (r * sin(angle));

    bool pose_plan_success;
    std::string reference_frame = "world";
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target, reference_frame, pose_plan);

    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan);
    }

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_1;

    // Get the start Pose
    geometry_msgs::Pose start_pose_1 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_1 = start_pose_1;
    end_pose_1.position.x -= 0;
    end_pose_1.position.y -= 2 * r * cos(angle);
    end_pose_1.position.z -= 2 * r * sin(angle);

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints1;
    waypoints1.push_back(end_pose_1);

    moveit_msgs::RobotTrajectory trajectory1;
    trajectory1 = ArmController::planCartesianPath(start_pose_1, waypoints1, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory1);
    n.setParam("/record_pose", false);

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_2;

    // Get the start Pose
    geometry_msgs::Pose start_pose_2 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_2 = start_pose_2;
    end_pose_2.position.x += r;
    end_pose_2.position.y += r * cos(angle);
    end_pose_2.position.z += r * sin(angle);

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints2;
    waypoints2.push_back(end_pose_2);

    moveit_msgs::RobotTrajectory trajectory2;
    trajectory2 = ArmController::planCartesianPath(start_pose_2, waypoints2, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory2);
    n.setParam("/record_pose", false);

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_3;

    // Get the start Pose
    geometry_msgs::Pose start_pose_3 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_3 = start_pose_3;
    end_pose_3.position.x -= 2 * r;
    end_pose_3.position.y -= 0;
    end_pose_3.position.z -= 0;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints3;
    waypoints3.push_back(end_pose_3);

    moveit_msgs::RobotTrajectory trajectory3;
    trajectory3 = ArmController::planCartesianPath(start_pose_3, waypoints3, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory3);
    n.setParam("/record_pose", false);

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan_4;

    // Get the start Pose
    geometry_msgs::Pose start_pose_4 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose_4 = start_pose_4;
    end_pose_4.position.x += r;
    end_pose_4.position.y += r * cos(angle);
    end_pose_4.position.z += r * sin(angle);

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints4;
    waypoints4.push_back(end_pose_4);

    moveit_msgs::RobotTrajectory trajectory4;
    trajectory4 = ArmController::planCartesianPath(start_pose_4, waypoints4, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory4);
    n.setParam("/record_pose", false);*/
}