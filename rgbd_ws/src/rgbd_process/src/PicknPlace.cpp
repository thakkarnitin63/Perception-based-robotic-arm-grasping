
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Grasp.h> 
#include <gazebo_msgs/GetModelState.h>

// #include <shapes.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
// #include <shapes/shape_operations.h>

// #include <shape_tools/construct_mesh.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h> 
// #include <gpd_ros/GraspConfigList.h>
// #include "moveit_visual_tools.h" 
#include <cmath> 
#include <moveit_visual_tools/moveit_visual_tools.h>

const double tau = 2 * M_PI;

geometry_msgs::Pose calculated_mean_pose;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    calculated_mean_pose=msg->pose;
    std::cout<<calculated_mean_pose.position;
}

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group,moveit_visual_tools::MoveItVisualTools& visual_tools)
{
// BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in 
  //your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
  // transform from `"panda_link8"` to the palm of the end effector.
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  //-0.008, -1.521, -3.132
  orientation.setRPY(1.470, 0.800, 1.541);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x=calculated_mean_pose.position.x;
  grasps[0].grasp_pose.pose.position.y=calculated_mean_pose.position.y;
  grasps[0].grasp_pose.pose.position.z=calculated_mean_pose.position.z;
//   // std::cout<<calculated_mean_pose.position.x<<std::endl;
//   // x: 0.37061
// // y: 0.0613297
// // z: 0.0909926

  grasps[0].pre_grasp_approach.direction.header.frame_id="panda_link0";
// //   // //dieection is set as positive x axis;
  
  grasps[0].pre_grasp_approach.direction.vector.x =1.0;
  // grasps[0].pre_grasp_approach.direction.vector.z =0.8;
//   // grasps[0].pre_grasp_approach.direction.vector.
//   // grasps[0].pre_grasp_approach.direction.vector.x

  grasps[0].pre_grasp_approach.min_distance=0.095;
  grasps[0].pre_grasp_approach.desired_distance=0.115;

//   // // setting pose grasp retreat;

  grasps[0].post_grasp_retreat.direction.header.frame_id="panda_link0";
//   // // direction is set as positive z axis;
  grasps[0].post_grasp_retreat.direction.vector.z=1.0;
  // grasps[0].pre_grasp_approach.direction.vector.x=0.5;
  // grasps[0].pre_grasp_approach.direction.vector.x
  grasps[0].post_grasp_retreat.min_distance=0.05;
  grasps[0].post_grasp_retreat.desired_distance = 0.2;

//   //end effector pose;
  openGripper(grasps[0].pre_grasp_posture);

  closedGripper(grasps[0].grasp_posture);

  move_group.pick("object",grasps);


}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
     // Create vector to hold 1 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);
    collision_objects[0].header.frame_id = "panda_link0";
    collision_objects[0].id = "object";

    shapes::Mesh* mesh = shapes::createMeshFromResource("file:///home/nitin/BottleModel/mesh/BottleModel.dae");
    shape_msgs::Mesh mesh_msg;
    // shapes::ShapeMsg mesh_msg;
    shapes::ShapeMsg mesh_msg_shape;
    shapes::constructMsgFromShape(mesh, mesh_msg_shape);
    mesh_msg=boost::get<shape_msgs::Mesh>(mesh_msg_shape);
    collision_objects[0].meshes.push_back(mesh_msg);

    ros::NodeHandle nh;
    ros::ServiceClient getModelStateClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState getModelState;
    getModelState.request.model_name = "BottleModel"; // Replace with your model's name
    if (getModelStateClient.call(getModelState)) {
        geometry_msgs::Pose model_pose = getModelState.response.pose;
        collision_objects[0].mesh_poses.push_back(model_pose);
    } else {
        ROS_ERROR("Failed to call service /gazebo/get_model_state.");
        return;
    }
    
    // Set the operation type and apply the collision object
    collision_objects[0].operation = collision_objects[0].ADD;
    planning_scene_interface.applyCollisionObjects(collision_objects);

}



int main(int argc, char** argv)
{
    ros::init(argc,argv,"PicknPlaceNode");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Subscriber sub=nh.subscribe("/ComputedCentroid",10,poseCallback);

    
    // Get the model's pose from the Gazebo service response

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(45.0);
    // group.setPlannerId("geometric::RRTConnect");
    group.setNumPlanningAttempts(4);

    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    ROS_INFO("Visual tools initialized.");

    // group.setMaxStep(0.01);
    // move_group.setPlannerParams("panda_arm", {{"range", 0.01}}); 
    addCollisionObjects(planning_scene_interface);
    // collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    // acm.setEntry("panda_link6", "object", true);
    // acm.setEntry("panda_link7", "object", true);
    // planning_scene_interface.setAllowedCollisionMatrix(acm);
     // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();
    //subscribe to mean position
    pick(group,visual_tools);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // my_plan = group.plan();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success) {
      ROS_ERROR("Planning failed");
      return 1;
    }

    robot_trajectory::RobotTrajectoryPtr robot_trajectory(new robot_trajectory::RobotTrajectory(group.getCurrentState()->getRobotModel(), "panda_arm"));
    robot_trajectory->setRobotTrajectoryMsg(*group.getCurrentState(), my_plan.trajectory_);

    moveit_msgs::RobotTrajectory robot_trajectory_msg;
    robot_trajectory->getRobotTrajectoryMsg(robot_trajectory_msg, group.getActiveJoints());

    // visual_tools.publishTrajectoryLine(robot_trajectory_msg);
    visual_tools.publishTrajectoryLine(robot_trajectory, group.getCurrentState()->getLinkModel("panda_link8"), rviz_visual_tools::BLUE);

    visual_tools.trigger();

    
    ros::waitForShutdown();

    return 0;


    //
}