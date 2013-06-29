/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */
/* Modified by Rajesh */

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

void pointHeadAtObject(const std::string &ref_frame, const std::string &pointing_frame)
{
  // tilt the pr2 head to point to the table
  actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> point_head_client("/head_traj_controller/point_head_action", true);

  while (!point_head_client.waitForServer(ros::Duration(10.0))){
    ROS_INFO("Waiting for the point_head_action server to come up");
  }

  pr2_controllers_msgs::PointHeadGoal goal;

  geometry_msgs::PointStamped point;
  point.header.frame_id = ref_frame;
  point.point.x = 0.6;
  point.point.y = 0.0;
  point.point.z = 0.625;

  goal.target = point;
  goal.pointing_frame = pointing_frame;
  goal.min_duration = ros::Duration(0.5);
  goal.max_velocity = 1.0;

  point_head_client.sendGoal(goal);
  point_head_client.waitForResult(ros::Duration(10.0));
}

void setGripper(double value, double effort = -1.0)
{
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> gripper_client("/r_gripper_controller/gripper_action", true);

  while (!gripper_client.waitForServer(ros::Duration(10.0))){
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action action to come up");
  }

  pr2_controllers_msgs::Pr2GripperCommandGoal goal;
  goal.command.position = value;
  goal.command.max_effort = effort;

  gripper_client.sendGoal(goal);
  gripper_client.waitForResult(ros::Duration(10.0));
}

void openGripper()
{
  setGripper(1.0);
}

void closeGripper()
{
  setGripper(0.0, 50);
}

void liftTorso()
{
  actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> torso_client("torso_controller/position_joint_action", true);

  while (!torso_client.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the torso action server to come up");
  }

  pr2_controllers_msgs::SingleJointPositionGoal goal;
  goal.position = 0.15; // all the way up is 2.0
  goal.min_duration = ros::Duration(2.0);
  goal.max_velocity = 1.0;

  torso_client.sendGoal(goal);
  torso_client.waitForResult();
}

void pick(moveit::planning_interface::MoveGroup &group)
{
  std::vector<manipulation_msgs::Grasp> grasps;
  
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = 0.52;
  p.pose.position.y = 0.01;
  p.pose.position.z = 0.625;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  manipulation_msgs::Grasp g;
  g.grasp_pose = p;
  
  g.approach.direction.vector.x = 1.0;
  g.approach.direction.header.frame_id = "r_wrist_roll_link";
  g.approach.min_distance = 0.2;
  g.approach.desired_distance = 0.4;

  g.retreat.direction.header.frame_id = "base_footprint";
  g.retreat.direction.vector.z = 1.0;
  g.retreat.min_distance = 0.0;
  g.retreat.desired_distance = 0.0;

  g.pre_grasp_posture.name.resize(1, "r_gripper_joint");
  g.pre_grasp_posture.position.resize(1);
  g.pre_grasp_posture.position[0] = 1.0;
  
  g.grasp_posture.name.resize(1, "r_gripper_joint");
  g.grasp_posture.position.resize(1);
  g.grasp_posture.position[0] = 0.9;
  
  g.max_contact_force = 10.0;
  
  grasps.push_back(g);
  group.setSupportSurfaceName("table");

  ROS_INFO("Pickup state: Start approaching");

  while (!group.pick("part", grasps)){
    ROS_INFO("Pickup state: Pick failed. Try again");
    group.setStartStateToCurrentState();
  }

  ROS_INFO("Pickup state: Approaching finished. Start closing gripper");

  closeGripper();

  ROS_INFO("Pickup state: Gripper closed. Start lifting Torso");

  liftTorso();

  ROS_INFO("Pickup state: Torso lifted. Pickup finished");
}

void place(moveit::planning_interface::MoveGroup &group)
{
  std::vector<manipulation_msgs::PlaceLocation> loc;
  
  geometry_msgs::PoseStamped p; 
  p.header.frame_id = "base_footprint";
  p.pose.position.x = 0.7;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.5;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  manipulation_msgs::PlaceLocation g;
  g.place_pose = p;  
  
  g.approach.direction.vector.z = -1.0;
  g.retreat.direction.vector.x = -1.0;
  g.retreat.direction.header.frame_id = "base_footprint";
  g.approach.direction.header.frame_id = "r_wrist_roll_link";
  g.approach.min_distance = 0.1;
  g.approach.desired_distance = 0.2;
  g.retreat.min_distance = 0.1;
  g.retreat.desired_distance = 0.25;
  
  g.post_place_posture.name.resize(1, "r_gripper_joint");
  g.post_place_posture.position.resize(1);
  g.post_place_posture.position[0] = 1;
  
  loc.push_back(g);
  group.setSupportSurfaceName("table");


  // add path constraints
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = p.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0; 
  group.setPathConstraints(constr);  
  group.setPlannerId("RRTConnectkConfigDefault");
  
  group.place("part", loc);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

  ros::WallDuration(1.0).sleep();  

  //pointHeadAtObject("base_footprint", "high_def_frame");

  // moveit stuff
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "map";
  
  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.2;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.0;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.55;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.98;
  co.primitive_poses[0].position.y = 0.0;  
  co.primitive_poses[0].position.z = 0.275;
  co.primitive_poses[0].orientation.w = 1.0;
  pub_co.publish(co);

  ROS_INFO("Added collision obejct: table");

  // object to pick
  co.id = "part";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.08;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.08;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.15;
  
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = 0.01;  
  co.primitive_poses[0].position.z = 0.625;
  pub_co.publish(co);

  ROS_INFO("Added collision object: part");
  
  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();
  
  moveit::planning_interface::MoveGroup group("right_arm");
  group.setPlanningTime(60.0);
  group.setStartStateToCurrentState();

  pick(group);

  //ros::WallDuration(1.0).sleep();
  
  //place(group);  

  ros::waitForShutdown();
  return 0;
}
