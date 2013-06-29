/*
 * Description: Set the arms of PR2 to home pose for navigation.
 * Author: Rajesh
 */

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>

void tiltHead(moveit::planning_interface::MoveGroup &head, double value)
{
  head.setJointValueTarget("head_tilt_joint", value);

  head.move();
}

void moveArmsToHomePose(moveit::planning_interface::MoveGroup &arms)
{
  std::map<std::string, double> target;

  target["r_elbow_flex_joint"] = -2.12441;
  //  target["r_forearm_roll_joint"] = -1.4175;
  target["r_shoulder_lift_joint"] = 1.24853;
  target["r_shoulder_pan_joint"] = -1.5;
  //  target["r_upper_arm_roll_joint"] = -1.55669;
  //  target["r_wrist_flex_joint"] = -1.8417;
  //  target["r_wrist_roll_joint"] = 0.21436;

  arms.setJointValueTarget(target);

  while (!arms.move()){
    ROS_INFO("Right arm motion failed. Retrying.");
  }

  target.clear();

  target["l_elbow_flex_joint"] = -1.68339;
  target["l_forearm_roll_joint"] = -1.73434;
  target["l_shoulder_lift_joint"] = 1.24852;
  target["l_shoulder_pan_joint"] = 0.06024;
  target["l_upper_arm_roll_joint"] = 1.78907;
  target["l_wrist_flex_joint"] = -0.0962141;
  target["l_wrist_roll_joint"] = -0.0864407;

  arms.setJointValueTarget(target);

  while (!arms.move()){
    ROS_INFO("Left arm motion failed. Retrying.");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_home_pose");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  //  moveit::planning_interface::MoveGroup head("head");
  //  head.setPlanningTime(60.0);
  //  head.setStartStateToCurrentState();

  //  tiltHead(head, 0.8);

  moveit::planning_interface::MoveGroup lr_arms("arms");
  lr_arms.setPlanningTime(60.0);
  lr_arms.setStartStateToCurrentState();

  moveArmsToHomePose(lr_arms);

  //  tiltHead(head, 0.0);
  return 0;
}
