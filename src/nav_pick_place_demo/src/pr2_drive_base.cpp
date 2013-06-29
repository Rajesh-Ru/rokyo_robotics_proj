/*
 * Description: Move the base of PR2.
 * Author: Rajesh
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_drive_base");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/base_controller/command", 1);

  geometry_msgs::Twist base_cmd;
  double baseTime; // in second
  double curTime; // in second
  ros::Rate r(10); // 10 Hz

  do{
    baseTime = curTime = ros::Time::now().toSec();
  }while (baseTime == 0.0);

  //  printf("baseTime = %f, curTime = %f\n", baseTime, curTime);

  if (strcmp(argv[1], "-l") == 0)
    base_cmd.linear.x = atof(argv[2]); // m/s
  else if (strcmp(argv[1], "-r") == 0)
    base_cmd.angular.z = atof(argv[2]); // radian/s
  else{
    ROS_INFO("Invalid command!");
    return 1;
  }

  double duration = atof(argv[3]);

  ROS_INFO("Start driving base.");

  while (curTime - baseTime <= duration){
    cmd_vel_pub.publish(base_cmd);
    r.sleep();
    curTime = ros::Time::now().toSec();
    //    printf("baseTime = %f, curTime = %f\n", baseTime, curTime);
  }

  ROS_INFO("Finished driving base.");
}
