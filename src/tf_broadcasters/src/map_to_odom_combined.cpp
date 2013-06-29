#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void poseCb(const nav_msgs::OdometryConstPtr &msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform t;

  t.setOrigin(tf::Vector3(msg->pose.pose.position.x,
			  msg->pose.pose.position.y,
			  msg->pose.pose.position.z));
  t.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,
			       msg->pose.pose.orientation.y,
			       msg->pose.pose.orientation.z,
			       msg->pose.pose.orientation.w));

  br.sendTransform(tf::StampedTransform(t, ros::Time::now(),
					"map", "odom_combined"));
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_to_odom_combined");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/base_pose_ground_truth", 10, &poseCb);

  ros::spin();
  return 0;
}
