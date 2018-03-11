#include <ros/ros.h>
#include <vfh_rover/OctomapProcessing.h>
#include <vfh_rover/Vehicle.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

using namespace octomap;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "vfh_rover_node");
  ros::NodeHandle n;

  Vehicle v = {-5,0,1, 0.5,0.5,0.7, 0.1};
  OctomapProcessing op (M_PI/30, v, 3, n);
  ros::Subscriber sub1 = n.subscribe("/octomap_full", 3, &OctomapProcessing::octomapCallback, &op);
  ros::Subscriber sub2 = n.subscribe("/goal", 3, &OctomapProcessing::goalCallback, &op);
  ros::Subscriber sub3 = n.subscribe("/vehiclePose", 3, &OctomapProcessing::poseCallback, &op);
  ros::spin();
  return 0;
}
