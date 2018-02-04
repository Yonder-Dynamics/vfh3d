#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf/transform_datatypes.h>
#include <iostream>

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
  octomap::AbstractOcTree* octree = octomap_msgs::msgToMap(*msg);
  std::cout << "Got it" << std::endl;
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "vfh_rover_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/octomap_full", 3, octomapCallback);
  ros::spin();
  return 0;
}
