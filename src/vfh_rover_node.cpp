#include <ros/ros.h>
#include <vfh_rover/OctomapProcessing.h>
#include <iostream>

int main(int argc, char ** argv) {
  ros::init(argc, argv, "vfh_rover_node");
  ros::NodeHandle n;
  OctomapProcessing op;
  ros::Subscriber sub = n.subscribe("/octomap_full", 3,
      &OctomapProcessing::octomapCallback, &op);
  ros::spin();
  return 0;
}
