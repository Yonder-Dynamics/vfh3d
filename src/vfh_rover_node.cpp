#include <ros/ros.h>
#include <vfh_rover/OctomapProcessing.h>
#include <vfh_rover/HistogramUpdate.h>
#include <vfh_rover/Vehicle.h>
#include <iostream>

using namespace octomap;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "vfh_rover_node");
  ros::NodeHandle n;
  Vehicle v = {0,0,6, 1,1,1, 2};
  OctomapProcessing op (M_PI/10, v, 5, n);
  ros::Subscriber sub = n.subscribe("/octomap_full", 3,
      &OctomapProcessing::octomapCallback, &op);
  ros::spin();
  return 0;
}
