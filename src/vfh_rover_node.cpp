#include <ros/ros.h>
#include <vfh_rover/OctomapProcessing.h>
#include <vfh_rover/HistogramUpdate.h>
#include <iostream>

using namespace octomap;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "vfh_rover_node");
  ros::NodeHandle n;
  OctomapProcessing op;
  ros::Subscriber sub = n.subscribe("/octomap_full", 3,
      &OctomapProcessing::octomapCallback, &op);
  HistogramUpdate* up = new HistogramUpdate(0.5, 5, 5, 5);
  octomath::Vector3 center = octomath::Vector3(0,0,0);
  up->build(op.getMap(), center);
  ros::spin();
  return 0;
}
