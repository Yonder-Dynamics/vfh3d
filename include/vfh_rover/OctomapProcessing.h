#pragma once
#include <vfh_rover/Vehicle.h>
#include <vfh_rover/HistogramUpdate.h>
#include <octomap/octomap.h>
#include <octomap/math/Vector3.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

class OctomapProcessing {
 public:
  HistogramUpdate* hu;
  Vehicle vehicle;
  float maxRange;
  ros::Publisher pub;
  OctomapProcessing(float alpha, Vehicle v, float maxRange, ros::NodeHandle n);
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
};
