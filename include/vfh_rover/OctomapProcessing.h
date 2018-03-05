#pragma once
#include <vfh_rover/Vehicle.h>
#include <vfh_rover/HistogramUpdate.h>
#include <octomap/octomap.h>
#include <octomap/math/Vector3.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

class OctomapProcessing {
 public:
  HistogramUpdate* hu;
  Vehicle vehicle;
  float maxRange;
  bool gotGoal, gotOcto;
  boost::shared_ptr<octomap::OcTree> tree;
  geometry_msgs::Pose goal;
  ros::Publisher histogram_pub, pose_pub, next_pose_pub;
  OctomapProcessing(float alpha, Vehicle v,
                    float maxRange, ros::NodeHandle n);
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void process();
  float getHeading();
};
