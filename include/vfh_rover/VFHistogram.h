#pragma once
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <octomap/octomap.h>
#include <vfh_rover/Vehicle.h>
#include <vfh_rover/util.h>
#include <geometry_msgs/Pose.h>
#include <vfh_rover/PolarHistogram.h>
#include <vfh_rover/PathParams.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> RGBPointCloud;

class VFHistogram : public PolarHistogram {
 public:
  /** Constructor with width and height parameters **/
  VFHistogram(boost::shared_ptr<octomap::OcTree> tree, Vehicle &v,
              float maxRange, float alpha);
  bool isIgnored(float x, float y, float z, float ws);
  float getArea(int elevation);
  // For filling histogram
  void addVoxel(float x, float y, float z, float val,
                float voxel_radius, float maxRange);
  void addVoxel(float x, float y, float z, float val);
  void checkTurning(float x, float y, float z, float val,
                    Vehicle &v, float voxel_radius);
  void binarize(int range);
  std::vector<geometry_msgs::Pose> findPaths(int width, int height);
  geometry_msgs::Pose* optimalPath(Vehicle &v, geometry_msgs::Pose goal,
                                   PathParams p, std::vector<geometry_msgs::Pose>* openPoses);
};
