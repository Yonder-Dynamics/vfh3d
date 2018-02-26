#pragma once
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <octomap/octomap.h>
#include <vfh_rover/Vehicle.h>
#include <vfh_rover/util.h>
#include <geometry_msgs/Pose.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> RGBPointCloud;

class Histogram {
 public:
  /** Constructor with width and height parameters **/
  Histogram(float alpha, float ox, float oy, float oz);

  int getI(float x, float y);
  int getJ(float x, float y, float z);
  bool isIgnored(float x, float y, float z, float ws);

  int getWidth();
  int getHeight();
  float getAlpha();
  float mean();

  // For processing histogram
  float getValue(int i, int j);
  void setValue(int i, int j, float val);
  void addValue(float i, float j, float val);
  // Functions for manipulating windows of values
  void addValues(float* vals, int x, int y, int width, int height);
  void addValues(float val, int x, int y, int width, int height);
  void setValues(float* vals, int x, int y, int width, int height);
  void getValues(float* return_vals, int x, int y, int width, int height);
  // For filling histogram
  void addVoxel(float x, float y, float z, float val,
                float voxel_radius, float maxRange);
  void addVoxel(float x, float y, float z, float val);
  void checkTurning(float x, float y, float z, float val,
                    Vehicle v, float voxel_radius);
  std::vector<geometry_msgs::Pose> findPaths(int width, int height);

  std::string displayString();

  RGBPointCloud::Ptr displayCloud(float radius);

 private:
  float* data;
  float alpha, ox, oy, oz;
};
