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

class PolarHistogram {
 public:
  /** Constructor with width and height parameters **/
  PolarHistogram(float alpha, float ox, float oy, float oz);

  ~Histogram() {
    delete(data);
  }

  int getI(float x, float y);
  int getJ(float x, float y, float z);
  bool isIgnored(float x, float y, float z, float ws);
  int getWidth();
  int getHeight();

  //// Stats
  // Gets number of cells for statistics
  int getN();
  float mean();
  float std();
  float getMeanArea();
  float getArea(int elevation);
  //// For processing histogram
  float getValue(int i, int j);
  void setValue(int i, int j, float val);
  void addValue(float i, float j, float val);
  //// Functions for manipulating windows of values
  void addValues(float* vals, int x, int y, int width, int height);
  void addValues(float val, int x, int y, int width, int height);
  void setValues(float* vals, int x, int y, int width, int height);
  void getValues(float* return_vals, int x, int y, int width, int height);

  std::string displayString();

  RGBPointCloud::Ptr displayCloud(float radius);

 public:
  float* data;
  float alpha, ox, oy, oz;
};
