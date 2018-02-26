#pragma once
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <octomap/octomap.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> RGBPointCloud;

class Histogram {
 public:
  /** Constructor with width and height parameters **/
  Histogram(float alpha, float ox, float oy, float oz);

  int getI(float x, float y);
  int getJ(float x, float y, float z);
  int getE(float x, float y);
  int getZ(float x, float y, float z);
  bool isIgnored(float x, float y, float z, float ws);

  int getWidth();
  int getHeight();
  float getAlpha();
  float mean();
  float std();
  float getMeanArea();
  float getArea(int elevation);

  // For processing histogram
  float getValue(int i, int j);
  void setValue(int i, int j, float val);
  // For filling histogram
  void addValue(float x, float y, float z, float val);
  void addVoxel(float x, float y, float z, float val,
                float voxel_radius, float maxRange);

  std::string displayString();

  RGBPointCloud::Ptr displayCloud(float radius);

 private:
  float* data;
  float alpha, ox, oy, oz;
};
