#include <math.h>
#include <vfh_rover/Histogram.h>
#include <string>
#include <assert.h>
#include <iostream>
#include <cmath>
#include <pcl/point_types.h>

Histogram::Histogram(float alpha,
                     float ox, float oy, float oz):
  alpha(alpha), ox(ox), oy(oy), oz(oz)
{
  data = new float[getWidth() * getHeight()]();
}

float Histogram::getValue(int i, int j) {
  return this->data[(j*getWidth())+i];
}

void Histogram::setValue(int i, int j, float val) {
  this->data[(j*getWidth())+i] = val;
}

void Histogram::addValue(float x, float y, float z, float val) {
  int az = getI(x, y);
  int el = getJ(x, y, z);
  assert(az<getWidth() && el<getHeight());
  setValue(az, el, getValue(az, el) + val);
}

void Histogram::addVoxel(float x, float y, float z, float val,
                         float voxel_radius, float maxRange) {
  // For weight calculation
  float dist = sqrt(pow(x-ox, 2) +
                    pow(y-oy, 2) +
                    pow(z-oz, 2));
  float enlargement = floor(atan2(voxel_radius, dist)/alpha)/alpha;
  float a = 0.5;
  float b = 4*(a-1)/pow(maxRange-1, 2);
  float h = val*val*(a-b*(dist-voxel_radius));

  float be = getI(x, y);
  float bz = getJ(x, y, z);
  int voxelCellSize = (int)ceil(enlargement); // divided by 2
  for (int i=be-voxelCellSize; i<be+voxelCellSize; i++) {
    for (int j=bz-voxelCellSize; j<bz+voxelCellSize; j++) {
      setValue(i % getWidth(), j % getHeight(), getValue(i, j) + h);
    }
  }
}

int Histogram::getE(float x, float y) {
  return atan2(x-ox, y-oy)/alpha + getWidth()/2;
}

int Histogram::getZ(float x, float y, float z) {
  float p = sqrt(pow((x-ox), 2) + pow((y-oy), 2));
  return atan2(z-oz, p)/alpha;
}

int Histogram::getI(float x, float y) {
  int i = (int)floor(atan2(x-ox, y-oy)/alpha + getWidth()/2);
  return i;
}

int Histogram::getJ(float x, float y, float z) {
  float p = sqrt(pow((x-ox), 2) + pow((y-oy), 2));
  float j = (int)floor(atan2(z-oz, p)/alpha);
  //std::cout << z << ", " << j << std::endl;
  return j;
}

bool Histogram::isIgnored(float x, float y, float z, float ws) {
  float dist = sqrt(pow((x-ox), 2) +
                    pow((y-oy), 2) +
                    pow((z-oz), 2));
  return dist > ws;
}

int Histogram::getWidth() {
  return int(((2*M_PI)/this->alpha));
}

int Histogram::getHeight() {
  return int(ceil((M_PI)/this->alpha));
}

std::string Histogram::displayString() {
  std::string s;
  for (int j=0; j<this->getHeight(); j++) {
    for (int i=0; i<this->getWidth(); i++) {
      s.append(std::to_string(this->getValue(i, j)));
      s.append(", ");
    }
    s.append("\n");
  }
  return s;
}

RGBPointCloud::Ptr Histogram::displayCloud(float radius) {
  RGBPointCloud::Ptr pc (new RGBPointCloud);
  for (int i=0; i<this->getWidth(); i++) {
    for (int j=0; j<this->getHeight(); j++) {
      float val = getValue(i, j);
      float az = alpha*i + alpha/2;
      float el = alpha*j + alpha/2;
      pcl::PointXYZRGB p;
      float sign = (el < M_PI/2) ? 1 : -1;
      p.x = cos(el)*sin(az)+ox;
      p.y = cos(el)*cos(az)+oy;
      p.z = sign*radius*sin(el)+oz;
      float color = log(abs(val))*40;
      p.r = p.g = p.b = color;
      pc->points.push_back(p);
    }
  }
  return pc;
}
