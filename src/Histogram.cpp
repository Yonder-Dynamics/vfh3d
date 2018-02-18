#include <math.h>
#include <vfh_rover/Histogram.h>
#include <string>
#include <assert.h>
#include <iostream>
#include <cmath>
#include <pcl/point_types.h>

int modulus(int x, int modY) {
  if (x < 0)
    return (modY + x % modY) % modY;
  else
    return x % modY;
}

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
  //assert(0 < az && az<getWidth() && 0 < el && el<getHeight());
  setValue(az, el, getValue(az, el) + val);
}

void Histogram::addVoxel(float x, float y, float z, float val,
                         float voxel_radius, float maxRange) {
  // For weight calculation
  float dist = sqrt(pow(x-ox, 2) +
                    pow(y-oy, 2) +
                    pow(z-oz, 2));
  float enlargement = floor(asin(voxel_radius/dist)/alpha);
  float a = 0.5;
  float b = 4*(a-1)/pow(maxRange-1, 2);
  float h = val*val*(a-b*(dist-voxel_radius));

  float bz = getI(x, y);
  float be = getJ(x, y, z);
  int voxelCellSize = (int)(enlargement/alpha); // divided by 2
  int az,el;
  for (int i=bz-voxelCellSize; i<bz+voxelCellSize; i++) {
    for (int j=be-voxelCellSize; j<be+voxelCellSize; j++) {
      az = modulus(i, getWidth());
      el = modulus(j, getHeight());
      setValue(az, el, getValue(az, el) + h);
      if (az < 0 or el < 0 or az >= getWidth() or el >= getHeight())
        std::cout << az << ", " << el << ", " << getWidth() << ", " << getHeight() << "i: "<< i << " j: " << j << std::endl;
    }
  }

}

float Histogram::mean() {
  float s = 0;
  for (int j=0; j<this->getHeight(); j++) {
    for (int i=0; i<this->getWidth(); i++) {
      s += getValue(i, j);
    }
  }
  return s / getWidth() / getHeight();
}

int Histogram::getI(float x, float y) {
  int i = (int)floor(atan2(x-ox, y-oy)/alpha + getWidth()/2) % getWidth();
  return i;
}

int Histogram::getJ(float x, float y, float z) {
  float p = sqrt(pow((x-ox), 2) + pow((y-oy), 2));
  float j = ((int)floor(atan2(z-oz, p)/alpha) + getHeight()) % getHeight();
  return j;
}

bool Histogram::isIgnored(float x, float y, float z, float ws) {
  float dist = sqrt(pow((x-ox), 2) +
                    pow((y-oy), 2) +
                    pow((z-oz), 2));
  return dist > ws;
}

int Histogram::getWidth() {
  return int(ceil((2*M_PI)/this->alpha));
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
  float m = mean();
  for (int i=0; i<this->getWidth(); i++) {
    for (int j=0; j<this->getHeight(); j++) {
      float val = getValue(i, j) * (255/m);
      float az = alpha*i + alpha/2;
      float el = alpha*j + alpha/2;
      pcl::PointXYZRGB p;
      float sign = (el < M_PI/2) ? 1 : -1;
      p.x = -sign*radius*cos(el)*sin(az)+ox;
      p.y = -sign*radius*cos(el)*cos(az)+oy;
      p.z = sign*radius*sin(el)+oz;
      float color = val < 0 ? 0 : val;
      color = color > 255 ? 255 : color;
      p.r = p.g = p.b = color;
      pc->points.push_back(p);
    }
  }
  return pc;
}
