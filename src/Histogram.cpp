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

int clip(int x, int min, int max) {
  int c = (x > max) ? max : x;
  return (c < min) ? min : c;
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
  int voxelCellSize = 1; //(int)(enlargement/alpha); // divided by 2
  int az,el;
  //std::cout << "bz: " << bz << ", be: " << be << ", vc: " << voxelCellSize << std::endl;
  std::cout << "bz: " << bz << ", be: " << be << ", vc: " << voxelCellSize << std::endl;
  for (int i=bz-voxelCellSize; i<bz+voxelCellSize; i++) {
    for (int j=be-voxelCellSize; j<be+voxelCellSize; j++) {
      az = modulus(i, getWidth());
      //el = modulus(j, getHeight());
      //az = clip(i, 2, getWidth()-3);
      el = clip(j, 0, getHeight());
      if (el - j < 0) {
        std::cout << el << " h: " << getHeight() << " j: " << j << ", " << el - j << std::endl;
        el = j - el;
        az = modulus(az+getWidth()/2, getWidth());
      } else if (el - j > 0) {
        el = getHeight() - j - el - 1;
        az = modulus(az+getWidth()/2, getWidth());
      }
      setValue(az, el, getValue(az, el) + h);
      //std::cout << el << " h: " << getHeight() << " j: " << j << std::endl;
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

float Histogram::std() {
  float mean = this->mean();
  float val, total;
  for(int i=0; i<this->getWidth(); i++) {
    for(int j=0; j<this->getHeight(); j++) {
      val = this->getValue(i, j);
      total += pow((val - mean), 2);
    }
  }

  total /= (this->getWidth() * this->getHeight());
  total = pow(total, 0.5);
  return total;
}

float Histogram::getMeanArea() {
  float radius = 1;

  float area = 4*M_PI*pow(radius, 2);
  return area/(this->getWidth() *  this->getHeight());
}

float Histogram::getArea(int elevation) {
  float r1, r2, h1, h2, r, h;
  float elevationF = ((getHeight()/2 - elevation) >= 0 ? elevation : abs(getHeight() - elevation));
  h1 = cos(this->alpha * elevationF);
  h2 = cos(this->alpha * (elevationF+1));
  r1 = sin(this->alpha * elevationF);
  r2 = sin(this->alpha * (elevationF+1));
  
  r = (r1+r2)/2;
  h = (h1-h2);
//  std::cout << "Radius is ------------: " << r << " Height is ---------: "<< h << "At elevation: " << elevationF<<std::endl;
  return 2*M_PI*(r*h)/getWidth();
}

int Histogram::getI(float x, float y) {
  int i = modulus(floor(atan2(x-ox, y-oy)/alpha + getWidth()/2), getWidth());
  return i;
}

int Histogram::getJ(float x, float y, float z) {
  float p = sqrt(pow((x-ox), 2) + pow((y-oy), 2));
  //float j = modulus(-floor(atan2(z-oz, p)/alpha - getHeight()/2), getHeight());
  //int j = floor(getHeight()/2 - atan2(z-oz, p)/alpha);
  float at = atan2(z-oz, p)/alpha;
  if (at < 0) // modifications for the bottom half
    return floor(getHeight()/2 - at - 1.5);
  else // modifications for the top half
    return clip(at, 0, 2*getHeight());
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
      //color = color == 255 ? 255 : color;
      p.r = p.g = p.b = color;
      pc->points.push_back(p);
    }
  }
  return pc;
}
