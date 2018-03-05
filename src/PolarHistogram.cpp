#include <math.h>
#include <vfh_rover/PolarHistogram.h>
#include <vfh_rover/Vehicle.h>
#include <string>
#include <assert.h>
#include <iostream>
#include <cmath>
#include <pcl/point_types.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

Histogram::Histogram(float alpha,
    float ox, float oy, float oz):
  alpha(alpha), ox(ox), oy(oy), oz(oz)
{
  data = new float[getWidth() * getHeight()]();
}

int Histogram::getI(float x, float y) {
  return modulus(floor(atan2(x-ox, y-oy)/alpha), getWidth());
}

int Histogram::getJ(float x, float y, float z) {
  float p = sqrt(pow((x-ox), 2) + pow((y-oy), 2));
  return (atan2(z-oz, p)+M_PI/2)/alpha;
}

// j = 0 = top
float Histogram::getValue(int i, int j) {
  int az = modulus(i, getWidth());
  int el = clip(j, 0, getHeight()-1);
  if (j - el > 0) {
    // if el overflows the top and goes over to the other side
    // j is negative
    // el is 0
    el = getHeight() - (j-el);
    az = modulus(az, getWidth());
  } else if (j - el > 0) {
    // if el overflows the bot and goes over to the other side
    el = -(j-el);
    az = modulus(az+getWidth()/2, getWidth());
  }
  //std::cout << "g: " << (el*getWidth())+az << std::endl;
  return this->data[(el*getWidth())+az];
}

void Histogram::setValue(int i, int j, float val) {
  int az = modulus(i, getWidth());
  int el = clip(j, 0, getHeight()-1);
  if (j - el > 0) {
    // if el overflows the top and goes over to the other side
    // j is negative
    // el is 0
    el = getHeight() - (j-el);
    az = modulus(az, getWidth());
  } else if (j - el > 0) {
    // if el overflows the bot and goes over to the other side
    el = -(j-el);
    az = modulus(az+getWidth()/2, getWidth());
  }
  //std::cout << "HI3 " << i << ", " << j << ", " << getHeight() << std::endl;
  //std::cout << "s: " << (el*getWidth())+az << std::endl;
  this->data[(el*getWidth())+az] = val;
}

void Histogram::addValue(float i, float j, float val) {
  setValue(i, j, getValue(i, j) + val);
}

void Histogram::setValues(float* vals, int x, int y, int width, int height) {
  float el, az;
  for (int i=x; i<x+width; i++) {
    for (int j=y; j<y+height; j++) {
      setValue(i, j, vals[(j-y)*width+(i-x)]);
    }
  }
}

void Histogram::addValues(float* vals, int x, int y, int width, int height) {
  float el, az;
  for (int i=x; i<x+width; i++) {
    for (int j=y; j<y+height; j++) {
      setValue(i, j, getValue(i, j) + vals[(j-y)*width+(i-x)]);
    }
  }
}

void Histogram::addValues(float val, int x, int y, int width, int height) {
  float el, az;
  for (int i=x; i<x+width; i++) {
    for (int j=y; j<y+height; j++) {
      setValue(i, j, getValue(i, j) + val);
    }
  }
}

void Histogram::getValues(float* return_vals, int x, int y, int width, int height) {
  float el, az;
  for (int i=x; i<x+width; i++) {
    for (int j=y; j<y+height; j++) {
      return_vals[(j-y)*width+(i-x)] = getValue(i, j);
    }
  }
}

float Histogram::mean() {
  return sum(data, getWidth()*getHeight()) / getHeight() / getWidth();
}

float Histogram::std() {
  float mean = this->mean();
  float val, total;
  for(int i=0; i<this->getWidth(); i++) {
    for(int j=0; j<this->getHeight(); j++) {
      val = this->getValue(i, j);
      if(val != 0)
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
  std::cout << "Radius is ------------: " << r << " Height is ---------: "<< h << "At elevation: " << elevationF<<std::endl;
  return 2*M_PI*(r*h)/getWidth();
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
      float az = alpha*modulus(i-getWidth()/2, getWidth()) + alpha/2;
      float el = alpha*modulus(j-getHeight()/2, getHeight()) + alpha/2;
      pcl::PointXYZRGB p;
      float sign = (el < M_PI/2) ? 1 : -1;
      p.x = -sign*radius*cos(el)*sin(az)+ox;
      p.y = -sign*radius*cos(el)*cos(az)+oy;
      p.z = sign*radius*sin(el)+oz;
      float color = val < 0 ? 0 : val;
      color = color >= 255 ? 255 : color;
      p.r = p.g = p.b = color;
      pc->points.push_back(p);
    }
  }
  return pc;
}
