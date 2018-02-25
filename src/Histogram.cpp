#include <math.h>
#include <vfh_rover/Histogram.h>
#include <vfh_rover/Vehicle.h>
#include <string>
#include <assert.h>
#include <iostream>
#include <cmath>
#include <pcl/point_types.h>

void scalar_add(float* vals, float val, float* ret_val, int count) {
  for (int i = 0; i < count; i++) {
    ret_val[i] = vals[i] + val;
  }
}

void add(float* x, float* y, float* ret_val, int count) {
  for (int i = 0; i < count; i++) {
    ret_val[i] = x[i] + y[i];
  }
}

void add(float* x, float y, float* ret_val, int count) {
  for (int i = 0; i < count; i++) {
    ret_val[i] = x[i] + y;
  }
}

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

void Histogram::addValue(float i, float j, float val) {
  setValue(i, j, getValue(i, j) + val);
}

void Histogram::setValues(float* vals, int x, int y, int width, int height) {
  float el, az;
  for (int i=x; i<x+width; i++) {
    for (int j=y; j<y+height; j++) {
      az = modulus(i, getWidth());
      //el = modulus(j, getHeight());
      //az = clip(i, 2, getWidth()-3);
      el = clip(j, 0, getHeight());
      if (el - j < 0) {
        el = j - el;
        az = modulus(az+getWidth()/2, getWidth());
      } else if (el - j > 0) {
        el = getHeight() - j - el - 1;
        az = modulus(az+getWidth()/2, getWidth());
      }
      setValue(az, el, vals[(j-y)*width+(i-x)]);
    }
  }
}

void Histogram::addValues(float* vals, int x, int y, int width, int height) {
  float el, az;
  for (int i=x; i<x+width; i++) {
    for (int j=y; j<y+height; j++) {
      //az = modulus(i, getWidth());
      az = clip(i, 0, getWidth());
      //el = modulus(j, getHeight());
      //az = clip(i, 2, getWidth()-3);
      el = clip(j, 0, getHeight());
      if (el - j < 0) {
        el = 0;// j - el;
        az = 0;//modulus(az+getWidth()/2, getWidth());
      } else if (el - j > 0) {
        el = 0; //getHeight() - j - el - 1;
        az = 0;//modulus(az+getWidth()/2, getWidth());
      }
      setValue(az, el, getValue(az, el) + vals[(j-y)*width+(i-x)]);
    }
  }
}

void Histogram::addValues(float val, int x, int y, int width, int height) {
  std::cout << "A" << std::endl;
  float el, az;
  for (int i=x; i<x+width; i++) {
    for (int j=y; j<y+height; j++) {
      //az = modulus(i, getWidth());
      az = clip(i, 0, getWidth());
      //el = modulus(j, getHeight());
      //az = clip(i, 2, getWidth()-3);
      el = clip(j, 0, getHeight());
      if (el - j < 0) {
        el = 0;j - el;
        az = 0;modulus(az+getWidth()/2, getWidth());
      } else if (el - j > 0) {
        el = 0;getHeight() - j - el - 1;
        az = 0;modulus(az+getWidth()/2, getWidth());
      }
      setValue(az, el, getValue(az, el) + val);
    }
  }
}

void Histogram::getValues(float* return_vals, int x, int y, int width, int height) {
  float el, az;
  for (int i=x; i<x+width; i++) {
    for (int j=y; j<y+height; j++) {
      az = modulus(i, getWidth());
      //el = modulus(j, getHeight());
      //az = clip(i, 2, getWidth()-3);
      el = clip(j, 0, getHeight());
      if (el - j < 0) {
        el = j - el;
        az = modulus(az+getWidth()/2, getWidth());
      } else if (el - j > 0) {
        el = getHeight() - j - el - 1;
        az = modulus(az+getWidth()/2, getWidth());
      }
      return_vals[(j-y)*width+(i-x)] = getValue(az, el);
    }
  }
}

void Histogram::addVoxel(float x, float y, float z, float val) {
  int az = getI(x, y);
  int el = getJ(x, y, z);
  //assert(0 < az && az<getWidth() && 0 < el && el<getHeight());
  setValue(az, el, getValue(az, el) + val);
}

void Histogram::addVoxel(float x, float y, float z, float val,
                         float voxel_radius, float maxRange) {
  std::cout << "V" << std::endl;
  // For weight calculation
  float dist = sqrt(pow(x-ox, 2) +
                    pow(y-oy, 2) +
                    pow(z-oz, 2));
  float enlargement = floor(asin(voxel_radius/dist)/alpha);
  float a = 0.5;
  float b = 4*(a-1)/pow(maxRange-1, 2);
  float h = val*val*(a-b*(dist-voxel_radius));

  int bz = getI(x, y);
  int be = getJ(x, y, z);
  int voxelCellSize = 1; //(int)(enlargement/alpha); // divided by 2
  int az,el;
  addValues(h, bz-voxelCellSize, be-voxelCellSize, 2*voxelCellSize, 2*voxelCellSize);
  /*
  float gv[4*voxelCellSize*voxelCellSize];
  getValues(gv, bz-voxelCellSize, be-voxelCellSize, 2*voxelCellSize, 2*voxelCellSize);
  add(gv, h, gv, 4*voxelCellSize*voxelCellSize);
  setValues(gv, bz-voxelCellSize, be-voxelCellSize, 2*voxelCellSize, 2*voxelCellSize);
  */
}

void Histogram::checkTurning(float x, float y, float z, float val,
                             Vehicle v, float voxel_radius) {
  // Iterate over half possible ways the rover can move (then check left and right
  int j = getJ(x,y,z);
  int i = getI(x,y);
  float turningLeftCenterX = v.turningRadiusR()*sin(v.heading);
  float turningRightCenterX = -v.turningRadiusR()*sin(v.heading);
  float turningLeftCenterY = v.turningRadiusL()*cos(v.heading);
  float turningRightCenterY = -v.turningRadiusL()*cos(v.heading);
  float dr = sqrt(pow((turningRightCenterX - (x-ox)), 2) +
                  pow((turningRightCenterY - (x-ox)), 2));
  float dl = sqrt(pow((turningLeftCenterX - (x-ox)), 2) +
                  pow((turningLeftCenterY - (x-ox)), 2));
  std::cout << dr << ", " << dl << std::endl;
  float rad = v.safety_radius+v.radius()+voxel_radius;
  if (dr < v.turningRadiusR()+rad || dl < v.turningRadiusL()+rad)
    addValue(i, j, val);
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
  int i = modulus(floor(atan2(x-ox, y-oy)/alpha + getWidth()/2), getWidth());
  return i;
}

int Histogram::getJ(float x, float y, float z) {
  float p = sqrt(pow((x-ox), 2) + pow((y-oy), 2));
  //float j = modulus(-floor(atan2(z-oz, p)/alpha - getHeight()/2), getHeight());
  //int j = floor(getHeight()/2 - atan2(z-oz, p)/alpha);
  float at = atan2(z-oz, p)/alpha;
  //return clip(floor(((float)getHeight())/2 - at+0.5), 0, getHeight());
  if (at < 0) // modifications for the bottom half
    return clip(floor(((float)getHeight())/2 - at -alpha*2), 0, getHeight()-1);
  else // modifications for the top half
    return clip(at, 0, getHeight());
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
