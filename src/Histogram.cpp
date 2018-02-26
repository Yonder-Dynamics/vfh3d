#include <math.h>
#include <vfh_rover/Histogram.h>
#include <vfh_rover/Vehicle.h>
#include <string>
#include <assert.h>
#include <iostream>
#include <cmath>
#include <pcl/point_types.h>
#include <geometry_msgs/Pose.h>

Histogram::Histogram(float alpha,
    float ox, float oy, float oz):
  alpha(alpha), ox(ox), oy(oy), oz(oz)
{
  data = new float[getWidth() * getHeight()]();
}

int Histogram::getI(float x, float y) {
  int i = modulus(floor(atan2(x-ox, y-oy)/alpha), getWidth());
  return i;
}

int Histogram::getJ(float x, float y, float z) {
  float p = sqrt(pow((x-ox), 2) + pow((y-oy), 2));
  //float j = modulus(-floor(atan2(z-oz, p)/alpha - getHeight()/2), getHeight());
  //int j = floor(getHeight()/2 - atan2(z-oz, p)/alpha);
  float at = (atan2(z-oz, p)+M_PI/2)/alpha;
  //return clip(floor(((float)getHeight())/2 - at+0.5), 0, getHeight());
  /*
    if (at < 0) // modifications for the bottom half
    return clip(floor(((float)getHeight())/2 - at -alpha*2), 0, getHeight()-1);
    else // modifications for the top half
    return clip(at, 0, getHeight());
  */
  return at;
}

// j = 0 = top
float Histogram::getValue(int i, int j) {
  int az = modulus(i, getWidth());
  int el = clip(j, 0, getHeight()-1);
  if (j - el > 0) {
    std::cout << "HI1 " << i << ", " << j << std::endl;
    // if el overflows the top and goes over to the other side
    // j is negative
    // el is 0
    el = getHeight() - (j-el);
    az = modulus(az, getWidth());
  } else if (j - el > 0) {
    // if el overflows the bot and goes over to the other side
    std::cout << "HI2" << std::endl;
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
    std::cout << "HI1 " << i << ", " << j << std::endl;
    // if el overflows the top and goes over to the other side
    // j is negative
    // el is 0
    el = getHeight() - (j-el);
    az = modulus(az, getWidth());
  } else if (j - el > 0) {
    // if el overflows the bot and goes over to the other side
    std::cout << "HI2" << std::endl;
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

void Histogram::addVoxel(float x, float y, float z, float val) {
  int az = getI(x, y);
  int el = getJ(x, y, z);
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
  float rad = v.safety_radius+v.radius()+voxel_radius;
  if (dr < v.turningRadiusR()+rad || dl < v.turningRadiusL()+rad)
    addValue(i, j, val);
}

std::vector<geometry_msgs::Pose> Histogram::findPaths(int width, int height) {
  std::cout << "A" << std::endl;
  float ret_vals[width*height];
  std::vector<geometry_msgs::Pose> ps;
  float az, el;
  for (int i=0; i<getWidth(); i++) {
    for (int j=0; j<getHeight(); j++) {
      getValues(ret_vals, i, j, width, height);
      /*
      for (int i2=0; i2<width; i2++) {
        for (int j2=0; j2<height; j2++) {
          std::cout << ret_vals[j2*width+i2] << ", ";
        }
        std::cout << std::endl;
      }
      */
      float s = sum(ret_vals, width*height);
      //std::cout << s << std::endl;
      if (abs(s) < 0.0001) {
        az = -(i+(float)width/2-getWidth()/4)*alpha;
        el = M_PI/2-(j+(float)height/2-1)*alpha;
        //std::cout << el << ", " << az << std::endl;
        geometry_msgs::Pose p;
        // Abbreviations for the various angular functions
        double cy = cos(az * 0.5);
        double sy = sin(az * 0.5);
        double cr = cos(0 * 0.5); // roll = 0
        double sr = sin(0 * 0.5); // roll = 0
        double cp = cos(el * 0.5);
        double sp = sin(el * 0.5);

        p.orientation.w = cy * cr * cp + sy * sr * sp;
        p.orientation.x = cy * sr * cp - sy * cr * sp;
        p.orientation.y = cy * cr * sp + sy * sr * cp;
        p.orientation.z = sy * cr * cp - cy * sr * sp;

        p.position.x = ox;
        p.position.y = oy;
        p.position.z = oz;
        ps.push_back(p);
      }
    }
  }
  return ps;
}

float Histogram::mean() {
  return sum(data, getWidth()*getHeight()) / getWidth() / getHeight();
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
      float az = alpha*modulus(i-getWidth()/2, getWidth()) + alpha/2;
      float el = alpha*modulus(j-getHeight()/2, getHeight()) + alpha/2;
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
