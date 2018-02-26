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
    //std::cout << "HI1 " << i << ", " << j << std::endl;
    // if el overflows the top and goes over to the other side
    // j is negative
    // el is 0
    el = getHeight() - (j-el);
    az = modulus(az, getWidth());
  } else if (j - el > 0) {
    // if el overflows the bot and goes over to the other side
    //std::cout << "HI2" << std::endl;
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
    //std::cout << "HI1 " << i << ", " << j << std::endl;
    // if el overflows the top and goes over to the other side
    // j is negative
    // el is 0
    el = getHeight() - (j-el);
    az = modulus(az, getWidth());
  } else if (j - el > 0) {
    // if el overflows the bot and goes over to the other side
    //std::cout << "HI2" << std::endl;
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

geometry_msgs::Pose Histogram::optimalPath(geometry_msgs::Pose* prevPath, Vehicle v, float goalHeading,
                                           float goalWeight, float prevWeight, float headingWeight) {
  std::vector<geometry_msgs::Pose> open = findPaths(int(v.safety_radius+v.w), int(v.safety_radius+v.h));
  float vals[open.size()];
  for (int i = 0; i < open.size(); i++) {
    geometry_msgs::Pose * p = &open.at(i);
    // Calculate difference between heading of path and the goal and vehicle
    float vecPath[3] = {1,0,0};
    auto ori = p->orientation; // orientation struct
    float real[3] = {ori.x, ori.y, ori.z};
    float v1[3];
    mul(real, 2*dot(real, vecPath, 3), v1,  3);
    float v2[3];
    mul(vecPath, ori.w*ori.w - dot(real,real,3), v2, 3);
    float v3[3];
    cross(real, vecPath, v3);
    float v4[3];
    mul(v3, 2*ori.w, v4, 3);
    float v5[3];
    float res[3];
    add(v1, v2, v5, 3);
    add(v3, v5, res, 3);
    float path_heading = atan2(res[1], res[0]);
    float goal_diff = abs(goalHeading - path_heading);
    float veh_diff = abs(v.heading - path_heading);
    float prev_diff;
    if (prevPath != NULL) {
      float prev_diff_v[3];
      float prevQuat[4] = {prevPath->orientation.x, prevPath->orientation.y,
                          prevPath->orientation.z, prevPath->orientation.w};
      float pathQuat[4] = {ori.x, ori.y, ori.z, ori.w};
      mul(prevQuat, -1, prev_diff_v, 4);
      add(prev_diff_v, pathQuat, prev_diff_v, 4);
      prev_diff = sum(prev_diff_v, 4);
    } else {
      prev_diff = 0;
    }
    float value = goalWeight*goal_diff + prevWeight*prev_diff + headingWeight*veh_diff;
    vals[i] = value;
  }
  int bestPath = maxInd(vals, open.size());
  return open[bestPath];
}

float Histogram::mean() {
  return sum(data, getWidth()*getHeight()) / getWidth() / getHeight();
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
      color = color > 255 ? 255 : color;
      p.r = p.g = p.b = color;
      pc->points.push_back(p);
    }
  }
  return pc;
}
