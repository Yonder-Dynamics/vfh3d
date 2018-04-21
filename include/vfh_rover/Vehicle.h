#pragma once
#include <octomap/math/Vector3.h>
#include <algorithm>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>

using namespace Eigen;

class Vehicle {
  public:
    float x, y, z, h, w, d, safety_radius; // meters
    float maxIncline, minIncline; // radians
    bool prevSet;
    Quaternionf orientation;
    geometry_msgs::Pose* prevHeading;

    Vehicle(float x, float y, float z, float h, float w, float d, float safety_radius, float maxIncline, float minIncline, bool prevSet):x(x), y(y), z(z), h(h), w(w), d(d), safety_radius(safety_radius), maxIncline(maxIncline), minIncline(minIncline), prevSet(prevSet) {};

    void setPose(geometry_msgs::Pose pose) {
      x = pose.position.x;
      y = pose.position.y;
      z = pose.position.z;
      orientation = Quaternionf(pose.orientation.w, pose.orientation.x,
          pose.orientation.y, pose.orientation.z);
    }

    float getHeading() {
      orientation.normalize();
      Eigen::Matrix3f mat = orientation.toRotationMatrix();
      Eigen::Vector3f vec = mat.eulerAngles(0,1,2);
      return vec(2);
    }

    float radius() {
      float max_dim = std::max(std::max(h,w),d);
      return max_dim/2;
    }

    float turningRadiusR() {
      return 0.2;
    }

    float turningRadiusL() {
      return 0.2;
    }
    octomath::Vector3 center() {
      return octomath::Vector3(x, y, z);
    }

    octomath::Vector3 min() {
      return octomath::Vector3(x-w/2, y-h/2, z-d/2);
    }

    octomath::Vector3 max() {
      return octomath::Vector3(x+w/2, y+h/2, z+d/2);
    }
  private:
};
