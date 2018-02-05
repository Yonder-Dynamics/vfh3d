#include <math.h>
#include <vfh_rover/Histogram.h>

float Histogram::getValue(int az, int el) {
  return this->data[(el*getWidth())+x];
}

void Histogram::setValue(int az, int el, float val) {
  this->data[(y*getWidth())+x] = val;
}

void Histogram::addValue(int x, int y, int z, float val) {
  int az = calcAzimuth(x, y);
  int el = calcElevation(x, y, z);
  setValue(az, el, getValue(az, el) + val);
}

int Histogram::calcAzimuth(float x, float y) {
  return (int)((1/alpha)*(atan((x-ox)/y-oy)));
}

int Histogram::calcElevation(float x, float y, float z) {
  float p = sqrt(pow((x-ox), 2.0) + pow((y-oy), 2.0));
  return (int)((1/alpha)*atan((z-oz)/p));
}

bool Histogram::isIgnored(float x, float y, float z, float ws) {
  float dist = sqrt(pow((x-ox), 2.0) + pow((y-oy), 2.0) + pow((z-oz), 2.0));
  return dist > (ws/2.0);
}

int Histogram::getWidth() {
  return ((2*M_PI)/this->alpha);
}

int Histogram::getHeight() {
  return ((M_PI)/this->alpha);
}
