#include <vfh-rover/Histogram.h>
#include <math.h>

float Histogram::getValue(int x, int y) {
  return this->data[(y*getWidth())+x]; 
}

void Histogram::setValue(int x, int y, float val) {
  this->data[(y*getWidth())+x] = val;
}

void Histogram::addValue(int azimuth, int elevation, float val) { 
  this->data[(y*getWidth())+x] += val;
}

int Histogram::calcAzimuth(float alpha, float xi, float xo, float yi, float, yo) {
  return (int)((1/alpha)*(atan((xi-xo)/yi-yo)));
}

int Histogram::calcElevation(float alpha, float zi, float zo, float xi, float xo, float yi, float yo) {
  float p = sqrt(pow((xi-xo), 2.0) + pow((yi-yo), 2.0));
  return (int)((1/alpha)*atan((zi-zo)/p));
}

bool Histogram::isIgnored(float xi, float yi, float zi, float xo, float yo, float zo, float ws) {
  float dist = sqrt(pow((xi-xo), 2.0) + pow((yi-yo), 2.0) + pow((zi-zo), 2.0));
  return dist > (ws/2.0);
}

int Histogram::getWidth() {
  return ((2*M_PI)/this->alpha);
}

int Histogram::getHeight() {
  return ((M_PI)/this->alpha);
}
