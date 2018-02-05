class Histogram {
 public:
  /** Constructor with width and height parameters **/
  Histogram(float alpha): alpha(alpha){
    data = new float[getWidth() * getHeight()];
  }
  float getValue(int x, int y);
  void addValue(int x, int y, float val);
  int calcAzimuth(float alpha, float xi, float xo, float yi, float yo);
  int calcElevation(float alpha, float zi, float zo, float xi, float xo, float yi, float yo);
  bool isIgnored(float xi, float yi, float zi, float xo, float yo, float zo, float ws);
  void setValue(int x, int y, float val);
  int getWidth();
  int getHeight();
  float getAlpha();
 private:
  float* data;
  float alpha;

};
