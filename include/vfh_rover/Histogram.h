class Histogram {
 public:
  /** Constructor with width and height parameters **/
  Histogram(float alpha, float ox, float oy, float oz): alpha(alpha), ox(ox), oy(oy), oz(oz) {
    data = new float[getWidth() * getHeight()];
  }

  int calcAzimuth(float x, float y);
  int calcElevation(float x, float y, float z);
  bool isIgnored(float x, float y, float z, float ws);

  int getWidth();
  int getHeight();
  float getAlpha();

  // For processing histogram
  float getValue(int az, int el);
  void setValue(int az, int el, float val);
  // For filling histogram
  void addValue(int x, int y, int z, float val);

 private:
  float* data;
  float alpha, ox, oy, oz;

};
