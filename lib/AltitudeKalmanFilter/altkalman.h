#ifndef kl
#define kl

class KalLib  {
  public:
    double getAltEst();
    void setAlt(double altitudeData);
    float altEst = 0.0;
    float altVariance = 1.12184278324081E-05;  


  private:
  // Change the value of altVariance to make the data smoother or respond faster
  float varProcess = 1e-8;
  float PC = 0.0;
  float K = 0.0;
  float UP = 1.0;
  float altitudedata;
};
#endif