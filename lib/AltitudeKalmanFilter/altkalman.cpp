#include "altkalman.h"

void KalLib::setAlt(double altitudeData) {
  altitudeData = altitudedata;
}

// Pretend this is one or more complex and involved functions you have written
double KalLib::getAltEst() {

    // Predict the next covariance
    PC =  UP + varProcess;

    // Compute the kalman gain
    K = PC / (PC + altVariance);

    // Update the covariance 
    UP = (1 - K) * PC;

    // Re-define variables
    float Xp =  altEst;
    float Zp = Xp;

    // Final altitude estimation
    altEst = K * (altitudedata - Zp) + Xp;  
  }


  