#ifndef FILTER_H
#define FILTER_H

typedef struct {
    float q; //process noise covariance
    float r; //measurement noise covariance
    float x; //estimated value
    float p; //estimation error covariance
    float k; // adaptive Kalman filter gain
} KalmanStruct ;

extern int KalmanFilter(KalmanStruct *filter_addr, float measurement);

#endif
