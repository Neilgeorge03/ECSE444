#ifndef FILTER_H
#define FILTER_H

typedef struct {
    float q; //process noise covariance
    float r; //measurement noise covariance
    float x; //estimated value
    float p; //estimation error covariance
    float k; // adaptive Kalman filter gain
} kalman_state ;


#define KALMAN_SUCCESS 0
#define KALMAN_DIV_BY_ZERO 1
#define KALMAN_OVERFLOW 2

extern int KalmanFilter(kalman_state *filter_addr, float measurement);
extern int KalmanFilterArray(kalman_state *filter_addr, float *measurement, float *result_addr);

#endif
