// kalman.h
#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    double q;  // Process noise covariance
    double r;  // Measurement noise covariance
    double x;  // Value (filtered value)
    double p;  // Estimation error covariance
    double k;  // Kalman gain
} KalmanFilter;

// Function prototypes
void kalman_init(KalmanFilter* kf, double process_noise, double measurement_noise, double initial_estimate);
double kalman_update(KalmanFilter* kf, double measurement);

#endif // KALMAN_H
