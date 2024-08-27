// kalman.c
#include "kalman.h"

void kalman_init(KalmanFilter* kf, double process_noise, double measurement_noise, double initial_estimate) {
    kf->q = process_noise;
    kf->r = measurement_noise;
    kf->x = initial_estimate;
    kf->p = 1.0;
    kf->k = 0.0;
}

double kalman_update(KalmanFilter* kf, double measurement) {
    // Prediction update
    kf->p = kf->p + kf->q;

    // Measurement update
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1 - kf->k) * kf->p;

    return kf->x;
}
