#include <stdio.h>
#include "kalman.h"
#include <math.h>


// Initialize the Kalman filter
void kalman_init(KalmanFilter *kf) {
    kf->state[0] = 0.0;  // Initial latitude
    kf->state[1] = 0.0;  // Initial longitude

    // Initial covariance matrix (high uncertainty)
    kf->covariance[0][0] = 1000.0;
    kf->covariance[0][1] = 0.0;
    kf->covariance[1][0] = 0.0;
    kf->covariance[1][1] = 1000.0;

    // Process noise covariance matrix
    kf->process_noise[0][0] = 1.0;
    kf->process_noise[0][1] = 0.0;
    kf->process_noise[1][0] = 0.0;
    kf->process_noise[1][1] = 1.0;

    // Measurement noise covariance matrix
    kf->measurement_noise[0][0] = 0.5;
    kf->measurement_noise[0][1] = 0.0;
    kf->measurement_noise[1][0] = 0.0;
    kf->measurement_noise[1][1] = 0.5;

    // Identity matrix
    matrix_identity(kf->identity);
}

// Predict step of the Kalman filter
void kalman_predict(KalmanFilter *kf) {
    float predicted_covariance[DIM][DIM];

    // Update the covariance: P' = P + Q
    matrix_add(kf->covariance, kf->process_noise, predicted_covariance);
    matrix_copy(predicted_covariance, kf->covariance);
}

// Update step of the Kalman filter
void kalman_update(KalmanFilter *kf, float measurement[DIM]) {
    float kalman_gain[DIM][DIM];
    float measurement_residual[DIM];
    float residual_covariance[DIM][DIM];
    float temp1[DIM][DIM], temp2[DIM][DIM];
    float updated_state[DIM];

    // Measurement residual: y = z - Hx (simplified to y = z - x since H = I)
    measurement_residual[0] = measurement[0] - kf->state[0];
    measurement_residual[1] = measurement[1] - kf->state[1];

    // Residual covariance: S = P + R
    matrix_add(kf->covariance, kf->measurement_noise, residual_covariance);

    // Calculate Kalman Gain: K = P * S^(-1)
    matrix_inverse(residual_covariance, temp1);
    matrix_mult(kf->covariance, temp1, kalman_gain);

    // Update state: x' = x + Ky
    matrix_mult_vector(kalman_gain, measurement_residual, updated_state);
    kf->state[0] += updated_state[0];
    kf->state[1] += updated_state[1];

    // Update covariance: P' = (I - K)P
    matrix_mult(kalman_gain, kf->covariance, temp1);
    matrix_sub(kf->identity, kalman_gain, temp2);
    matrix_mult(temp2, temp1, kf->covariance);
}

// Matrix addition
void matrix_add(float a[DIM][DIM], float b[DIM][DIM], float result[DIM][DIM]) {
    for (int i = 0; i < DIM; i++)
        for (int j = 0; j < DIM; j++)
            result[i][j] = a[i][j] + b[i][j];
}

// Matrix subtraction
void matrix_sub(float a[DIM][DIM], float b[DIM][DIM], float result[DIM][DIM]) {
    for (int i = 0; i < DIM; i++)
        for (int j = 0; j < DIM; j++)
            result[i][j] = a[i][j] - b[i][j];
}

// Matrix multiplication
void matrix_mult(float a[DIM][DIM], float b[DIM][DIM], float result[DIM][DIM]) {
    for (int i = 0; i < DIM; i++) {
        for (int j = 0; j < DIM; j++) {
            result[i][j] = 0.0;
            for (int k = 0; k < DIM; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

// Matrix multiplication with a vector
void matrix_mult_vector(float matrix[DIM][DIM], float vector[DIM], float result[DIM]) {
    for (int i = 0; i < DIM; i++) {
        result[i] = 0.0;
        for (int j = 0; j < DIM; j++) {
            result[i] += matrix[i][j] * vector[j];
        }
    }
}

// Create an identity matrix
void matrix_identity(float matrix[DIM][DIM]) {
    for (int i = 0; i < DIM; i++) {
        for (int j = 0; j < DIM; j++) {
            matrix[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

// Copy one matrix to another
void matrix_copy(float src[DIM][DIM], float dest[DIM][DIM]) {
    for (int i = 0; i < DIM; i++)
        for (int j = 0; j < DIM; j++)
            dest[i][j] = src[i][j];
}

// Inverse of a 2x2 matrix
void matrix_inverse(float a[DIM][DIM], float result[DIM][DIM]) {
    float det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if (det == 0) {
        printf("Matrix is not invertible!\n");
        return;
    }
    float inv_det = 1.0 / det;
    result[0][0] = a[1][1] * inv_det;
    result[0][1] = -a[0][1] * inv_det;
    result[1][0] = -a[1][0] * inv_det;
    result[1][1] = a[0][0] * inv_det;
}

// Function to calculate the distance between two lat/lon points using the Haversine formula
float haversine_distance(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371e3; // Earth radius in meters
    float dlat = (lat2 - lat1) * (M_PI / 180.0);
    float dlon = (lon2 - lon1) * (M_PI / 180.0);
    float a = sin(dlat / 2) * sin(dlat / 2) +
              cos(lat1 * (M_PI / 180.0)) * cos(lat2 * (M_PI / 180.0)) *
              sin(dlon / 2) * sin(dlon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c; // Distance in meters
}

// Function to calculate speed in m/s given distance and time delta in seconds
float calculate_speed(float distance, float delta_time) {
    if (delta_time == 0) return 0.0;
    return distance / delta_time;
}
