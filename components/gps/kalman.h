// kalman.h
#ifndef KALMAN_H
#define KALMAN_H

#include <stdio.h>

#define DIM 2  // Dimension for 2D filter (latitude, longitude)

// Function prototypes for matrix operations
void matrix_add(float a[DIM][DIM], float b[DIM][DIM], float result[DIM][DIM]);
void matrix_sub(float a[DIM][DIM], float b[DIM][DIM], float result[DIM][DIM]);
void matrix_mult(float a[DIM][DIM], float b[DIM][DIM], float result[DIM][DIM]);
void matrix_mult_vector(float matrix[DIM][DIM], float vector[DIM], float result[DIM]);
void matrix_identity(float matrix[DIM][DIM]);
void matrix_copy(float src[DIM][DIM], float dest[DIM][DIM]);
void matrix_inverse(float a[DIM][DIM], float result[DIM][DIM]);

// Structure for Kalman Filter
typedef struct {
    float state[DIM];           // State vector [latitude, longitude]
    float covariance[DIM][DIM]; // State covariance matrix
    float process_noise[DIM][DIM];  // Process noise covariance matrix
    float measurement_noise[DIM][DIM]; // Measurement noise covariance matrix
    float identity[DIM][DIM];   // Identity matrix
} KalmanFilter;

// Function prototypes for Kalman Filter
void kalman_predict(KalmanFilter* kf);
void kalman_update(KalmanFilter* kf, float measurement[DIM]);
void kalman_init(KalmanFilter* kf);

float haversine_distance(float lat1, float lon1, float lat2, float lon2);
float calculate_speed(float distance, float delta_time);

#endif // KALMAN_H
