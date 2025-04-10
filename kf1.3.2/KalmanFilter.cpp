// KalmanFilter.cpp
#include "KalmanFilter.h"

// Constructor: initializes the filter parameters
KalmanFilter::KalmanFilter(float processNoise, float measurementNoise, float estimationError, float initialEstimate) {
    processNoiseQ = processNoise;
    measurementNoiseR = measurementNoise;
    estimationErrorP = estimationError;
    estimate = initialEstimate;
}

// Update function: processes the measurement and rate to estimate the state
float KalmanFilter::update(float measurement, float rate, float dt) {
    // Predict phase
    estimate += rate * dt; // Predict the next state based on rate and time delta
    estimationErrorP += processNoiseQ; // Increase estimation error by process noise

    // Update phase
    kalmanGainK = estimationErrorP / (estimationErrorP + measurementNoiseR); // Compute Kalman Gain
    estimate += kalmanGainK * (measurement - estimate); // Correct the estimate with the measurement
    estimationErrorP *= (1 - kalmanGainK); // Update estimation error

    return estimate; // Return the updated estimate
}
