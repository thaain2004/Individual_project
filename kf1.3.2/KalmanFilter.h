// KalmanFilter.h
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
public:
    KalmanFilter(float processNoise, float measurementNoise, float estimationError, float initialEstimate);

    float update(float measurement, float rate, float dt);

private:
    float processNoiseQ;
    float measurementNoiseR;
    float estimationErrorP;
    float kalmanGainK;
    float estimate;
};

#endif // KALMANFILTER_H