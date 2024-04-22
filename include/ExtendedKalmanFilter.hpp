#pragma once
#include <StandardCplusplus.h>
#include <Eigen30.h>

#include <Eigen/Dense>

using namespace Eigen;

/*
EKF for Micromouse positional estimates. State vector of the form:

[x; y; θ; b; d]



*/
class ExtendedKalmanFilter
{
private:
    MatrixXd state;            // State estimate
    MatrixXd covariance;       // Covariance matrix
    MatrixXd processNoise;     // Process noise covariance matrix
    MatrixXd measurementNoise; // Measurement noise covariance matrix
    MatrixXd stateTransition;  // State transition matrix
    MatrixXd measurementFunc;  // Measurement function matrix

public:
    ExtendedKalmanFilter(MatrixXd initialState, MatrixXd initialCovariance,
                         MatrixXd processNoise, MatrixXd measurementNoise,
                         MatrixXd stateTransition, MatrixXd measurementFunc)
        : state(initialState), covariance(initialCovariance),
          processNoise(processNoise), measurementNoise(measurementNoise),
          stateTransition(stateTransition), measurementFunc(measurementFunc) {}

    // MatrixXd computeStateTransition(double dt, double rl, double rr, double L, double theta, double vl, double vr)
    // {
    //     MatrixXd F = MatrixXd(3, 3);

    //     // Compute elements of F
    //     double cos_theta = cos(theta);
    //     double sin_theta = sin(theta);
    //     double vel_sum = rl * (vl + vr) / 2.0;
    //     double vel_diff = rl * (vr - vl) / L;

    //     // Populate F matrix

    //     F << 1, 0, -dt * vel_sum * sin_theta,
    //         0, 1, dt * vel_sum * cos_theta,
    //         0, 0, 1 + dt * vel_diff;

    //     return F;
    // }
    // Prediction step
    void predict()
    {
        state = stateTransition * state;
        covariance = stateTransition * covariance * stateTransition.transpose() + processNoise;
    }

    // Update step
    void update(const MatrixXd &measurement)
    {
        MatrixXd innovation = measurement - measurementFunc * state;
        MatrixXd innovationCovariance = measurementFunc * covariance * measurementFunc.transpose() + measurementNoise;
        MatrixXd kalmanGain = covariance * measurementFunc.transpose() * innovationCovariance.inverse();

        state = state + kalmanGain * innovation;
        covariance = (MatrixXd::Identity(state.rows(), state.rows()) - kalmanGain * measurementFunc) * covariance;
    }

    // Getter for current state estimate
    MatrixXd getState() const
    {
        return state;
    }
};