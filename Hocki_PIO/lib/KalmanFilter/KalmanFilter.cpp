#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(float period)
{
     T = period;
     xhat << 0, 0;
     P << 0, 0,
         0, 0;
     A << 1, T,
         0, 1;
     B << 0.5 * T * T, T;
     Q = 0 * Eigen::Matrix2f::Identity();
     R = 0.000001f * Eigen::Matrix4f::Identity();
     C << 0, 0,
         0, 0,
         0, 0,
         0, 0;
}

Eigen::Vector2f KalmanFilter::update(Eigen::Vector4f measured, float control)
{

     // Predict
     xhat = A * xhat + B * control;
     P = A * P * A.transpose() + Q;

     // Update
     K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
     // K = Eigen::Matrix<float, 2, 4>::Zero();
     xhat = xhat + K * (measured - C * xhat);
     P = (Eigen::Matrix2f::Identity() - K * C) * P;

     // Update Jacobian
     C << 0, 2 * RADIUS * xhat(1) * xhat(1),
         0, 2 * RADIUS * xhat(1) * xhat(1),
         0, 2 * RADIUS * xhat(1) * xhat(1),
         0, 2 * RADIUS * xhat(1) * xhat(1);

     return xhat;
}