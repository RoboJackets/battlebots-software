#include <Eigen.h>
#include <Eigen/Dense>

class Kalman {
    public:
        Kalman(float period = 1/100);
        Eigen::Vector2f update(Eigen::Vector4f measured, float control);
    private:
        Eigen::Vector2f xhat;               // Predicted State (angle, angular velocity)
        Eigen::Matrix2f P;                  // Covariance of Predicted State
        Eigen::Matrix<float, 4, 2> C;       // Jacobian (Maps to measured (4 accelerometers))
        Eigen::Matrix<float, 2, 4> K;       // Kalman Gain
        Eigen::Matrix4f R;                  // Covariance of Sensor Noise
        Eigen::Matrix2f Q;                  // Noise Matrix
        Eigen::Matrix2f A;                  // State Update Matrix
        Eigen::Vector2f B;                  // Control Matrix
        float RADIUS;                       // Radius
        float T;                            // Period
        float t;                            // Time
};