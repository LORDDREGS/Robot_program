#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter() {}

void KalmanFilter::initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& P) {
    x_ = initial_state;
    P_ = P;
}

void KalmanFilter::predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q) {
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {
    Eigen::VectorXd y = z - H * x_;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() /* / S */;
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H) * P_;
}

Eigen::VectorXd KalmanFilter::getState() const {
    return x_;
}
