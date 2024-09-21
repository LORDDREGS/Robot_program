#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

#include <Eigen/Dense>

class KalmanFilter
{
  public:
    KalmanFilter();
    void initialize(const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &P);
    void predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q);
    void update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);
    Eigen::VectorXd getState() const;

  private:
    Eigen::VectorXd x_; // состояние
    Eigen::MatrixXd P_; // ковариация состояния
};

#endif // KALMANFILTER_HPP
