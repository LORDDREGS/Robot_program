#include "StateModel.hpp"

StateModel::StateModel()
{
}

Eigen::VectorXd StateModel::predictState(const Eigen::VectorXd &current_state, double delta_t) const
{
    // Предсказание следующего состояния: x' = F * x
    return F_ * current_state;
}

Eigen::MatrixXd StateModel::getJacobian(const Eigen::VectorXd &state, double delta_t) const
{
    // Возвращаем матрицу перехода состояния (Jacobian)
    return F_;
}

void StateModel::setTransitionMatrix(const Eigen::MatrixXd &F)
{
    F_ = F;
}

Eigen::MatrixXd StateModel::getTransitionMatrix() const
{
    return F_;
}

void StateModel::setProcessNoiseMatrix(const Eigen::MatrixXd &Q)
{
    Q_ = Q;
}

Eigen::MatrixXd StateModel::getProcessNoiseMatrix() const
{
    return Q_;
}
