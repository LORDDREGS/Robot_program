#include "MeasurementModel.hpp"

MeasurementModel::MeasurementModel()
{
}

Eigen::VectorXd MeasurementModel::predictMeasurement(const Eigen::VectorXd &state) const
{
    // Предсказание измерения: z = H * x
    return H_ * state;
}

Eigen::MatrixXd MeasurementModel::getJacobian(const Eigen::VectorXd &state) const
{
    // Возвращаем матрицу наблюдения (Jacobian)
    return H_;
}

void MeasurementModel::setObservationMatrix(const Eigen::MatrixXd &H)
{
    H_ = H;
}

Eigen::MatrixXd MeasurementModel::getObservationMatrix() const
{
    return H_;
}

void MeasurementModel::setMeasurementNoiseMatrix(const Eigen::MatrixXd &R)
{
    R_ = R;
}

Eigen::MatrixXd MeasurementModel::getMeasurementNoiseMatrix() const
{
    return R_;
}
