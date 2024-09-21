#ifndef MEASUREMENTMODEL_HPP
#define MEASUREMENTMODEL_HPP

#include <Eigen/Dense>

class MeasurementModel
{
  public:
    // Конструктор
    MeasurementModel();

    // Метод для получения предсказанных измерений на основе текущего состояния
    Eigen::VectorXd predictMeasurement(const Eigen::VectorXd &state) const;

    // Метод для получения матрицы Якобиана функции измерения
    Eigen::MatrixXd getJacobian(const Eigen::VectorXd &state) const;

    // Установка и получение матрицы наблюдения
    void setObservationMatrix(const Eigen::MatrixXd &H);
    Eigen::MatrixXd getObservationMatrix() const;

    // Установка и получение матрицы шума измерений
    void setMeasurementNoiseMatrix(const Eigen::MatrixXd &R);
    Eigen::MatrixXd getMeasurementNoiseMatrix() const;

  private:
    Eigen::MatrixXd H_; // Матрица наблюдения
    Eigen::MatrixXd R_; // Матрица шума измерений
};

#endif // MEASUREMENTMODEL_HPP
