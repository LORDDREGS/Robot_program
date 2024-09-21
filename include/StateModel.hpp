#ifndef STATEMODEL_HPP
#define STATEMODEL_HPP

#include <Eigen/Dense>

class StateModel
{
  public:
    // Конструктор
    StateModel();

    // Метод для прогнозирования следующего состояния
    Eigen::VectorXd predictState(const Eigen::VectorXd &current_state, double delta_t) const;

    // Метод для получения матрицы Якобиана функции перехода состояния
    Eigen::MatrixXd getJacobian(const Eigen::VectorXd &state, double delta_t) const;

    // Установка и получение матрицы перехода состояния
    void setTransitionMatrix(const Eigen::MatrixXd &F);
    Eigen::MatrixXd getTransitionMatrix() const;

    // Установка и получение матрицы шума процесса
    void setProcessNoiseMatrix(const Eigen::MatrixXd &Q);
    Eigen::MatrixXd getProcessNoiseMatrix() const;

  private:
    Eigen::MatrixXd F_; // Матрица перехода состояния
    Eigen::MatrixXd Q_; // Матрица шума процесса
};

#endif // STATEMODEL_HPP
