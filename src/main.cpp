#include "KalmanFilter.hpp"
#include "Dijkstra.hpp"
#include <iostream>
#include <matplot/matplot.h>

int main() {
    // Пример использования расширенного фильтра Калмана
    KalmanFilter kf;
    Eigen::VectorXd initial_state(4);
    initial_state << 0, 0, 0, 0;
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(4, 4);
    kf.initialize(initial_state, P);

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4) * 0.1;
    kf.predict(F, Q);

    Eigen::VectorXd z(2);
    z << 1, 1;
    Eigen::MatrixXd H(2, 4);
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2) * 0.1;

    kf.update(z, H, R);

    std::cout << "State after update: \n" << kf.getState() << std::endl;

    // Визуализация результатов с использованием Matplot++
    std::vector<double> time = {0, 1, 2, 3, 4, 5};
    std::vector<double> values = {0, 1, 2, 3, 4, 5};

    matplot::plot(time, values);
    matplot::xlabel("Time");
    matplot::ylabel("Value");
    matplot::title("Example Plot");
    matplot::show();

    // Пример использования алгоритма Дейкстры
    Dijkstra dijkstra(5);
    dijkstra.addEdge(0, 1, 10);
    dijkstra.addEdge(0, 2, 5);
    dijkstra.addEdge(1, 2, 2);
    dijkstra.addEdge(1, 3, 1);
    dijkstra.addEdge(2, 3, 9);
    dijkstra.addEdge(2, 4, 2);
    dijkstra.addEdge(3, 4, 4);

    auto path = dijkstra.shortestPath(0, 4);
    std::cout << "Shortest path from 0 to 4: ";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    return 0;
}
