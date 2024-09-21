#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <limits>
#include <vector>

class Dijkstra
{
  public:
    Dijkstra(int vertices);
    void addEdge(int u, int v, double weight);
    std::vector<int> shortestPath(int start, int end);
    double getDistance(int start, int end);

  private:
    int vertices_;
    std::vector<std::vector<std::pair<int, double>>> adj_list_;
};

#endif // DIJKSTRA_HPP
