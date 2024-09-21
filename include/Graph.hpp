#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>

class Graph
{
  public:
    Graph(int vertices);
    void addEdge(int u, int v, double weight);
    const std::vector<std::pair<int, double>> &getAdjList(int u) const;

  private:
    int vertices_;
    std::vector<std::vector<std::pair<int, double>>> adj_list_;
};

#endif // GRAPH_HPP
