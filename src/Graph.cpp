#include "Graph.hpp"

Graph::Graph(int vertices) : vertices_(vertices)
{
    adj_list_.resize(vertices);
}

void Graph::addEdge(int u, int v, double weight)
{
    adj_list_[u].emplace_back(v, weight);
}

const std::vector<std::pair<int, double>> &Graph::getAdjList(int u) const
{
    return adj_list_[u];
}
