#include <algorithm>
#include <queue>

#include "Dijkstra.hpp"

Dijkstra::Dijkstra(int vertices) : vertices_(vertices)
{
    adj_list_.resize(vertices);
}

void Dijkstra::addEdge(int u, int v, double weight)
{
    adj_list_[u].emplace_back(v, weight);
    adj_list_[v].emplace_back(u, weight); // для неориентированного графа
}

std::vector<int> Dijkstra::shortestPath(int start, int end)
{
    std::vector<double> dist(vertices_, std::numeric_limits<double>::infinity());
    std::vector<int> prev(vertices_, -1);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    dist[start] = 0;
    pq.emplace(0, start);

    while (!pq.empty())
    {
        int u = pq.top().second;
        pq.pop();

        if (u == end)
            break;

        for (const auto &[v, weight] : adj_list_[u])
        {
            double alt = dist[u] + weight;
            if (alt < dist[v])
            {
                dist[v] = alt;
                prev[v] = u;
                pq.emplace(alt, v);
            }
        }
    }

    std::vector<int> path;
    for (int at = end; at != -1; at = prev[at])
    {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

double Dijkstra::getDistance(int start, int end)
{
    std::vector<double> dist(vertices_, std::numeric_limits<double>::infinity());
    dist[start] = 0;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    pq.emplace(0, start);

    while (!pq.empty())
    {
        int u = pq.top().second;
        pq.pop();

        if (u == end)
            break;

        for (const auto &[v, weight] : adj_list_[u])
        {
            double alt = dist[u] + weight;
            if (alt < dist[v])
            {
                dist[v] = alt;
                pq.emplace(alt, v);
            }
        }
    }

    return dist[end];
}
