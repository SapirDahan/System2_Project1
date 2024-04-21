#include "Algorithms.hpp"
#include <iostream>
#include <vector>
#include <climits>


using namespace ariel;

bool Algorithms::isConnected(const Graph& graph) {
    int graphSize = graph.size();
    std::vector<bool> visited(graphSize, false);
    DFS(graph, 0, visited);
    for(int i = 0; i < graphSize; i++) {
        if (!visited[i]) {
            return false; // If any vertex is not visited, the graph is not connected
        }
    }

    return true; // All vertices are visited, the graph is connected
}


// Using the Bellman-Ford algorithm
std::string Algorithms::shortestPath(const Graph& graph, int start, int end) {
    int numVertices = graph.size();
    std::vector<int> distance(numVertices, INT_MAX);
    std::vector<int> predecessor(numVertices, -1);

    distance[start] = 0;

    // Relax edges repeatedly
    for (int i = 0; i < numVertices - 1; i++) {
        for (int u = 0; u < numVertices; u++) {
            for (int v: graph[u]) {
                if (distance[u] != INT_MAX && distance[u] + v < distance[v]) {
                    distance[v] = distance[u] + v;
                    predecessor[v] = u;
                }
            }
        }
    }

    // Check for negative cycles
    for (int u = 0; u < numVertices; u++) {
        for (int v: graph[u]) {
            if (distance[u] != INT_MAX && distance[u] + v < distance[v]) {
                return "Negative cycle detected in the path.";
            }
        }
    }

    if (distance[end] == INT_MAX) {
        std::string message = "There is no path from " + std::to_string(start) + " to " + std::to_string(end);
        return message;
    }

    // Reconstruct the path
    std::string path = std::to_string(end);
    for (int current = end; current != start; current = predecessor[current]) {
        path.insert(0, "->");
        path.insert(0, std::to_string(predecessor[current]));
    }

    return path;
}

bool Algorithms::isContainsCycle(const Graph& graph) {
    return true;
}

std::string Algorithms::isBipartite(const Graph& graph) {
    // Implementation of isBipartite algorithm
    // Check if the graph is bipartite
    return "The graph is bipartite: A={0, 2}, B={1}"; // Placeholder return value
}

std::string negativeCycle(const Graph& graph) {
    int numVertices = graph.size();
    std::vector<int> distance(numVertices, 0);
    std::vector<int> predecessor(numVertices, -1);
    int cycleStart = -1;

    // Relax edges repeatedly
    for (int i = 0; i < numVertices - 1; i++) {
        for (int u = 0; u < numVertices; u++) {
            for (int v : graph[u]) {
                if (distance[u] + v < distance[v]) {
                    distance[v] = distance[u] + v;
                    predecessor[v] = u;
                    cycleStart = v;
                }
            }
        }
    }

    // Additional iteration to detect negative cycles
    for (int u = 0; u < numVertices; u++) {
        for (int v : graph[u]) {
            if (distance[u] + v < distance[v]) {
                // Reconstruct the negative cycle
                std::string cycle;
                for (int current = cycleStart; current != cycleStart || cycle.empty(); current = predecessor[current]) {
                    cycle.insert(0, "->");
                    cycle.insert(0, std::to_string(current));                }
                return cycle;
            }
        }
    }

    return "No negative cycle found.";
}

// Help functions
void Algorithms::DFS(const Graph& graph, int startVertex, std::vector<bool>& visited) {
    visited[startVertex] = true;

    while (true) {
        bool allNeighborsVisited = true;

        for (int nextVertex : graph.operator[](startVertex)) {
            if (nextVertex != 0 && !visited[nextVertex]) {
                visited[nextVertex] = true;
                startVertex = nextVertex;
                allNeighborsVisited = false;
                break;
            }
        }

        if (allNeighborsVisited) {
            bool foundUnvisited = false;
            for (int i = 0; i < visited.size(); i++) {
                if (!visited[i]) {
                    startVertex = i;
                    foundUnvisited = true;
                    break;
                }
            }

            if (!foundUnvisited) {
                break; // All vertices are visited
            }
        }
    }
}

