#include "Algorithms.hpp"
#include <iostream>
#include <vector>
#include <climits>


using namespace ariel;

bool Algorithms::isConnected(const Graph& graph) {
    size_t graphSize = graph.size();
    std::vector<bool> visited(graphSize, false);
    DFS(graph, 0, visited);
    for(size_t i = 0; i < graphSize; i++) {
        if (!visited[i]) {
            return false; // If any vertex is not visited, the graph is not connected
        }
    }

    return true; // All vertices are visited, the graph is connected
}


// Using the Bellman-Ford algorithm
std::string Algorithms::shortestPath(const Graph& graph, size_t start, size_t end) {
    size_t numVertices = graph.size();
    std::vector<size_t> distance(numVertices, INT_MAX);
    std::vector<int> predecessor(numVertices, -1);

    distance[start] = 0;

    // Relax edges repeatedly
    for (size_t i = 0; i < numVertices - 1; i++) {
        for (size_t u = 0; u < numVertices; u++) {
            for (int v : graph[u]) {
                if (distance[u] != INT_MAX && distance[u] + static_cast<size_t>(v) < distance[static_cast<size_t>(v)]) {
                    distance[static_cast<size_t>(v)] = distance[u] + static_cast<size_t>(v);
                    predecessor[static_cast<size_t>(v)] = u;
                }
            }
        }
    }

    // Check for negative cycles
    for (size_t u = 0; u < numVertices; u++) {
        for (int v : graph[u]) {
            if (distance[u] != INT_MAX && distance[u] + static_cast<size_t>(v) < distance[static_cast<size_t>(v)]) {
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
    for (size_t current = end; current != start; current = static_cast<size_t>(predecessor[current])) {
        path.insert(0, "->");
        path.insert(0, std::to_string(predecessor[current]));
    }

    return path;
}

static bool DFSForDetectingCycles(const Graph& graph, int startVertex, std::vector<bool>& visited, std::vector<int>& parent, std::string& cyclePath) {
    visited[startVertex] = true;

    for(size_t nextVertex : graph[startVertex]) {
        if (!visited[nextVertex]) {
            parent[nextVertex] = startVertex;
            if (DFSForDetectingCycles(graph, nextVertex, visited, parent, cyclePath)) {
                if (!cyclePath.empty() && cyclePath.back() != '>') {
                    cyclePath += "->"; // Add arrow separator between vertices
                }
                cyclePath += std::to_string(nextVertex); // Add the vertex to the path

                if (nextVertex == startVertex) {
                    return true; // Cycle found
                }
                return false;
            }
        } else if (nextVertex != parent[startVertex]) {
            // Cycle detected, construct the cycle path
            size_t current = startVertex;
            while (current != nextVertex) {
                cyclePath += std::to_string(current) + "->";
                current = parent[current];
            }
            cyclePath += std::to_string(nextVertex) + "->" + std::to_string(startVertex); // Complete the cycle path
            return true;
        }
    }

    return false;
}

static std::string isContainsCycle(const Graph& graph) {
    size_t numVertices = graph.size();
    std::vector<bool> visited(numVertices, false);
    std::vector<int> parent(numVertices, -1);
    std::string cyclePath;

    for (size_t i = 0; i < numVertices; ++i) {
        if (!visited[i]) {
            if (DFSForDetectingCycles(graph, i, visited, parent, cyclePath)) {
                return cyclePath; // Cycle found
            }
        }
    }
    return ""; // No cycle found
}

std::string Algorithms::isBipartite(const Graph& graph) {
    size_t numVertices = graph.size();
    std::vector<size_t> color(numVertices, -1);
    std::unordered_set<size_t> A, B;

    for (size_t i = 0; i < numVertices; ++i) {
        if (color[i] == -1) {
            if (!bfs(graph, i, color, A, B)) {
                return "0"; // Not bipartite
            }
        }
    }

    return formatBipartiteSets(A, B);
}

std::string negativeCycle(const Graph& graph) {
    size_t numVertices = graph.size();
    std::vector<size_t> distance(numVertices, 0);
    std::vector<size_t> predecessor(numVertices, -1);
    size_t cycleStart = -1;

    // Relax edges repeatedly
    for (size_t i = 0; i < numVertices - 1; i++) {
        for (size_t u = 0; u < numVertices; u++) {
            for (size_t v : graph[u]) {
                if (distance[u] + v < distance[v]) {
                    distance[v] = distance[u] + v;
                    predecessor[v] = u;
                    cycleStart = v;
                }
            }
        }
    }

    // Additional iteration to detect negative cycles
    for (size_t u = 0; u < numVertices; u++) {
        for (size_t v : graph[u]) {
            if (distance[u] + v < distance[v]) {
                // Reconstruct the negative cycle
                std::string cycle;
                for (size_t current = cycleStart; current != cycleStart || cycle.empty(); current = predecessor[current]) {
                    cycle.insert(0, "->");
                    cycle.insert(0, std::to_string(current));                }
                return cycle;
            }
        }
    }

    return "No negative cycle found.";
}

// Help functions
void Algorithms::DFS(const Graph& graph, size_t startVertex, std::vector<bool>& visited) {
    visited[startVertex] = true;

    while (true) {
        bool allNeighborsVisited = true;

        for (size_t nextVertex : graph.operator[](startVertex)) {
            if (nextVertex != 0 && !visited[nextVertex]) {
                visited[nextVertex] = true;
                startVertex = nextVertex;
                allNeighborsVisited = false;
                break;
            }
        }

        if (allNeighborsVisited) {
            bool foundUnvisited = false;
            for (size_t i = 0; i < visited.size(); i++) {
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

static bool bfs(const Graph& graph, size_t startVertex, std::vector<size_t>& color, std::unordered_set<size_t>& A, std::unordered_set<size_t>& B) {
    std::queue<size_t> q;
    q.push(startVertex);
    color[startVertex] = 0; // Assign the first color

    while (!q.empty()) {
        size_t currVertex = q.front();
        q.pop();

        for (size_t nextVertex : graph[currVertex]) {
            if (color[nextVertex] == static_cast<size_t>(-1)) {
                color[nextVertex] = 1 - color[currVertex]; // Assign the opposite color
                (color[nextVertex] == 0 ? A : B).insert(nextVertex); // Add to the corresponding set
                q.push(nextVertex);
            } else if (color[nextVertex] == color[currVertex]) {
                return false; // Not bipartite
            }
        }
    }

    return true; // Bipartite
}


static std::string setToString(const std::unordered_set<size_t>& s) {
    std::string str;
    for (size_t elem : s) {
        str += std::to_string(elem) + ", ";
    }
    if (!str.empty()) {
        str.pop_back(); // Remove the last comma
        str.pop_back(); // Remove the space
    }
    return str;
}

static std::string formatBipartiteSets(const std::unordered_set<size_t>& A, const std::unordered_set<size_t>& B) {
    std::string result = "The graph is bipartite: A={";
    result += setToString(A);
    result += "}, B={";
    result += setToString(B);
    result += "}";
    return result;
}
