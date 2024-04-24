#include "Algorithms.hpp"
#include <iostream>
#include <vector>
#include <climits>


using namespace ariel;

bool Algorithms::isConnected(const Graph& graph) {
    const unsigned int graphSize = graph.size();
    std::vector<bool> visited(graphSize, false);

    DFS(graph, graphSize - 1, visited);

    for(size_t i = 0; i < graphSize; i++) {
        if (!visited[i]) {
            return false; // If any vertex is not visited, the graph is not connected
        }
    }

    return true; // All vertices are visited, the graph is connected
}


// Using the Bellman-Ford algorithm
std::string Algorithms::shortestPath(const Graph& graph, int start, int end) {
    const unsigned int numVertices = graph.size();
    std::vector<int> distance(numVertices, INT_MAX);
    std::vector<int> predecessor(numVertices, -1);

    distance[static_cast<unsigned int>(start)] = 0;

    // Relax edges repeatedly
    for (unsigned int i = 0; i < numVertices - 1; i++) {
        for (unsigned int u = 0; u < numVertices; u++) {
            for (const auto v : graph.operator[](u)) {
                if (distance[u] != INT_MAX && distance[u] + static_cast<int>(v) < distance[static_cast<unsigned int>(v)]) {
                    cout<<"get in"<<endl;
                    distance[static_cast<unsigned int>(v)] = distance[u] + static_cast<int>(v);
                    predecessor[static_cast<unsigned int>(v)] = static_cast<int>(u);
                }
            }
        }
    }

    // Check for negative cycles
    for (unsigned int u = 0; u < numVertices; u++) {
        for (const auto v : graph.operator[](u)) {
            if (distance[u] != INT_MAX && distance[u] + static_cast<int>(v) < distance[static_cast<unsigned int>(v)]) {
                return "Negative cycle detected in the path.";
            }
        }
    }

    if (distance[static_cast<unsigned int>(end)] == INT_MAX) {
        std::string message = "There is no path from " + std::to_string(start) + " to " + std::to_string(end);
        return message;
    }

    // Reconstruct the path
    std::string path = std::to_string(end);
    for (unsigned int current = static_cast<unsigned int>(end); current != static_cast<unsigned int>(start); current = static_cast<unsigned int>(predecessor[current])) {
        path.insert(0, "->");
        path.insert(0, std::to_string(predecessor[current]));
    }

    return path;
}

bool Algorithms::DFSForDetectingCycles(const Graph& graph, int startVertex, std::vector<bool>& visited, std::vector<int>& parent, std::string& cyclePath) {
    visited[static_cast<unsigned int>(startVertex)] = true;

    for(const auto nextVertex : graph.operator[](static_cast<unsigned int>(startVertex))) {
        if (!visited[static_cast<unsigned int>(nextVertex)]) {
            parent[static_cast<unsigned int>(nextVertex)] = startVertex;
            if (DFSForDetectingCycles(graph, static_cast<int>(nextVertex), visited, parent, cyclePath)) {
                if (!cyclePath.empty() && cyclePath.back() != '>') {
                    cyclePath += "->"; // Add arrow separator between vertices
                }
                cyclePath += std::to_string(nextVertex); // Add the vertex to the path

                if (nextVertex == startVertex) {
                    return true; // Cycle found
                }
                return false;
            }
        } else if (nextVertex != parent[static_cast<unsigned int>(startVertex)]) {
            // Cycle detected, construct the cycle path
            int current = startVertex;
            while (current != nextVertex) {
                cyclePath += std::to_string(current) + "->";
                current = parent[static_cast<unsigned int>(current)];
            }
            cyclePath += std::to_string(nextVertex) + "->" + std::to_string(startVertex); // Complete the cycle path
            return true;
        }
    }

    return false;
}

std::string Algorithms::isContainsCycle(const Graph& graph) {
    const unsigned int numVertices = graph.size();
    std::vector<bool> visited(numVertices, false);
    std::vector<int> parent(numVertices, -1);
    std::string cyclePath;

    for (unsigned int i = 0; i < numVertices; ++i) {
        if (!visited[i]) {
            if (DFSForDetectingCycles(graph, static_cast<int>(i), visited, parent, cyclePath)) {
                return cyclePath; // Cycle found
            }
        }
    }
    return "0"; // No cycle found
}

std::string Algorithms::isBipartite(const Graph& graph) {
    const unsigned int numVertices = graph.size();
    std::vector<int> color(numVertices, -1);
    std::unordered_set<int> A, B;

    for (unsigned int i = 0; i < numVertices; ++i) {
        if (color[i] == -1) {
            if (!bfs(graph, static_cast<int>(i), color, A, B)) {
                return "0"; // Not bipartite
            }
        }
    }

    return formatBipartiteSets(A, B);
}

std::string Algorithms::negativeCycle(const Graph& graph) {
    const unsigned int numVertices = graph.size();
    std::vector<int> distance(numVertices, INT_MAX);
    std::vector<unsigned int> predecessor(numVertices, INT_MAX);
    int cycleStart = -1;

    distance[0] = 0;

    // Relax edges repeatedly
    for (unsigned int i = 0; i < numVertices; i++) {
        cycleStart = -1;
        for (unsigned int u = 0; u < numVertices; u++) {
            for (const auto v : graph.operator[](u)) {
                if(graph.operator[](u)[static_cast<unsigned int>(v)] != 0 && distance[u] < INT_MAX && distance[u] + graph.operator[](u)[static_cast<unsigned int>(v)] < distance[static_cast<unsigned int>(v)]) {
                    distance[static_cast<unsigned int>(v)] = max(distance[u] + graph.operator[](u)[static_cast<unsigned int>(v)], -INT_MAX);
                    predecessor[static_cast<unsigned int>(v)] = u;
                    cycleStart = v;
                }
            }
        }
    }

    if(cycleStart < 0) {
         return "No negative cycle found.";
    }
    else {
        for(size_t i = 0; i < numVertices; i++) {
            cycleStart = static_cast<int>(predecessor[static_cast<unsigned int>(cycleStart)]);
        }
        string cycle;
        auto v = static_cast<unsigned int>(cycleStart);
        do {
            cycle += to_string(v);
            cycle += "->";
            v = static_cast<unsigned int>(predecessor[v]);
        } while(v != cycleStart);

        //remove te last ->
        cycle = cycle.substr(0, cycle.length() - 2);
        return cycle;
    }


}

// Help functions
void Algorithms::DFS(const Graph& graph, unsigned int startVertex, std::vector<bool>& visited) {
    visited[startVertex] = true; // Mark the current vertex as visited

    for (const auto nextVertex : graph.operator[](startVertex)) {
        if (!visited[static_cast<unsigned int>(nextVertex)]) {
            DFS(graph, static_cast<unsigned int>(nextVertex), visited); // Recursively visit the neighbor
        }
    }
}

bool Algorithms::bfs(const Graph& graph, int startVertex, std::vector<int>& color, std::unordered_set<int>& A, std::unordered_set<int>& B) {
    std::queue<unsigned int> q;
    q.push(static_cast<unsigned int>(startVertex));
    color[static_cast<unsigned int>(startVertex)] = 0; // Assign the first color

    while (!q.empty()) {
        const unsigned int currVertex = q.front();
        q.pop();

        for (const auto nextVertex : graph.operator[](currVertex)) {
            if (color[static_cast<unsigned int>(nextVertex)] == -1) {
                color[static_cast<unsigned int>(nextVertex)] = 1 - color[currVertex]; // Assign the opposite color
                (color[static_cast<unsigned int>(nextVertex)] == 0 ? A : B).insert(nextVertex); // Add to the corresponding set
                q.push(static_cast<unsigned int>(nextVertex));
            }
            else if (color[static_cast<unsigned int>(nextVertex)] == color[currVertex]) {
                return false; // Not bipartite
            }
        }
    }

    return true; // Bipartite
}


std::string Algorithms::setToString(const std::unordered_set<int>& s) {
    std::string str;
    for (int elem : s) {
        str += std::to_string(elem) + ", ";
    }
    if (!str.empty()) {
        str.pop_back(); // Remove the last comma
        str.pop_back(); // Remove the space
    }
    return str;
}

std::string Algorithms::formatBipartiteSets(const std::unordered_set<int>& A, const std::unordered_set<int>& B) {
    std::string result = "The graph is bipartite: A={";
    result += setToString(A);
    result += "}, B={";
    result += setToString(B);
    result += "}";
    return result;
}
