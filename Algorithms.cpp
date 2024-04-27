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

    for (unsigned int i = 0; i < numVertices; i++) {
        for (unsigned int j = 0; j < numVertices; j++) {
            for (unsigned int k = 0; k < numVertices; k++) {

                // Checking if there an edge
                if(graph.getEdge(j,k) != 0) {
                    if (distance[j] != INT_MAX && distance[j] + static_cast<int>(k) < distance[k]) {
                        //cout<<numVertices<<endl;
                        distance[k] = distance[j] + static_cast<int>(k);
                        predecessor[k] = static_cast<int>(j);
                        }
                }
            }
        }
    }

    // // Relax edges repeatedly
    // for (unsigned int i = 0; i < numVertices - 1; i++) {
    //     for (unsigned int u = 0; u < numVertices; u++) {
    //         for (const auto v : graph.operator[](u)) {
    //             if (distance[u] != INT_MAX && distance[u] + static_cast<int>(v) < distance[static_cast<unsigned int>(v)]) {
    //                 //cout<<numVertices<<endl;
    //                 distance[static_cast<unsigned int>(v)] = distance[u] + static_cast<int>(v);
    //                 predecessor[static_cast<unsigned int>(v)] = static_cast<int>(u);
    //             }
    //         }
    //     }
    // }
    //
    // // Check for negative cycles
    // for (unsigned int u = 0; u < numVertices; u++) {
    //     for (const auto v : graph.operator[](u)) {
    //         if (distance[u] != INT_MAX && distance[u] + static_cast<int>(v) < distance[static_cast<unsigned int>(v)]) {
    //             return "Negative cycle detected in the path.";
    //         }
    //     }
    // }

    for (unsigned int j = 0; j < numVertices; j++) {
        for (unsigned int k = 0; k < numVertices; k++) {

            // Checking if there an edge
            if(graph.getEdge(j,k) != 0) {
                if (distance[j] != INT_MAX && distance[j] + static_cast<int>(k) < distance[k]) {
                    return "Negative cycle detected in the path.";
                }
            }
        }
    }
    if (distance[static_cast<unsigned int>(end)] == INT_MAX) {
        return "-1";
    }

    // Reconstruct the path
    std::string path = std::to_string(end);
    for (unsigned int current = static_cast<unsigned int>(end); current != static_cast<unsigned int>(start); current = static_cast<unsigned int>(predecessor[current])) {
        path.insert(0, "->");
        path.insert(0, std::to_string(predecessor[current]));
    }

    return path;
}

std::string Algorithms::isContainsCycle(const Graph& graph) {
    // const unsigned int numVertices = graph.size();
    // std::vector<bool> visited(numVertices, false);
    // std::vector<int> parent(numVertices, -1);
    // std::string cyclePath;
    //
    // for (unsigned int i = 0; i < numVertices; ++i) {
    //     if (!visited[i]) {
    //         if (DFSForDetectingCycles(graph, static_cast<int>(i), visited, parent, cyclePath)) {
    //             return cyclePath; // Cycle found
    //         }
    //     }
    // }
    // return "0"; // No cycle found

    const unsigned int numVertices = graph.size();
    vector<bool> visited(numVertices, false);
    vector<unsigned int> parent(numVertices, numeric_limits<unsigned int>::max());
    string cyclePath = "";

    for (unsigned int i = 0; i < numVertices; i++) {
        if (!visited[i]) {
            if (DFSForDetectingCycles(i, numeric_limits<unsigned int>::max(), graph, visited, parent, cyclePath)) {
                // Check if the cycle has at least 3 different nodes
                if (countOccurrences(cyclePath, '>') >= 2) {
                    return cyclePath;
                } else {
                    // Reset for next iteration
                    cyclePath = "";
                    fill(visited.begin(), visited.end(), false);
                }
            }
        }
    }
    return "0";
}

std::string Algorithms::isBipartite(const Graph& graph) {
    const unsigned int numVertices = graph.size();
    std::vector<int> color(numVertices, -1);
    std::unordered_set<int> A, B;
    string result = "";
    for (unsigned int i = 0; i < numVertices; ++i) {
        if (color[i] == -1) {
            if (!bfs(graph, static_cast<int>(i), color, A, B)) {
                return "0"; // Not bipartite
            }
        }
    }

    formatBipartiteSets(A, B, result);
    return result;
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

bool Algorithms::DFSForDetectingCycles(unsigned int node, unsigned int parentNode, const Graph& graph, vector<bool>& visited, vector<unsigned int>& parent, string& cyclePath) {
    // visited[static_cast<unsigned int>(startVertex)] = true;
    //
    // for(const auto nextVertex : graph.operator[](static_cast<unsigned int>(startVertex))) {
    //     if (!visited[static_cast<unsigned int>(nextVertex)]) {
    //         parent[static_cast<unsigned int>(nextVertex)] = startVertex;
    //         if (DFSForDetectingCycles(graph, static_cast<int>(nextVertex), visited, parent, cyclePath)) {
    //             if (!cyclePath.empty() && cyclePath.back() != '>') {
    //                 cyclePath += "->"; // Add arrow separator between vertices
    //             }
    //             cyclePath += std::to_string(nextVertex); // Add the vertex to the path
    //
    //             if (nextVertex == startVertex) {
    //                 return true; // Cycle found
    //             }
    //             return false;
    //         }
    //     } else if(nextVertex != parent[static_cast<unsigned int>(startVertex)]) {
    //         // Cycle detected, construct the cycle path
    //         int current = startVertex;
    //         while (current != nextVertex) {
    //             cyclePath += std::to_string(current) + "->";
    //             current = parent[static_cast<unsigned int>(current)];
    //         }
    //         cyclePath += std::to_string(nextVertex) + "->" + std::to_string(startVertex); // Complete the cycle path
    //         return true;
    //     }
    // }
    //
    // return false;
    visited[node] = true;
    parent[node] = parentNode;
    for (const unsigned int nextNode : graph.getConnectedVertices(node)) {
        //unsigned int next = static_cast<unsigned int>(nextNode);
        if (!visited[nextNode]) {
            if (DFSForDetectingCycles(nextNode, node, graph, visited, parent, cyclePath)) {
                return true;
            }
        }
        else if (nextNode != parentNode) {
            // Cycle found
            // unsigned int current = node;
            // cyclePath = to_string(nextNode);
            // while (current != numeric_limits<unsigned int>::max() && current != nextNode) {
            //     cyclePath = std::to_string(current).append("->").append(cyclePath);
            //     current = parent[current];
            // }
            // cyclePath += "->" + to_string(nextNode);
            // return true;
            unsigned int current = node;
            //The cycle is:
            cyclePath = "The cycle is: " + to_string(nextNode); // Initialize cyclePath with the last vertex in the cycle
            while (current != numeric_limits<unsigned int>::max() && current != nextNode) {
                cyclePath += "->" + std::to_string(current); // Append current vertex to the end
                current = parent[current];
            }
            cyclePath += "->" + to_string(nextNode); // Append the last vertex again to close the cycle
            return true;
        }
    }
    return false;
}

bool Algorithms::bfs(const Graph& graph, int startVertex, std::vector<int>& color, std::unordered_set<int>& A, std::unordered_set<int>& B) {
    std::queue<unsigned int> q;
    q.push(static_cast<unsigned int>(startVertex));
    color[static_cast<unsigned int>(startVertex)] = 1; // Assign the first color
    A.insert(startVertex);

    while (!q.empty()) {
        unsigned int currVertex = q.front();
        q.pop();

        for (const unsigned int nextVertex : graph.getConnectedVertices(currVertex)) {
            if (color[static_cast<unsigned int>(nextVertex)] == -1) {

                color[static_cast<unsigned int>(nextVertex)] = 3 - color[currVertex]; // Assign the opposite color
                if(color[static_cast<unsigned int>(nextVertex)] == 1) {
                    A.insert(static_cast<int>(nextVertex));
                }
                else {
                    B.insert(static_cast<int>(nextVertex));
                }
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

void Algorithms::formatBipartiteSets(const std::unordered_set<int>& A, const std::unordered_set<int>& B, std::string& result) {
    result = "The graph is bipartite: A={";
    result.append(setToString(A));
    result.append("}, B={");
    result.append(setToString(B));
    result.append("}.");
}

unsigned int Algorithms::countOccurrences(const std::string& str, const char target) {
    unsigned int count = 0;
    for (const char ch : str) {
        if (ch == target) {
            count++;
        }
    }
    return count;
}
