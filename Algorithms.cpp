#include "Algorithms.hpp"
#include <iostream>
#include <vector>
#include <climits>


using namespace ariel;

bool Algorithms::isConnected(const Graph& graph) {
    const unsigned int graphSize = graph.size();
    std::vector<bool> visited(graphSize, false);

    for(unsigned int i = 0; i < graphSize; i++) {
        for(unsigned int j = 0; j < graphSize; j++) {
            if(i != j) {
                if(!hasPath(graph, i , j, visited)) {
                    return false;
                }
                visited.assign(graphSize, false);
            }
        }
    }
    return true;
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
//     const unsigned int numVertices = graph.size();
//     std::vector<int> distance(numVertices, INT_MAX);
//     std::vector<unsigned int> predecessor(numVertices, INT_MAX);
//     int cycleStart = -1;
//
//     distance[0] = 0;
//
//     // Relax edges repeatedly
//     for (unsigned int i = 0; i < numVertices; i++) {
//         cycleStart = -1;
//         for (unsigned int u = 0; u < numVertices; u++) {
//             for (const auto v : graph.operator[](u)) {
//                 if(graph.operator[](u)[static_cast<unsigned int>(v)] != 0 && distance[u] < INT_MAX && distance[u] + graph.operator[](u)[static_cast<unsigned int>(v)] < distance[static_cast<unsigned int>(v)]) {
//                     distance[static_cast<unsigned int>(v)] = max(distance[u] + graph.operator[](u)[static_cast<unsigned int>(v)], -INT_MAX);
//                     predecessor[static_cast<unsigned int>(v)] = u;
//                     cycleStart = v;
//                 }
//             }
//         }
//     }
//
//     if(cycleStart < 0) {
//          return "No negative cycle found.";
//     }
//     else {
//         for(size_t i = 0; i < numVertices; i++) {
//             cycleStart = static_cast<int>(predecessor[static_cast<unsigned int>(cycleStart)]);
//         }
//         string cycle;
//         auto v = static_cast<unsigned int>(cycleStart);
//         do {
//             cycle += to_string(v);
//             cycle += "->";
//             v = static_cast<unsigned int>(predecessor[v]);
//         } while(v != cycleStart);
//
//         //remove te last ->
//         cycle = cycle.substr(0, cycle.length() - 2);
//         return cycle;
//     }
    // const unsigned int numVertices = graph.size();
    // std::vector<int> distance(numVertices, INT_MAX);
    // std::vector<int> predecessor(numVertices, -1);
    //
    // int cycleStart = -1;
    //
    // distance[0] = 0;
    //
    // // Relax edges repeatedly
    // for (unsigned int i = 0; i < numVertices; i++) {
    //     cycleStart = -1;
    //     for (unsigned int u = 0; u < numVertices; u++) {
    //         for (const auto v : graph[u]) {
    //             const unsigned int y = static_cast<unsigned int>(v);
    //             if (distance[u] < INT_MAX && graph[u][y] != 0 && distance[u] + graph[u][y] < distance[y]) {
    //                 distance[y] = distance[u] + graph[u][y];
    //                 predecessor[y] = u;
    //                 cycleStart = v;
    //             }
    //         }
    //     }
    // }
    //
    // if (cycleStart < 0) {
    //     return "No negative cycle found.";
    // } else {
    //     unsigned int v = static_cast<unsigned int>(cycleStart);
    //     std::string cycle;
    //     do {
    //         cycle += std::to_string(v);
    //         cycle += "->";
    //         v = predecessor[v];
    //     } while (v != static_cast<unsigned int>(cycleStart));
    //
    //     // Remove the last "->"
    //     cycle = cycle.substr(0, cycle.length() - 2);
    //     return cycle;
    // }

    // const unsigned int V = graph.size();
    // std::vector<int> distance(V, std::numeric_limits<int>::max());
    // std::vector<int> predecessor(V, -1);
    //
    // // Start from each vertex to handle disconnected graphs
    // for (unsigned int start = 0; start < V; start++) {
    //     if (distance[start] == std::numeric_limits<int>::max()) {
    //         distance[start] = 0;
    //
    //         // Relax all edges V-1 times
    //         for (unsigned int i = 0; i < V - 1; ++i) {
    //             for (unsigned int u = 0; u < V; ++u) {
    //                 for (unsigned int v = 0; v < V; ++v) {
    //                     int weight = graph.getEdge(u, v);
    //                     if (weight != 0 && distance[u] < std::numeric_limits<int>::max() && distance[u] + weight < distance[v]) {
    //                         distance[v] = distance[u] + weight;
    //                         predecessor[v] = static_cast<int>(u);
    //                     }
    //                 }
    //             }
    //         }
    //         // Check for negative cycles
    //         for (unsigned int u = 0; u < V; ++u) {
    //             for (unsigned int v = 0; v < V; ++v) {
    //
    //                 int weight = graph.getEdge(u, v);
    //
    //                 if (weight != 0 && distance[u] < std::numeric_limits<int>::max() && distance[u] + weight < distance[v]) {
    //                     // Negative cycle detected, construct the cycle path
    //
    //                     std::vector<int> cycle;
    //                     unsigned int current = v;
    //
    //                     // To ensure we get the complete cycle, even if we detect it in the middle
    //                     //for (int i = 0; i < V; ++i) current = static_cast<unsigned int>(predecessor[current]);
    //
    //                     unsigned int startOfCycle = current;
    //                     do {
    //                         cycle.push_back(static_cast<int>(current));
    //                         current = static_cast<unsigned int>(predecessor[current]);
    //                     } while (current != startOfCycle);
    //
    //                     cycle.push_back(static_cast<int>(startOfCycle)); // Complete the cycle
    //
    //                     // Reverse the cycle to start with the first vertex found in negative cycle
    //                     //std::reverse(cycle.begin(), cycle.end());
    //
    //                     // Convert cycle to string format
    //                     std::string cycleStr;
    //                     for (unsigned int i = 0; i < cycle.size(); ++i) {
    //                         cycleStr += std::to_string(cycle[i]);
    //                         if (i < cycle.size() - 1) cycleStr += "->";
    //                     }
    //                     return cycleStr;
    //                 }
    //             }
    //         }
    //
    //         // Reset distances for next start vertex
    //         std::fill(distance.begin(), distance.end(), std::numeric_limits<int>::max());
    //         std::fill(predecessor.begin(), predecessor.end(), -1);
    //     }
    // }
    //
    // return "No negative cycle found.";


    const unsigned int V = graph.size();
    std::vector<int> distance(V, std::numeric_limits<int>::max());
    std::vector<int> predecessor(V, -1);

    // Start from each vertex to handle disconnected graphs
    for (unsigned int start = 0; start < V; start++) {
        if (distance[start] == std::numeric_limits<int>::max()) {
            distance[start] = 0;

            // Relax all edges V-1 times
            for (unsigned int i = 0; i < V - 1; ++i) {
                for (unsigned int u = 0; u < V; ++u) {
                    for (unsigned int v = 0; v < V; ++v) {
                        int weight = graph.getEdge(u, v);
                        if (weight != 0 && distance[u] < std::numeric_limits<int>::max() && distance[u] + weight < distance[v]) {
                        distance[v] = distance[u] + weight;
                        predecessor[v] = static_cast<int>(u);
                    }
                }
            }
        }

        // Reset distances for next start vertex
        std::fill(distance.begin(), distance.end(), std::numeric_limits<int>::max());
        std::fill(predecessor.begin(), predecessor.end(), -1);
        }
    }

    // Check for negative cycles after all shortest paths have been computed
    for (unsigned int u = 0; u < V; ++u) {
        for (unsigned int v = 0; v < V; ++v) {
            int weight = graph.getEdge(u, v);
            if (weight != 0 && distance[u] < std::numeric_limits<int>::max() && distance[u] + weight < distance[v]) {
                // Negative cycle detected, construct the cycle path
                std::vector<int> cycle;
                unsigned int current = v;
                unsigned int startOfCycle = current;
                do {
                    cycle.push_back(static_cast<int>(current));
                    current = static_cast<unsigned int>(predecessor[current]);
                } while (current != startOfCycle);

                cycle.push_back(static_cast<int>(startOfCycle)); // Complete the cycle

                // Convert cycle to string format
                std::string cycleStr;
                for (unsigned int i = 0; i < cycle.size(); ++i) {
                    cycleStr += std::to_string(cycle[i]);
                    if (i < cycle.size() - 1) cycleStr += "->";
                }
                cout<<cycleStr<<endl;
                return cycleStr;
            }
        }
    }

    return "No negative cycle found.";



    // const unsigned int V = graph.size();
    // std::vector<int> distance(V);
    // std::vector<int> predecessor(V);
    //
    // // Attempt to find a negative cycle starting from each vertex
    // for (unsigned int start = 0; start < V; ++start) {
    //     std::fill(distance.begin(), distance.end(), std::numeric_limits<int>::max());
    //     std::fill(predecessor.begin(), predecessor.end(), -1);
    //
    //     distance[start] = 0; // Start from this vertex
    //
    //     // Relax all edges V-1 times
    //     for (unsigned int i = 1; i < V; ++i) {
    //         for (unsigned int u = 0; u < V; ++u) {
    //             for (unsigned int v = 0; v < V; ++v) {
    //                 int weight = graph.getEdge(u, v);
    //                 if (weight != 0 && distance[u] < std::numeric_limits<int>::max() && distance[u] + weight < distance[v]) {
    //                     distance[v] = distance[u] + weight;
    //                     predecessor[v] = static_cast<int>(u);
    //                 }
    //             }
    //         }
    //     }
    //
    //     // Check for negative cycles
    //     for (unsigned int u = 0; u < V; ++u) {
    //         for (unsigned int v = 0; v < V; ++v) {
    //             int weight = graph.getEdge(u, v);
    //             if (weight != 0 && distance[u] + weight < distance[v]) {
    //                 // Negative cycle detected, reconstruct the cycle path
    //                 std::vector<bool> visited(V, false);
    //                 std::vector<int> cycle;
    //                 unsigned int current = v;
    //
    //                 // Trace back the cycle
    //                 for(unsigned int i = 0; i < V; ++i) current = static_cast<unsigned int>(predecessor[current]);
    //
    //                 unsigned int startOfCycle = current;
    //                 do {
    //                     cycle.push_back(static_cast<int>(current));
    //                     current = static_cast<unsigned int>(predecessor[current]);
    //                 } while(current != startOfCycle);
    //
    //                 cycle.push_back(static_cast<int>(startOfCycle)); // Complete the cycle
    //
    //                 // Convert cycle to string format
    //                 std::string cycleStr;
    //                 for (unsigned int i = cycle.size() - 1; i >= 0; --i) { // Reverse the cycle to start from the beginning
    //                     if(i != cycle.size() - 1) cycleStr += "->";
    //                     cycleStr += std::to_string(cycle[i]);
    //                 }
    //
    //                 return cycleStr;
    //             }
    //         }
    //     }
    // }
    //
    // return "No negative cycle found.";
    //
    //

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
    bool neighborsInA = false;
    bool neighborsInB = false;
    for(const unsigned int startNeighbors : graph.getConnectedVertices(static_cast<unsigned int>(startVertex))) {
        if (color[static_cast<unsigned int>(startNeighbors)] == 1) {
            neighborsInA = true;
        }
        if (color[static_cast<unsigned int>(startNeighbors)] == 2) {
            neighborsInB = true;
        }
    }

    if (neighborsInA && neighborsInB) {
        return false;
    }

    if(neighborsInA) {
        color[static_cast<unsigned int>(startVertex)] = 2; // Assign the first color
        B.insert(startVertex);

    }

    else {
        color[static_cast<unsigned int>(startVertex)] = 1; // Assign the first color
        A.insert(startVertex);

    }


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
    for (const int elem : s) {
        str = ", " + std::to_string(elem) + str;
    }

    if (!str.empty()) {
        str.erase(0, 2); // Erase the first two characters
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

bool Algorithms::hasPath(const Graph& graph, unsigned int currentVertex, unsigned int toVertex, std::vector<bool>& visited) {
    // Mark the current vertex as visited
    visited[currentVertex] = true;

    // Base case: If we reach the destination vertex, return true
    if (currentVertex == toVertex) {
        return true;
    }

    // Recursive step: Explore all adjacent vertices of currentVertex
    const std::vector<unsigned int>& edges = graph.getConnectedVertices(currentVertex);
    for (unsigned int i = 0; i < edges.size(); i++) {
        unsigned int adjacentVertex = edges[i];
        if (!visited[adjacentVertex]) {
            // Recursively call hasPath for unvisited adjacent vertices
            if (hasPath(graph, adjacentVertex, toVertex, visited)) {
                return true; // If a path is found, return true
            }
        }
    }

    // If we reach here, there is no path from currentVertex to toVertex
    return false;
}





