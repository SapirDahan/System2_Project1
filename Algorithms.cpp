#include "Algorithms.hpp"
#include <vector>
#include <climits>
#include <queue> // For Bipartite


using namespace ariel;


// Checking if the graph is strongly connected
bool Algorithms::isConnected(const Graph& graph) {

    const unsigned int graphSize = graph.size(); // Size of the graph

    // A vector that save who we visited
    std::vector<bool> visited(graphSize, false);

    // Going over all combinations of paths
    for(unsigned int i = 0; i < graphSize; i++) {
        for(unsigned int j = 0; j < graphSize; j++) {

            // We dont need to check if a vertex have a path to himself
            if(i != j) {

                // If there is no path return false because the graph is not strongly connected
                if(!hasPath(graph, i , j, visited)) {
                    return false;
                }

                // Refill the visited vector with false for the next round
                visited.assign(graphSize, false);
            }
        }
    }

    // All paths exict, therefore, our graph is strongly connected
    return true;
}


// Shortest path using the Bellman-Ford algorithm
string Algorithms::shortestPath(const Graph& graph, const unsigned int start, const unsigned int end) {

    const unsigned int numVertices = graph.size(); // Size of the graph

    // Check if the valuse are in range
    if(start >= numVertices || end >= numVertices) {
        throw std::invalid_argument("Invalid values: the start and the end vertex should be in range.");
    }

    std::vector<int> distance(numVertices, INT_MAX);
    std::vector<int> predecessor(numVertices, -1);

    distance[start] = 0;

    for (unsigned int i = 0; i < numVertices; i++) {
        for (unsigned int j = 0; j < numVertices; j++) {
            for (unsigned int k = 0; k < numVertices; k++) {

                // Checking if there an edge
                if(graph.getEdge(j,k) != 0) {
                    if (distance[j] != INT_MAX && distance[j] + graph.getEdge(j,k) < distance[k]) {
                        distance[k] = distance[j] + graph.getEdge(j,k);
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
                if (distance[j] != INT_MAX && distance[j] + graph.getEdge(j,k) < distance[k]) {
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
    for (unsigned int current = end; current != start; current = static_cast<unsigned int>(predecessor[current])) {
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
    const unsigned int numVertices = graph.size();

    for(unsigned int i = 0; i < numVertices; i++) {
        if(shortestPath(graph, i, i) == "Negative cycle detected in the path.") {
            return "Negative cycle detected in the path.";
        }
    }

    return "No negative cycle detected in the path.";
}

/////// Help functions ///////

// Help recursive function to find out if 2 vertex have a path
bool Algorithms::hasPath(const Graph& graph, unsigned int start, unsigned int end, std::vector<bool>& visited) {

    // Mark the current vertex as visited
    visited[start] = true;

    // Base case: If we reach the destination vertex, return true
    if (start == end) {
        return true;
    }

    // Recursive step: Explore all adjacent vertices of the start vertex
    const vector<unsigned int>& edges = graph.getConnectedVertices(start);

    // Going over all the edges
    for (unsigned int adjacentVertex : edges) {

        // Recursively call hasPath for unvisited adjacent vertices
        if (!visited[adjacentVertex]) {
            if (hasPath(graph, adjacentVertex, end, visited)) {
                return true; // If a path is found, return true
            }
        }
    }

    // If we reach here, there is no path from start vertex to end vertex
    return false;
}

// void Algorithms::DFS(const Graph& graph, unsigned int startVertex, std::vector<bool>& visited) {
//     visited[startVertex] = true; // Mark the current vertex as visited
//
//     for (const auto nextVertex : graph.operator[](startVertex)) {
//         if (!visited[static_cast<unsigned int>(nextVertex)]) {
//             DFS(graph, static_cast<unsigned int>(nextVertex), visited); // Recursively visit the neighbor
//         }
//     }
// }

bool Algorithms::DFSForDetectingCycles(unsigned int node, unsigned int parentNode, const Graph& graph, vector<bool>& visited, vector<unsigned int>& parent, string& cyclePath) {

    visited[node] = true;
    parent[node] = parentNode;
    for (const unsigned int nextNode : graph.getConnectedVertices(node)) {
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

unsigned int Algorithms::countOccurrences(const std::string& str, char target) {
    unsigned int count = 0;
    for (const char ch : str) {
        if (ch == target) {
            count++;
        }
    }
    return count;
}

