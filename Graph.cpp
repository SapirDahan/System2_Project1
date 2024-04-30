#include "Graph.hpp"

using namespace ariel;

void Graph::loadGraph(const std::vector<std::vector<int>>& graph) {
    // Check if the graph is a square matrix
    if (graph.size() != graph[0].size()) {
        throw std::invalid_argument("Invalid graph: The graph is not a square matrix.");
    }

    // Assign the graph to the matrix member variable
    matrix = graph;
}

std::string Graph::printGraph() const {
    int vertices = 0;

    for (const auto& innerVec : matrix) {
        for (int num : innerVec) {
            if (num != 0) {
                vertices++;
            }
        }
    }

    std::string description = "Graph with " + std::to_string(matrix.size()) + " vertices and " + std::to_string(vertices) + " edges.";
    return description;
}

// Help Functions
unsigned int Graph::size() const{
    return matrix.size();
}

const std::vector<int>& Graph::operator[](unsigned int index) const {
    return matrix[static_cast<std::vector<std::vector<int>>::size_type>(index)];
}

const int Graph::getEdge(unsigned int x, unsigned int y) const{
    // Check if x and y are within the bounds of the matrix
    if (x < 0 || x >= matrix.size() || y < 0 || y >= matrix[0].size()) {
        throw std::out_of_range("Index out of range");
    }

    return matrix[x][y];
}

std::vector<unsigned int> Graph::getConnectedVertices(unsigned int vertex) const {
    std::vector<unsigned int> connectedVertices;
    if (vertex >= matrix.size()) {
        throw std::out_of_range("Vertex index out of range");
    }

    const std::vector<int>& edges = matrix[vertex];
    for (unsigned int i = 0; i < edges.size(); ++i) {
        if (edges[i] != 0) {
            connectedVertices.push_back(i);
        }
    }

    return connectedVertices;
}