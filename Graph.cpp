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

void Graph::printGraph() const {
    int vertices = 0;

    for (const auto& innerVec : matrix) {
        for (int num : innerVec) {
            if (num != 0) {
                vertices++;
            }
        }
    }

    std::cout << "Graph with " << matrix.size() << " vertices and " << vertices << " edges." << std::endl;
}

// Help Functions
unsigned int Graph::size() const{
    return matrix.size();
}

const std::vector<int>& Graph::operator[](unsigned int index) const {
    return matrix[static_cast<std::vector<std::vector<int>>::size_type>(index)];
}