#include "Graph.hpp"

using namespace ariel;

void Graph::loadGraph(const std::vector<std::vector<int>>& graph) {
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
int Graph::size() const{
    return matrix.size();
}

const std::vector<int>& Graph::operator[](size_t index) const {
    return matrix[index];
}