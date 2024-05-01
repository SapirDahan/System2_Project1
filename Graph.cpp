#include "Graph.hpp"

using namespace ariel;


// Load the graph and check if it square
void Graph::loadGraph(const std::vector<std::vector<int>>& graph) {

    // Check the matrix is square
    for(unsigned int i = 0; i< graph.size(); i++){
        if (graph.size() != graph[i].size()) {
            throw invalid_argument("Invalid graph: The graph is not a square matrix.");
        }
    }
    // Assign the graph to the matrix member variable
    matrix = graph;
}

// Return how many vertex and edge the graph have
string Graph::printGraph() const {
    int edges = 0;

    // counting the edges
    for (const auto& edge : matrix) {
        for (int num : edge) {
            if (num != 0) {
                edges++;
            }
        }
    }

    // Constrate the string
    string description = "Graph with " + std::to_string(matrix.size()) + " vertices and " + to_string(edges) + " edges.";
    return description;
}

// Return the size
unsigned int Graph::size() const{
    return matrix.size();
}

// Return the row of the matrix
const std::vector<int>& Graph::operator[](unsigned int index) const {
    return matrix[static_cast<vector<vector<int>>::size_type>(index)];
}

// Get the edge
const int Graph::getEdge(unsigned int x, unsigned int y) const{

    // Check if x and y are within the bounds of the matrix
    if (x >= matrix.size() || y >= matrix[0].size()) {
        throw out_of_range("Index out of range");
    }

    return matrix[x][y];
}

// Get the vertex how are not 0
vector<unsigned int> Graph::getConnectedVertices(unsigned int vertex) const {

    vector<unsigned int> connectedVertices;

    // Check the bound
    if (vertex >= matrix.size()) {
        throw out_of_range("Vertex index out of range");
    }

    // Find the edges
    const vector<int>& edges = matrix[vertex];
    for (unsigned int i = 0; i < edges.size(); ++i) {
        if (edges[i] != 0) {
            connectedVertices.push_back(i);
        }
    }

    // Return the connected verteces
    return connectedVertices;
}
