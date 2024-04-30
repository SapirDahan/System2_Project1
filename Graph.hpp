#pragma once

#include <iostream>
#include <vector>

using namespace std;

namespace ariel{
    class Graph{
    private:
        vector<vector<int>> matrix;

    public:
        void loadGraph(const vector<vector<int>>& graph);
        [[nodiscard]] std::string printGraph() const;

        // Help functions
        [[nodiscard]] unsigned int size() const;
        const std::vector<int>& operator[](unsigned int index) const;
        const int getEdge(unsigned int x, unsigned int y) const;
        [[nodiscard]] std::vector<unsigned int> getConnectedVertices(unsigned int vertex) const;

    };
}