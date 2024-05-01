#pragma once

#include <iostream>
#include <vector>

using namespace std;

namespace ariel{
    class Graph{
        vector<vector<int>> matrix;

    public:
        void loadGraph(const vector<vector<int>>& graph);
        string printGraph() const;

        // Help functions
        unsigned int size() const;
        const vector<int>& operator[](unsigned int index) const;
        const int getEdge(unsigned int x, unsigned int y) const;
        vector<unsigned int> getConnectedVertices(unsigned int vertex) const;
    };
}