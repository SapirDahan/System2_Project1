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
        void printGraph() const;

        // Help functions
        [[nodiscard]] unsigned int size() const;
        const std::vector<int>& operator[](unsigned int index) const;
    };
}