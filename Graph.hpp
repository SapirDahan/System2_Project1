#pragma once

#include <iostream>
#include <vector>

using namespace std;

namespace ariel{
    class Graph{
    private:
        vector<vector<size_t>> matrix;

    public:
        void loadGraph(const vector<vector<size_t>>& graph);
        void printGraph() const;

        // Help functions
        [[nodiscard]] size_t size() const;
        const std::vector<size_t>& operator[](size_t index) const;
    };
}