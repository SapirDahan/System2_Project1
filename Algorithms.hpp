#include "Graph.hpp"

namespace ariel {
    class Algorithms {
    public:
        static bool isConnected(const Graph& graph);
        static std::string shortestPath(const Graph& graph, int start, int end);
        static bool isContainsCycle(const Graph& graph);
        static std::string isBipartite(const Graph& graph);
        static std::string negativeCycle(const Graph& graph);
        static void DFS(const Graph& graph, int currentVertex, std::vector<bool>& visited);
    };
}