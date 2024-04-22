#include "Graph.hpp"
#include <queue>
#include <unordered_set>

namespace ariel {
    class Algorithms {
    public:
        static bool isConnected(const Graph& graph);
        static std::string shortestPath(const Graph& graph, size_t start, size_t end);
        static std::string isContainsCycle(const Graph& graph);
        static std::string isBipartite(const Graph& graph);
        static std::string negativeCycle(const Graph& graph);
    private:
        static void DFS(const Graph& graph, size_t currentVertex, std::vector<bool>& visited);
        static bool DFSForDetectingCycles(const Graph& graph, int currentVertex, std::vector<bool>& visited, std::vector<size_t>& parent, std::string& cyclePath);
        static bool bfs(const Graph& graph, size_t currentVertex, std::vector<size_t>& color, std::unordered_set<size_t>& A, std::unordered_set<size_t>& B);
        static std::string formatBipartiteSets(const std::unordered_set<size_t>& A, const std::unordered_set<size_t>& B);
        static std::string setToString(const std::unordered_set<size_t>& s);
    };
}