#include "Graph.hpp"
#include <queue>
#include <unordered_set>

namespace ariel {
    class Algorithms {
    public:
        static bool isConnected(const Graph& graph);
        static std::string shortestPath(const Graph& graph, int start, int end);
        static std::string isContainsCycle(const Graph& graph);
        static std::string isBipartite(const Graph& graph);
        static std::string negativeCycle(const Graph& graph);
    private:
        static void DFS(const Graph& graph, unsigned int currentVertex, std::vector<bool>& visited);
        static bool DFSForDetectingCycles(const Graph& graph, int currentVertex, std::vector<bool>& visited, std::vector<int>& parent, std::string& cyclePath);
        static bool bfs(const Graph& graph, int currentVertex, std::vector<int>& color, std::unordered_set<int>& A, std::unordered_set<int>& B);
        static std::string formatBipartiteSets(const std::unordered_set<int>& A, const std::unordered_set<int>& B);
        static std::string setToString(const std::unordered_set<int>& s);
    };
}