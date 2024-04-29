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
        static bool DFSForDetectingCycles(unsigned int node, unsigned int parentNode, const Graph& graph, vector<bool>& visited, vector<unsigned int>& parent, string& cyclePath);
        static bool bfs(const Graph& graph, int currentVertex, std::vector<int>& color, std::unordered_set<int>& A, std::unordered_set<int>& B);
        static void formatBipartiteSets(const std::unordered_set<int>& A, const std::unordered_set<int>& B, std::string& result);
        static std::string setToString(const std::unordered_set<int>& s);
        static unsigned int countOccurrences(const std::string& str, const char target);
        static bool hasPath(const Graph& graph, unsigned int currentVertex, unsigned int toVertex, std::vector<bool>& visited);
    };
}