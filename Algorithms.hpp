# pragma once

#include "Graph.hpp"
#include <unordered_set> // For Bipartite

using namespace std;

namespace ariel {
    class Algorithms {
    public:
        static bool isConnected(const Graph& graph);
        static string shortestPath(const Graph& graph, unsigned int start, unsigned int end);
        static string isContainsCycle(const Graph& graph);
        static string isBipartite(const Graph& graph);
        static string negativeCycle(const Graph& graph);
    private:
        static bool hasPath(const Graph& graph, unsigned int start, unsigned int end, vector<bool>& visited);
        //static void DFS(const Graph& graph, unsigned int currentVertex, vector<bool>& visited);
        static bool DFSForDetectingCycles(unsigned int node, unsigned int parentNode, const Graph& graph, vector<bool>& visited, vector<unsigned int>& parent, string& cyclePath);
        static bool bfs(const Graph& graph, int start, vector<int>& color, unordered_set<int>& A, unordered_set<int>& B);
        static void formatBipartiteSets(const unordered_set<int>& A, const unordered_set<int>& B, string& result);
        static string setToString(const unordered_set<int>& s);
        static unsigned int countOccurrences(const string& str, char target);
    };
}