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
        static bool DFSForDetectingCycles(unsigned int node, unsigned int parentNode, const Graph& graph, vector<bool>& visited, vector<unsigned int>& parent, string& cyclePath);
        static bool BFSForBipatite(const Graph& graph, unsigned int start, vector<int>& color, unordered_set<unsigned int>& A, unordered_set<unsigned int>& B);
        static void formatBipartiteSets(const unordered_set<unsigned int>& A, const unordered_set<unsigned int>& B, string& result);
        static string setToString(const unordered_set<unsigned int>& s);
        static unsigned int countOccurrences(const string& str, char target);
    };
}