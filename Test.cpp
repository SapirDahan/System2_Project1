#include "doctest.h"
#include "Algorithms.hpp"
#include "Graph.hpp"


using namespace std;

TEST_CASE("Test isConnected")
{
    ariel::Graph g;

	// Strongly connected
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK((ariel::Algorithms::isConnected(g)));

	// Not conected
    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(!ariel::Algorithms::isConnected(g));

	// Connected but not strongly
    vector<vector<int>> graph3 = {
        {0, 1},
        {0, 0}};
    CHECK(!ariel::Algorithms::isConnected(g));
}

TEST_CASE("Test shortestPath")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);

    CHECK_THROWS(ariel::Algorithms::shortestPath(g, 0, 3));

    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "-1");

    vector<vector<int>> graph3 = {
        {0, -1, -2},
        {-1, 0, -5},
        {-2, -5, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "Negative cycle detected in the path.");

    vector<vector<int>> graph4 = {
        {0, 1, 2},
        {1, 0, -5},
        {2, -5, 0}};
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "Negative cycle detected in the path.");

    vector<vector<int>> graph5 = {
        {0, 2, 1},
        {0, 0, -5},
        {0, 0, 0}};

    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");

    // Negative cycle not in path
    vector<vector<int>> graph6 = {
        {0, 0, 0, 2, 1},
        {0, 0, 0, 0, -5},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0}};

    g.loadGraph(graph6);
    CHECK(ariel::Algorithms::shortestPath(g, 3, 4) == "3->4");
}

TEST_CASE("Test isContainsCycle"){
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isContainsCycle(g) != "0");
}
TEST_CASE("Test isBipartite")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2}, B={1}.");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isBipartite(g) == "0");

    vector<vector<int>> graph3 = {
        {0, 1, 0, 0, 0},
        {1, 0, 3, 0, 0},
        {0, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2, 4}, B={1, 3}.");
}

TEST_CASE("Test negative cycles"){
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2},
        {1, 0, -5},
        {2, -5, 0}};

    g.loadGraph(graph);
    CHECK(ariel::Algorithms::negativeCycle(g) == "The graph contains negative cycle");

    // Negative cycle not in path
    vector<vector<int>> graph2 = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 0, 2},
        {0, 0, -5, 0, 0}};

    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::negativeCycle(g) == "The graph contains negative cycle");

    vector<vector<int>> graph3 = {
        {0, 1, 0, 0, 0},
        {1, 0, 3, 0, 0},
        {0, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, -10, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::negativeCycle(g) == "The graph contains negative cycle");

 	vector<vector<int>> graph4 = {
        {0, 1, 0, 0, 0},
        {1, 0, 3, 0, 0},
        {0, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, -3, 0}};
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::negativeCycle(g) == "The graph NOT contains negative cycle");

	// Cycle sum 0
	vector<vector<int>> graph5 = {
        {0, 1, 0},
        {0, 0, 2},
        {-3, 0, 0}};

    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::negativeCycle(g) == "The graph NOT contains negative cycle");



}

TEST_CASE("Test invalid graph")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2, 0},
        {1, 0, 3, 0},
        {2, 3, 0, 4},
        {0, 0, 4, 0},
        {0, 0, 0, 5}};
    CHECK_THROWS(g.loadGraph(graph));

    vector<vector<int>> graph2 = {
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    CHECK_THROWS(g.loadGraph(graph2));

    vector<vector<int>> graph3 = {
        {0, 1, 2, 0},
        {1, 0, 3, 0},
        {2, 3, 0, 4, 5},
        {0, 0, 4, 0},
        {0, 0, 0, 5}};
    CHECK_THROWS(g.loadGraph(graph3));

    vector<vector<int>> graph4 = {
        {0, 1, 2, 0},
        {1, 0, 3, 0},
        {2, 3, 0, 4},
        {0, 0, 4},
        {0, 0, 0, 5}};
    CHECK_THROWS(g.loadGraph(graph4));

	vector<vector<int>> graph5 = {
        {0, 1, 2, 0, 1},
        {1, 0, 3, 0},
        {2, 3, 0, 4},
        {0, 0, 0, 4},
        {0, 0, 0, 5}};
    CHECK_THROWS(g.loadGraph(graph5));
}

TEST_CASE("Test print graph"){
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2, 0, 0},
        {1, 0, 3, 0, 0},
        {2, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph);

    CHECK(g.printGraph() == "Graph with 5 vertices and 10 edges.");

    vector<vector<int>> graph1 = {
        {0, 1},
        {0, 0}};
    g.loadGraph(graph1);

    CHECK(g.printGraph() == "Graph with 2 vertices and 1 edges.");

    vector<vector<int>> graph2 = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},};
    g.loadGraph(graph2);

    CHECK(g.printGraph() == "Graph with 5 vertices and 0 edges.");
}

TEST_CASE("Test graph size"){
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2, 0, 0},
        {1, 0, 3, 0, 0},
        {2, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph);

    CHECK(g.size() == 5);
}