#include "Graph.hpp"

int main() {
    // Define vertices and edges
    std::vector<std::string> vertices1 = {"1", "2", "3", "4", "5", "6"};
    std::vector<std::tuple<std::string, std::string, unsigned long>> edges1 = {
        {"1", "2", 7}, {"1", "3", 9}, {"1", "6", 14},
        {"2", "3", 10}, {"2", "4", 15}, {"3", "4", 11},
        {"3", "6", 2}, {"4", "5", 6}, {"5", "6", 9}};

    // Create graph
    Graph g(vertices1, edges1);

    // Print adjacency list
    g.printAdjacencyList();
    std::cout << std::endl;
    
    // Add a new vertex and print updated graph
    g.addVertex("7");
    g.printAdjacencyList();
    std::cout << std::endl;
    
    // Remove a vertex and print updated graph
    g.removeVertex("3");
    g.printAdjacencyList();
    std::cout << std::endl;
    
    std::cout << std::endl;

    return 0;
}
