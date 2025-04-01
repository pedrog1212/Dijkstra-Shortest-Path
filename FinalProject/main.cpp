#include "Graph.hpp"

int main() {
    Graph graph;

    graph.addVertex("A");
    graph.addVertex("B");
    graph.addVertex("C");
    graph.addVertex("D");

    graph.addEdge("A", "B", 1);
    graph.addEdge("B", "C", 2);
    graph.addEdge("A", "C", 2);
    graph.addEdge("C", "D", 1);

    std::vector<std::string> path;
    unsigned long distance = graph.shortestPath("A", "D", path);

    std::cout << "Shortest path from A to D: ";
    for (const auto& vertex : path) {
        std::cout << vertex << " ";
    }
    std::cout << "\nDistance: " << distance << std::endl;

    return 0;
}
