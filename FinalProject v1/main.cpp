// Example 1: Adding Vertices and Edges

#include "Graph.h"

int main() {
    Graph graph;

    graph.addVertex("A");
    graph.addVertex("B");
    graph.addVertex("C");

    graph.addEdge("A", "B", 5);
    graph.addEdge("B", "C", 10);

    std::cout << "Graph setup completed." << std::endl;

    return 0;
}

/*
// Example 2: Removing a Vertex

#include "Graph.h"

int main() {
    Graph graph;

    graph.addVertex("A");
    graph.addVertex("B");
    graph.addEdge("A", "B", 5);

    std::cout << "Removing vertex B..." << std::endl;
    graph.removeVertex("B");

    return 0;
}
*/

/*
// Example 3: Finding the Shortest Path
#include "Graph.h"

int main() {
    Graph graph;

    graph.addVertex("A");
    graph.addVertex("B");
    graph.addVertex("C");
    graph.addVertex("D");

    graph.addEdge("A", "B", 1);
    graph.addEdge("B", "C", 2);
    graph.addEdge("C", "D", 1);
    graph.addEdge("A", "D", 10);

    std::vector<std::string> path;
    unsigned long distance = graph.shortestPath("A", "D", path);

    std::cout << "Shortest path from A to D: ";
    for (const auto& vertex : path) {
        std::cout << vertex << " ";
    }
    std::cout << "\nTotal distance: " << distance << std::endl;

    return 0;
}
*/

/*
// Example 4: Removing an Edge

#include "Graph.h"

int main() {
    Graph graph;

    graph.addVertex("A");
    graph.addVertex("B");
    graph.addVertex("C");

    graph.addEdge("A", "B", 3);
    graph.addEdge("B", "C", 4);

    std::cout << "Removing edge between A and B..." << std::endl;
    graph.removeEdge("A", "B");

    // Attempting shortest path
    std::vector<std::string> path;
    unsigned long distance = graph.shortestPath("A", "C", path);

    if (distance == std::numeric_limits<unsigned long>::max()) {
        std::cout << "No path exists between A and C after removing the edge." << std::endl;
    } else {
        std::cout << "Path exists with distance: " << distance << std::endl;
    }

    return 0;
}
*/

/*

// Example 5: Complex Graph with Multiple Paths

#include "Graph.h"

int main() {
    Graph graph;

    graph.addVertex("A");
    graph.addVertex("B");
    graph.addVertex("C");
    graph.addVertex("D");
    graph.addVertex("E");

    graph.addEdge("A", "B", 4);
    graph.addEdge("A", "C", 2);
    graph.addEdge("B", "C", 1);
    graph.addEdge("B", "D", 5);
    graph.addEdge("C", "D", 8);
    graph.addEdge("C", "E", 10);
    graph.addEdge("D", "E", 2);

    std::vector<std::string> path;
    unsigned long distance = graph.shortestPath("A", "E", path);

    std::cout << "Shortest path from A to E: ";
    for (const auto& vertex : path) {
        std::cout << vertex << " ";
    }
    std::cout << "\nTotal distance: " << distance << std::endl;

    return 0;
}
*/


// Expected Output for Example 5:

//Shortest path from A to E: A C B D E 
//Total distance: 9

/*
// Example 6: Shortest Path with Multiple Routes and Cycles

#include "Graph.h"

int main() {
    Graph graph;

    // Adding vertices
    graph.addVertex("A");
    graph.addVertex("B");
    graph.addVertex("C");
    graph.addVertex("D");
    graph.addVertex("E");
    graph.addVertex("F");

    // Adding edges with weights
    graph.addEdge("A", "B", 1);
    graph.addEdge("A", "C", 5);
    graph.addEdge("B", "C", 2);
    graph.addEdge("B", "D", 2);
    graph.addEdge("C", "E", 3);
    graph.addEdge("D", "E", 1);
    graph.addEdge("D", "F", 4);
    graph.addEdge("E", "F", 1);

    // Finding the shortest path
    std::vector<std::string> path;
    unsigned long distance = graph.shortestPath("A", "F", path);

    // Display the result
    std::cout << "Shortest path from A to F: ";
    for (const auto& vertex : path) {
        std::cout << vertex << " ";
    }
    std::cout << "\nTotal distance: " << distance << std::endl;

    return 0;
}
*/

/*
// Graph Visualization:

       1
    A ----- B
     \     / \
      5   2   2
       \ /     \
        C       D
         \     / \
          3   1   4
           \ /     \
            E ----- F
              1
*/              
// Expected Output:

//Shortest path from A to F: A B D E F 
//Total distance: 5


