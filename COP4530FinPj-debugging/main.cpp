//===============================================================================================================
// Name        : main.hpp
// Author      : Pedro Gonzalez Guzman
// Version     : 11/16/2024
// Description : Graph class client-tester. It creates an instance of a Graph class (undirected weighted Graph )
//               It adds and removes vertices and edges to the Graph. 
//               It finds several "shortest path between two vertices" using Dijkstra's Algorithm function. 
//==============================================================================================================
#include "Graph.hpp"

int main() {
    // Define vertices and edges
    std::vector<std::string> vertices1 = {"A", "B", "C", "D", "E", "F"};
    std::vector<std::tuple<std::string, std::string, unsigned long>> edges1 = {
        {"A", "B", 7}, {"A", "C", 9}, {"A", "F", 14},
        {"B", "C", 10}, {"B", "D", 15}, {"C", "D", 11},
        {"C", "F", 2}, {"D", "E", 6}, {"E", "F", 9}};

    // Creating the Graph
    Graph g = Graph (vertices1, edges1);

    // Print adjacency list
    std::cout << "............... Original Adj. List of Vertices " << std::endl;
    g.printAdjacencyList();
    
    // Add a new vertex and print updated graph
    std::cout << "............... adding new vertex G with edges B, D, F" << std::endl;
    g.addVertex("G");
    g.addEdge("G", "B", 3);
    g.addEdge("G", "D", 2);
    g.addEdge("G", "F", 5);
//    g.addEdge("G", "F", 5);   // logging error
//    g.addVertex("G");         // logging error
    
    g.printAdjacencyList();
    
    // Remove a vertex and print updated graph
//    std::cout << "................ Remove vertex C " << std::endl;
//    g.removeVertex("C");


    // Remove an Edge and print updated graph
    std::cout << "................ Remove edge A - B " << std::endl;
    g.removeEdge("A", "B");
//    g.removeEdge("A", "B");   // logging error
    g.printAdjacencyList();

    // Remove a Vertex and print updated graph
    std::cout << "................ Remove Vertex C " << std::endl;
    g.removeVertex("C");
//    g.removeVertex("C");   // logging error
    g.printAdjacencyList();

     std::cout << "................ Remove Vertex A, B" << std::endl;
    g.removeVertex("A");
    g.removeVertex("B");
    g.printAdjacencyList();

     std::cout << "................ Add Vertex A, B" << std::endl;
    g.addVertex("A");
    g.addVertex("B");
    g.addVertex("C");
    g.printAdjacencyList();

     std::cout << "................ Restore Original Vertices and Edges  " << std::endl;
    g.addEdge("A", "B", 7); //           {"A", "B", 7},  
    g.addEdge("A", "C", 9); //           {"A", "C", 9},  
    g.addEdge("A", "F", 14); //          {"A", "F", 14},
    g.addEdge("B", "C", 10); //          {"B", "C", 10},   

    g.addEdge("B", "D", 15); //          {"B", "D", 15},
    g.addEdge("C", "D", 11); //          {"C", "D", 11},
    g.addEdge("C", "F", 2); //           {"C", "F", 2}
    g.addEdge("G", "B", 3);
    g.printAdjacencyList();
    
     std::cout << "................ Remove Vertex B, G" << std::endl;
    g.removeVertex("B");
    g.removeVertex("G");
    g.printAdjacencyList();

     std::cout << "................ Restore Original Vertices and Edges  " << std::endl;
    g.addVertex("G");
    g.addVertex("B");
    g.addEdge("A", "B", 7); //           {"A", "B", 7},  
    g.addEdge("B", "C", 10); //          {"B", "C", 10},   
    g.addEdge("B", "D", 15); //          {"B", "D", 15},
    g.addEdge("G", "B", 3);
    g.addEdge("G", "D", 2);
    g.addEdge("G", "F", 5);
    g.printAdjacencyList();
    
    // Find shortest path
    std::cout << std::endl;
    std::cout << "............................................." << std::endl;
    std::cout << "Finding Shortest path from A to D: ";
    std::vector<std::string> path;
    unsigned long distance = g.shortestPath("A", "D", path);

    // Print results
    std::cout << "Shortest path from A to D: ";
    for (const auto& vertex : path) {
        std::cout << vertex << " ";
    }
    std::cout << "\nTotal distance: " << distance << std::endl;
    
    std::cout << std::endl;
    
    // Find shortest path
    std::cout << "............................................." << std::endl;
    std::cout << "Finding Shortest path from A to D: ";
    distance = g.shortestPath("C", "D", path);

    // Print results
    std::cout << "Shortest path from C to D: ";
    for (const auto& vertex : path) {
        std::cout << vertex << " ";
    }
    std::cout << "\nTotal distance: " << distance << std::endl;
    
    std::cout << std::endl;

    std::cout << "................ Destroying the Graph  " << std::endl;

    return 0;
}
