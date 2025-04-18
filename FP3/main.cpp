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
    // ..........     Creating the Graph #1   ..........
    //
     // Define vertices and edges of the graph 1
    vector<string> vertices1 = {"A", "B", "C", "D", "E", "F"};
    
    vector<tuple<string, string, unsigned long>> edges1 = {
        {"A", "B", 7}, {"A", "C", 9}, {"A", "F", 14},
        {"B", "C", 10}, {"B", "D", 15}, {"C", "D", 11},
        {"C", "F", 2}, {"D", "E", 6}, {"E", "F", 9}};

    // Graph g =  Graph (vertices1, edges1);  // destroyer is call automatically when main() ends
    // Constructing the Graph
    Graph* g = new Graph (vertices1, edges1);

    // Print adjacency list
    cout << "............... #1 Adj. List of Vertices " << endl;
    g->printAdjacencyList();
    
    // Add a new vertex and print updated graph
    cout << "............... adding new vertex G with edges B, D, F" << endl;
    g->addVertex("G");
    g->addEdge("G", "B", 3);
    g->addEdge("G", "D", 2);
    g->addEdge("G", "F", 5);
//    g->addEdge("G", "F", 5);   // logging error
//    g->addVertex("G");         // logging error
    
    g->printAdjacencyList();
    
    // Remove a vertex and print updated graph
//    cout << "................ Remove vertex C " << endl;
//    g->removeVertex("C");


    // Remove an Edge and print updated graph
    cout << "................ Remove edge A - B " << endl;
    g->removeEdge("A", "B");
//    g->removeEdge("A", "B");   // logging error
    g->printAdjacencyList();

    // Remove a Vertex and print updated graph
    cout << "................ Remove Vertex C " << endl;
    g->removeVertex("C");
//    g->removeVertex("C");   // logging error
    g->printAdjacencyList();

    cout << "................ Remove Vertex A, B" << endl;
    g->removeVertex("A");
    g->removeVertex("B");
    g->printAdjacencyList();

    cout << "................ Add Vertex A, B" << endl;
    g->addVertex("A");
    g->addVertex("B");
    g->addVertex("C");
    g->printAdjacencyList();

    cout << "................ Restore Original Vertices and Edges  " << endl;
    g->addEdge("A", "B", 7); //           {"A", "B", 7},  
    g->addEdge("A", "C", 9); //           {"A", "C", 9},  
    g->addEdge("A", "F", 14); //          {"A", "F", 14},
    g->addEdge("B", "C", 10); //          {"B", "C", 10},   

    g->addEdge("B", "D", 15); //          {"B", "D", 15},
    g->addEdge("C", "D", 11); //          {"C", "D", 11},
    g->addEdge("C", "F", 2); //           {"C", "F", 2}
    g->addEdge("G", "B", 3);
    g->printAdjacencyList();
    
    cout << "................ Remove Vertex B, G" << endl;
    g->removeVertex("B");
    g->removeVertex("G");
    g->printAdjacencyList();

    cout << "................ Restore Original Vertices and Edges  " << endl;
    g->addVertex("G");
    g->addVertex("B");
    g->addEdge("A", "B", 7); //           {"A", "B", 7},  
    g->addEdge("B", "C", 10); //          {"B", "C", 10},   
    g->addEdge("B", "D", 15); //          {"B", "D", 15},
    g->addEdge("G", "B", 3);
    g->addEdge("G", "D", 2);
    g->addEdge("G", "F", 5);
    g->printAdjacencyList();
    
    // Find shortest path
    cout << endl;
    cout << "............................................." << endl;
    cout << "Finding Shortest path from A to D: ";
    vector<string> path;
    unsigned long distance = g->shortestPath("A", "D", path);

    // Print results
    cout << "Shortest path from A to D: ";
    for (const auto& vertex : path) {
        cout << vertex << " ";
    }
    cout << "\nTotal distance: " << distance << endl;
    
    cout << endl;
    
    // Find shortest path
    cout << "............................................." << endl;
    cout << "Finding Shortest path from A to D: ";
    distance = g->shortestPath("C", "D", path);

    // Print results
    cout << "Shortest path from C to D: ";
    for (const auto& vertex : path) {
        cout << vertex << " ";
    }
    cout << "\nTotal distance: " << distance << endl;
    
    cout << endl;

    cout << ".........   Destroying the Graph #1 " << endl;
    
    delete g;
    
    vertices1.clear();
    edges1.clear();
    
    // ..........     Creating the Graph #2   ..........
    //
    // Define vertices and edges of the graph 2
    vertices1 = {"A", "B", "C", "D", "E"};
    
    edges1 = 
    {
        {"A", "B", 2}, {"A", "C", 3}, {"A", "D", 8},
        {"B", "C", 1}, {"B", "D", 3},
        {"C", "D", 5}, {"C", "E", 3},
        {"D", "E", 4}
        
    };

    cout << endl << endl;
    // Constructing the Graph
    //
    // Graph g =  Graph (vertices1, edges1);  
    Graph* g2 =  new Graph (vertices1, edges1);

    // Print adjacency list
    cout << "............... #2 Adj. List of Vertices " << endl;
    g2->printAdjacencyList();
    // Find shortest path
    cout << "............................................." << endl;
    cout << "Finding Shortest path from A to D: ";
    distance = g2->shortestPath("A", "D", path);

    // Print results
    cout << "Shortest path from A to D: ";
    for (const auto& vertex : path) {
        cout << vertex << " ";
    }
    cout << "\nTotal distance: " << distance << endl;
    
    cout << endl;

    cout << ".........   Destroying the Graph #2 " << endl;
    delete g2;
    
    vertices1.clear();
    edges1.clear();

    return 0;
}
