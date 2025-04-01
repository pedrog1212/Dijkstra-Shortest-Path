//===============================================================================================================
// Name        : Graph.hpp
// Author      : Pedro Gonzalez Guzman
// Version     : 11/16/2024
// Description : Graph class declaration. It implements an undirected weighted Graph ADT and 
//               performing Dijkstra's Algorithm to find the shortest path between two vertices. 
//               This graph is implemented using an adjacency list.
//==============================================================================================================

#include <iostream>
#include <algorithm>
#include <tuple>
#include <queue>
#include <limits>
#include "GraphBase.hpp"

using namespace std;


/**
 * @brief Represents a vertex in the graph.
 */
class Vertex 
{
    
private:
    string vertexLabel;

public:
    // Constructor
    Vertex(const string& label) : vertexLabel(label) {}

    // Destructor
    ~Vertex() = default;

    // Getter for vertexLabel
    string getVertexLabel() const {
        return vertexLabel;
    }

    // Setter for vertexLabel
    void setVertexLabel(const string& label) {
        vertexLabel = label;
    }
    
};


/**
 * @brief Represents an edge in the weighted graph.
 */
class Edge 
{
    
private:
    string vertexLabel1;
    string vertexLabel2;
    unsigned long weight;

public:
    // Constructor
    Edge(const string& label1, const string& label2, unsigned long w)
        : vertexLabel1(label1), vertexLabel2(label2), weight(w) {}

    // Destructor
    ~Edge() = default;

    // Getter for vertexLabel1
    string getVertexLabel1() const {
        return vertexLabel1;
    }

    // Setter for vertexLabel1
    void setVertexLabel1(const string& label1) {
        vertexLabel1 = label1;
    }

    // Getter for vertexLabel2
    string getVertexLabel2() const {
        return vertexLabel2;
    }

    // Setter for vertexLabel2
    void setVertexLabel2(const string& label2) {
        vertexLabel2 = label2;
    }

    // Getter for weight
    unsigned long getWeight() const {
        return weight;
    }

    // Setter for weight
    void setWeight(unsigned long w) {
        weight = w;
    }

    // Create a tuple representing the edge
    tuple<string, string, unsigned long> toTuple() const {
        return make_tuple(vertexLabel1, vertexLabel2, weight);
    }
    
};

/**
 * @brief Represents an undirected weighted graph.
 */
 
class Graph : public GraphBase
{
    
private:

    vector<Vertex*> vertices;                       // List of vertices

    vector<Edge*> edges;                            // List of edges
    
    vector<vector<int>> adjacencyList;              // Adjacency list representation
    

    // Helper function to find the vertex index using its label
    int getVertexIndex(const string& label) const;

public:

    // Constructor
    Graph( vector<string>& vertexLabels,
           vector<tuple<string, string, unsigned long>>& edgeTuples );

    // Destructor
    ~Graph();

    /**
     * @brief Adds a vertex to the graph.
     * @param label The label of the vertex to add.
     * @throws invalid_argument if a vertex with the same label already exists.
     */
    void addVertex( string label) override;

    /**
     * @brief Removes a vertex from the graph.
     * @param label The label of the vertex to remove.
     * @throws invalid_argument if the vertex does not exist.
     */
    void removeVertex( string label) override;
        
    /**
     * @brief Adds an edge between two vertices.
     * @param label1 The label of the first vertex.
     * @param label2 The label of the second vertex.
     * @param weight The weight of the edge.
     * @throws invalid_argument if the edge already exists or if vertices do not exist.
     */
    void addEdge( string label1,  string label2, unsigned long weight) override;

    /**
     * @brief Removes an edge between two vertices.
     * @param label1 The label of the first vertex.
     * @param label2 The label of the second vertex.
     * @throws invalid_argument if vertices do not exist or if the edge does not exist.
     */
    void removeEdge( string label1,  string label2) override;

    /**
     * @brief Calculates the shortest path between two vertices using Dijkstra's Algorithm.
     * @param startLabel The label of the starting vertex.
     * @param endLabel The label of the ending vertex.
     * @param path Reference to a vector that stores the shortest path (as vertex labels).
     * @return The total weight of the shortest path. Returns ULONG_MAX if no path exists.
     * @throws invalid_argument if either vertex does not exist.
     */
    unsigned long shortestPath( string startLabel, string endLabel, vector<string>& path) override;
    
    /**
     * @brief Prints the adjacency list for debugging purposes.
     */
    void printAdjacencyList() const;

};

