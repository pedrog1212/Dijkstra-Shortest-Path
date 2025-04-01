//===============================================================================================================
// Name        : Graph.hpp
// Author      : Pedro Gonzalez Guzman
// Version     : 11/16/2024
// Description : Graph class declaration. It implements an undirected weighted Graph ADT and 
//               performing Dijkstra's Algorithm to find the shortest path between two vertices. 
//               This graph is implemented using an adjacency list.
//==============================================================================================================

#include <iostream>
#include <unordered_map>
//#include <vector>
//#include <string>
#include <algorithm>
#include <tuple>
#include <queue>
#include <limits>
#include <iomanip>
#include "GraphBase.hpp"

using namespace std;


/**
 * @brief Represents a vertex in the graph.
 */
class Vertex 
{
    
private:
    std::string vertexLabel;

public:
    // Constructor
    Vertex(const std::string& label) : vertexLabel(label) {}

    // Destructor
    ~Vertex() = default;

    // Getter for vertexLabel
    std::string getVertexLabel() const {
        return vertexLabel;
    }

    // Setter for vertexLabel
    void setVertexLabel(const std::string& label) {
        vertexLabel = label;
    }
    
};


/**
 * @brief Represents an edge in the weighted graph.
 */
class Edge 
{
    
private:
    std::string vertexLabel1;
    std::string vertexLabel2;
    unsigned long weight;

public:
    // Constructor
    Edge(const std::string& label1, const std::string& label2, unsigned long w)
        : vertexLabel1(label1), vertexLabel2(label2), weight(w) {}

    // Destructor
    ~Edge() = default;

    // Getter for vertexLabel1
    std::string getVertexLabel1() const {
        return vertexLabel1;
    }

    // Setter for vertexLabel1
    void setVertexLabel1(const std::string& label1) {
        vertexLabel1 = label1;
    }

    // Getter for vertexLabel2
    std::string getVertexLabel2() const {
        return vertexLabel2;
    }

    // Setter for vertexLabel2
    void setVertexLabel2(const std::string& label2) {
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
    std::tuple<std::string, std::string, unsigned long> toTuple() const {
        return std::make_tuple(vertexLabel1, vertexLabel2, weight);
    }
    
};

/**
 * @brief Represents an undirected weighted graph.
 */
 
class Graph : public GraphBase
{
    
private:

    std::vector<Vertex*> vertices;                      // List of vertices

    std::vector<Edge*> edges;                           // List of edges
    
    std::vector<std::vector<int>> adjacencyList;        // Adjacency list representation
    

    // helper function to find the vertex index using its label
    int getVertexIndex(const std::string& label) const;

public:

    // Constructor
    Graph( std::vector<std::string>& vertexLabels,
           std::vector<std::tuple<std::string, std::string, unsigned long>>& edgeTuples );

    // Destructor
    ~Graph();

    /**
     * @brief Adds a vertex to the graph.
     * @param label The label of the vertex to add.
     * @throws std::invalid_argument if a vertex with the same label already exists.
     */
    void addVertex( std::string label) override;

    /**
     * @brief Removes a vertex from the graph.
     * @param label The label of the vertex to remove.
     * @throws std::invalid_argument if the vertex does not exist.
     */
    void removeVertex( std::string label) override;
        
    /**
     * @brief Adds an edge between two vertices.
     * @param label1 The label of the first vertex.
     * @param label2 The label of the second vertex.
     * @param weight The weight of the edge.
     * @throws std::invalid_argument if the edge already exists or if vertices do not exist.
     */
    void addEdge( std::string label1,  std::string label2, unsigned long weight) override;

    /**
     * @brief Removes an edge between two vertices.
     * @param label1 The label of the first vertex.
     * @param label2 The label of the second vertex.
     * @throws std::invalid_argument if vertices do not exist or if the edge does not exist.
     */
    void removeEdge( std::string label1,  std::string label2) override;

    /**
     * @brief Calculates the shortest path between two vertices using Dijkstra's Algorithm.
     * @param startLabel The label of the starting vertex.
     * @param endLabel The label of the ending vertex.
     * @param path Reference to a vector that stores the shortest path (as vertex labels).
     * @return The total weight of the shortest path. Returns ULONG_MAX if no path exists.
     * @throws std::invalid_argument if either vertex does not exist.
     */
    unsigned long shortestPath( std::string startLabel, std::string endLabel, std::vector<std::string>& path) override;
    
    /**
     * @brief Prints the adjacency list for debugging purposes.
     */
    void printAdjacencyList() const;

    /**
     * @brief Prints the vertex neighbors list for debugging purposes.
     */
    void printVertexNeigList(const std::string& label) const;
  
};

