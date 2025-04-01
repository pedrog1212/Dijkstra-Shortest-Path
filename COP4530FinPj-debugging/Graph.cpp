//===============================================================================================================
// Name        : Graph.cpp
// Author      : Pedro Gonzalez Guzman
// Version     : 11/16/2024
// Description : Graph class implementation. It implements an undirected weighted Graph ADT and 
//               performing Dijkstra's Algorithm to find the shortest path between two vertices. 
//               This graph is implemented using an adjacency list.
//==============================================================================================================

#include "Graph.hpp"

    // Constructor
    Graph::Graph( std::vector<std::string>& vertexLabels,
           std::vector<std::tuple<std::string, std::string, unsigned long>>& edgeTuples )
    {
        for ( auto& label : vertexLabels) {   // add vertices to the adjacent list
            addVertex(label);
            
        }
        
        for ( auto& [label1, label2, weight] : edgeTuples) { // add edges to the adjacent list
            addEdge(label1, label2, weight);
        }
    }

    // Destructor
    Graph::~Graph() {
        for (auto vertex : vertices) {   // remove all vertex instances
            delete vertex;
        }
        
        for (auto edge : edges) {       // remove all edges instances
           delete edge;    
        }
        
        // clear the containers: vertices, adjacencyList, edges
        vertices.clear();
        
        adjacencyList.clear();
        
        edges.clear();

    }

    // helper function to find the vertex index using its label
    int Graph::getVertexIndex(const std::string& label) const {
        
        auto it = std::find_if (vertices.begin(), vertices.end(), [label](Vertex* vertex){
            return vertex->getVertexLabel() == label;     // Compare label in vertices list
        });
        
        if (it != vertices.end()) {
            
            return std::distance(vertices.begin(), it);   // Calculate the index of vertices array
        }
        return -1;                                        // Return -1 if vertex not found
    }

    /**
     * @brief Adds a vertex to the graph.
     * @param label The label of the vertex to add.
     * @throws std::invalid_argument if a vertex with the same label already exists.
     */
    void Graph::addVertex( std::string label) {
        
        // Find the vertex by label
        auto it = std::find_if(vertices.begin(), vertices.end(), [label](Vertex* vertex) {
            return vertex->getVertexLabel() == label;   // find target label value inside the vertices list
        });
        
        if (it != vertices.end()) throw invalid_argument("Vertex already exists");

        vertices.push_back(new Vertex(label));          // Add the new vertex instance to vertices list
        
        adjacencyList.emplace_back();                   // Add an empty vector<int> for new vertex neighbors
    }

/**
     * @brief Removes a vertex from the graph.
     * @param label The label of the vertex to remove.
     * @throws std::invalid_argument if the vertex does not exist.
     */
    void Graph::removeVertex( std::string label) {
        
        int index = getVertexIndex(label);
        if (index == -1 ) throw invalid_argument("Missing vertex");

        // Remove edges pointing to this vertex in adjacency list
        for (auto& neighbors : adjacencyList) {
            
            // use erase-remove combo- remove() cannot resize the array! erase() remove it physically
            neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), index), neighbors.end());  
        }
        
        // Remove edges related to the vertex from the edges list
        edges.erase(std::remove_if(edges.begin(), edges.end(),
                               [&](Edge* edge) {
                                   return edge->getVertexLabel1() == label || edge->getVertexLabel2() == label;
                               }),
                edges.end());


        // Remove the vertex
        delete vertices[index];                             // deallocate the memory of this Vertex instance (created with new)
        
        vertices.erase(vertices.begin() + index);           // remove the elemnt w/index from the array of Vertices
        
        adjacencyList.erase(adjacencyList.begin() + index); // remove vetex's adjacency list

        // Adjust adjacency list indexes. Substracting 1, from the indexes larger than the erased one
        for (auto& neighbors : adjacencyList) {
            for (auto& vertexIdx : neighbors) {
                if (vertexIdx > index) {
                    vertexIdx--;
                }
            }
        }
    }

    /**
     * @brief Adds an edge between two vertices.
     * @param label1 The label of the first vertex.
     * @param label2 The label of the second vertex.
     * @param weight The weight of the edge.
     * @throws std::invalid_argument if the edge already exists or if vertices do not exist.
     */
    void Graph::addEdge( std::string label1, std::string label2, unsigned long weight) {
        
        int index1 = getVertexIndex(label1);
        
        int index2 = getVertexIndex(label2);
        
        if (index1 == -1 || index2 == -1 || index1 == index2) throw invalid_argument("Invalid vertex");

        //std::cout << " inside add Edges: " << endl;   // logging
        //std::cout << "label 1 " << vertices[index1]->getVertexLabel() << " is vertices index " << index1 << endl; // logging
        //std::cout << "label 2 " << vertices[index2]->getVertexLabel() << " is vertices index " << index2 << endl; // logging
        
        // Check if the edge already exists
        for (const auto& neighbor : adjacencyList[index1]) {
            
            //std::cout << "in AL label: "<< vertices[index1]->getVertexLabel() << ", index: " << index1 << ". Existing neighbor w/index: " << neighbor << endl;  // logging
            
            if (neighbor == index2) throw std::invalid_argument("Edge already exists");
            
        }

        // Add the edge to the adjacency list and edge list
        adjacencyList[index1].push_back(index2);
        
        adjacencyList[index2].push_back(index1);
        
        edges.push_back(new Edge(label1, label2, weight));
    }

/**
     * @brief Removes an edge between two vertices.
     * @param label1 The label of the first vertex.
     * @param label2 The label of the second vertex.
     * @throws std::invalid_argument if vertices do not exist or if the edge does not exist.
     */
    void Graph::removeEdge( std::string label1,  std::string label2) {
        
        // Validate that both vertices exist
        int index1 = getVertexIndex(label1);
        int index2 = getVertexIndex(label2);
        if (index1 == -1 || index2 == -1) throw invalid_argument("Missing vertex");

        // Remove the edge from adjacency list of index1 to index2
        auto& neighbors1 = adjacencyList[index1];
        auto it2 = std::find(neighbors1.begin(), neighbors1.end(), index2);  // in vertex label1, find the neighbor with label2 
        
        if (it2 == neighbors1.end()) {
            throw std::invalid_argument("Edge does not exist between the specified vertices");
        }

        neighbors1.erase(it2);  // erase in vertexr label1, the neighbor with label2

        // Remove the edge from adjacency list of index2 to index1
        auto& neighbors2 = adjacencyList[index2];
        auto it1 = std::find(neighbors2.begin(), neighbors2.end(), index1); // in vertexr label2, find the neighbor with label1 
        neighbors2.erase(it1);  // erase in vertex label2, the neighbor with label1

        // Remove the edge from the edge list
        edges.erase(std::remove_if(edges.begin(), edges.end(),
                                   [&](Edge* edge) {
                                       return (edge->getVertexLabel1() == label1 && edge->getVertexLabel2() == label2) ||
                                              (edge->getVertexLabel1() == label2 && edge->getVertexLabel2() == label1);
                                   }),
                    edges.end());
        
    }


    /**
     * @brief Calculates the shortest path between two vertices using Dijkstra's Algorithm.
     * @param startLabel The label of the starting vertex.
     * @param endLabel The label of the ending vertex.
     * @param path Reference to a vector that stores the shortest path (as vertex labels).
     * @return The total weight of the shortest path. Returns ULONG_MAX if no path exists.
     * @throws std::invalid_argument if either vertex does not exist.
     */
    unsigned long Graph::shortestPath( std::string startLabel, std::string endLabel, std::vector<std::string>& path) {
        
        // Declarations ////////////////
        int start = getVertexIndex(startLabel); // find the label index
        int end = getVertexIndex(endLabel);
    
        if (start == -1 || end == -1) throw invalid_argument("A Missing vertex");

        // Distances vector: stores the shortest distance from the start vertex to each vertex
        std::vector<unsigned long> distances (vertices.size(), std::numeric_limits<unsigned long>::max()); // vector initialized with ULONG_MAX to compare distances
        
        // Previous vector: stores the previous vertex indexes in the shortest path
        std::vector<int> previous (vertices.size(), -1);
    
        // Priority queue (PQ) for Dijkstra's algorithm. lambda fuction as comparator for PQ
        auto compare = [&](int a, int b) { return distances[a] > distances[b]; };   // PQ comparator to implement a Min-Heap
        
        std::priority_queue<int, std::vector<int>, decltype(compare)> pq (compare);  // declare PQ as a Min-Heap with comparator- lambda compare()

        // Functionality /////////////
        // Initialize distances and add start vertex to the priority queue
        distances[start] = 0;
        pq.push(start);             // push start vertex  
    
        // Dijkstra's algorithm uses BFS traversal with a priority-queue (PQ) to keep the minimal distance on the root
        
        while (!pq.empty()) {
            int current = pq.top();
            pq.pop();               // pop vertex with shorted distance to discover its neighbors
            
            std::cout << "" << endl;     //logging
            std::cout << "Current(popped) start vertex: " << vertices[current]->getVertexLabel() << endl;     //logging
            std::cout << "" << endl;     //logging

            // If we reached the destination vertex, exit the loop
            if (current == end) break;
    
            // Iterate through neighbors of the current vertex
            for (const auto& neighborIndex : adjacencyList[current]) {  // find the current vertex neighbors in the adjacencyList
                
                std::cout << "-----" << endl;     //logging
                std::cout << "  Neighbor vertex (neighborIndex): " << vertices[neighborIndex]->getVertexLabel() << endl;     //logging
                
                unsigned long edgeWeight = 0;
                
                for (const auto& edge : edges) {        // find the edge weight between current vertex its neighbor in edge list
                    if ((edge->getVertexLabel1() == vertices[current]->getVertexLabel() && edge->getVertexLabel2() == vertices[neighborIndex]->getVertexLabel()) 
                    ||  (edge->getVertexLabel1() == vertices[neighborIndex]->getVertexLabel() && edge->getVertexLabel2() == vertices[current]->getVertexLabel()))
                    {
                        std::cout << "    Found an Edge: " << edge->getVertexLabel1() << ", " << edge->getVertexLabel2() << " with Weight: " << edgeWeight <<endl ;     //logging

                        edgeWeight = edge->getWeight();
                        
                        std::cout <<  endl ;    //logging
                        
                        break;                                                      // when an edge is found break
                    }
                }
    
                unsigned long newDist = distances[current] + edgeWeight;            // increment the distance through this neighbor path
                
                std::cout << "    Min distance on this path: " << newDist << endl;   //logging
                
                std::cout << "    distance[neighborIndex] = Min : " << distances[neighborIndex] << endl;     //logging
    
                if (edgeWeight == 0) continue;                              // if no valid edge exists

                // If a shorter path is found
                if (newDist < distances[neighborIndex]) {
                    
                    distances[neighborIndex] = newDist;                             // found a new min distance to the neighbor
                    std::cout << "    distance[neighborIndex] updated : " << distances[neighborIndex] << endl;     //logging
                    
                    previous[neighborIndex] = current;                              // keep previous neighbor of the path
                    std::cout << "    Previous Vertex in Sort Path : " << vertices[current]->getVertexLabel() << endl;     //logging
                    
                    pq.push(neighborIndex);                                        // push neighbor with the min dist in the PQ 

                    std::cout << "-----" << endl;                                   //logging
                    std::cout << "     Push in PQ vertex: " << pq.top() << " label: " << vertices[neighborIndex]->getVertexLabel() <<endl;     //logging
                    
                }
            }
        }
    
        // If the destination vertex is unreachable
        if (distances[end] == std::numeric_limits<unsigned long>::max()) {
            return std::numeric_limits<unsigned long>::max();                       // return max possible unsigned long - ULONG_MAX
        }
    
        // Construct the shortest path
        path.clear(); // claen it
        
        for (int at = end; at != -1; at = previous[at]) {
            path.push_back(vertices[at]->getVertexLabel());
        }
        
        std::reverse(path.begin(), path.end());
    
        return distances[end];                  // return the min distance
        
    }
    
    
    /**
     * @brief Prints the adjacency list for debugging purposes.
     */
    void Graph::printAdjacencyList() const {
        
        for (size_t i = 0; i < adjacencyList.size(); ++i)
        {
            std::cout << vertices[i]->getVertexLabel() << ": ";
            
            for (const auto& neighbor : adjacencyList[i])
            {
                std::cout << vertices[neighbor]->getVertexLabel() << " ";
            }
            std::cout << std::endl;
        }
    }
    
    /**
     * @brief Prints the vertex neighbors list for debugging purposes.
     */
    void Graph::printVertexNeigList(const std::string& label) const {
    
        int index = getVertexIndex(label);
    
        // Remove edges pointing to this vertex
        for (size_t j = 0; j < adjacencyList[index].size(); j++) 
        
            std::cout << vertices[j]->getVertexLabel() << " ";
        
        //for (auto& n : neighbors) std::cout << n->getVertexLabel() << ": ";
        
    }

