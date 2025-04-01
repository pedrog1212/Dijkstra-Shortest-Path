
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
    Graph::Graph( vector<string>& vertexLabels,
           vector<tuple<string, string, unsigned long>>& edgeTuples )
    {
        for ( auto& label : vertexLabels) {   // add vertices to the adjacent list
            addVertex(label);
            
        }
        
        for ( auto& [label1, label2, weight] : edgeTuples) { // add edges to the adjacent list (comp. C++ 17)
            addEdge(label1, label2, weight);
        }
    }

    // Destructor
    Graph::~Graph() {
        for (auto vertex : vertices) {   // remove all Vertex instances
            delete vertex;
        }
        
        for (auto edge : edges) {       // remove all Edge instances
           delete edge;    
        }
        
        // clear the containers: vertices, adjacencyList, edges
        vertices.clear();
        
        adjacencyList.clear();
        
        edges.clear();

    }

    // helper function to find the vertex index using its label
    int Graph::getVertexIndex(const string& label) const {
        
        for ( int i = 0; i < vertices.size(); i++) {    // traverse the vector vertices
            
            if (vertices[i]->getVertexLabel() == label)  // if label found, return the index
            return i;
            
        }
        return -1;                                        // if vertex not found Return -1 
    }

    /**
     * @brief Adds a vertex to the graph.
     * 
     * @param label The label of the vertex to add.
     * @throws invalid_argument if a vertex with the same label already exists.
     */
    void Graph::addVertex( string label) {
        
         // find if new vertex label already exists in vertices list
        int index = getVertexIndex(label);
        
        if (index != -1 ) throw invalid_argument("Vertex already exists");  // terminate program with error

        vertices.push_back(new Vertex(label));          // Add the new Vertex class instance to vertices list
        
        adjacencyList.emplace_back();                   // Add an new row in Adj List vector<int> for vertex neighbors
    }

/**
     * @brief Removes a vertex from the graph.
     * 
     * @param label The label of the vertex to remove. 
     * @throws invalid_argument if the vertex does not exist.
     */
    void Graph::removeVertex( string label) {
        
         // find if target vertex label does not exists in vertices list
        int index = getVertexIndex(label);
        
        if (index == -1 ) throw invalid_argument("Missing vertex");
        
        for (auto& vertex : vertices)
            try {
                
                removeEdge( label, vertex->getVertexLabel());
                
            } catch (const invalid_argument& e) {
                continue;   // Code to handle the exception
            }
        
        // Remove the vertex
        delete vertices[index];                             // deallocate the memory of this Vertex instance (created with new)
        
        vertices.erase( vertices.begin() + index );           // remove the elemnt w/index from the array of Vertices
        
        adjacencyList.erase( adjacencyList.begin() + index ); // remove vetex's adjacency list

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
     * 
     * @param label1 The label of the first vertex.
     * @param label2 The label of the second vertex.
     * @param weight The weight of the edge.
     * 
     * @throws invalid_argument if the edge already exists or if vertices do not exist.
     */
    void Graph::addEdge( string label1, string label2, unsigned long weight) {
        
        int index1 = getVertexIndex(label1);
        
        int index2 = getVertexIndex(label2);
        
        if (index1 == -1 || index2 == -1 || index1 == index2) throw invalid_argument("Missing vertex");

        // Check if the edge already exists
        for ( auto& edge : edges) {        // find the edge and weight
        
            if ((edge->getVertexLabel1() == vertices[index1]->getVertexLabel() && edge->getVertexLabel2() == vertices[index2]->getVertexLabel()) 
            ||  (edge->getVertexLabel1() == vertices[index2]->getVertexLabel() && edge->getVertexLabel2() == vertices[index1]->getVertexLabel()))
            
                if (edge->getWeight() == weight) 
                    throw std::invalid_argument("Edge already exists");
                    
                else {
                    edge->setWeight(weight);    // update weight for existing edge
                    return;
                }    
        }

        // Add the edge to the adjacency list 
        adjacencyList[index1].push_back(index2);   
        
        adjacencyList[index2].push_back(index1);
        
        edges.push_back(new Edge(label1, label2, weight));  // create an instance of Edge class and add it to the Edges list
    }

    /**
     * @brief Removes an edge between two vertices.
     * 
     * @param label1 The label of the first vertex.
     * @param label2 The label of the second vertex.
     * 
     * @throws invalid_argument if vertices do not exist or if the edge does not exist.
     */
    void Graph::removeEdge( string label1,  string label2) {
        
        // Validate that both vertices exist
        int index1 = getVertexIndex(label1);
        int index2 = getVertexIndex(label2);
        
        if (index1 == -1 || index2 == -1) throw invalid_argument("Missing vertex");

        // Remove the edge from adjacency list of index1 to index2
        auto& neighbors1 = adjacencyList[index1];
        auto it2 = find(neighbors1.begin(), neighbors1.end(), index2);  // in vertex label1, find the neighbor with label2 
        
        if (it2 == neighbors1.end()) {
            throw invalid_argument("Edge does not exist between the specified vertices");
        }

        neighbors1.erase(it2);  // erase in vertex label1, the neighbor with label2

        // Remove the edge from adjacency list of index2 to index1
        auto& neighbors2 = adjacencyList[index2];
        auto it1 = find(neighbors2.begin(), neighbors2.end(), index1); // in vertexr label2, find the neighbor with label1 

        neighbors2.erase(it1);  // erase in vertex label2, the neighbor with label1

        // Remove the edge, if exist, from the edge list and free Edge memory

        for (auto it = edges.begin(); it != edges.end(); ) {
            
            Edge* edge = *it; //  iterator to access the edge pointer   ///Edge *edge = *it
            
            if ((edge->getVertexLabel1() == label1 && edge->getVertexLabel2() == label2) ||
                (edge->getVertexLabel1() == label2 && edge->getVertexLabel2() == label1)) {
                
                delete edge;    // Free the dynamically allocated edge instance memory
                
                it = edges.erase(it);     
            }
            else
                it++;
        }
/*
        // Remove the edge, if exist, from the edge list and free Edge memory
        bool found = false;
        
        edges.erase(
            remove_if(
                edges.begin(),
                edges.end(),
                [&](Edge* edge) {  // lambda function to find and remove the edge
                    found = (edge->getVertexLabel1() == label1 && edge->getVertexLabel2() == label2) ||
                            (edge->getVertexLabel1() == label2 && edge->getVertexLabel2() == label1); 
                            
                    if (found){
                        delete edge; // Free the dynamically allocated edge instance memory
                    }
                    return found;
                }
            ),
            edges.end()
        ); 
*/
    }


    /**
     * @brief Calculates the shortest path between two vertices using Dijkstra's Algorithm.
     * 
     * @param startLabel The label of the starting vertex.
     * @param endLabel The label of the ending vertex.
     * @param path Reference to a vector that stores the shortest path (as vertex labels).
     * 
     * @return The total weight of the shortest path. Returns ULONG_MAX if no path exists.
     * 
     * @throws invalid_argument if either vertex does not exist.
     */
    unsigned long Graph::shortestPath( string startLabel, string endLabel, vector<string>& path) {
        
        int start = getVertexIndex(startLabel);
        int end = getVertexIndex(endLabel);
    
        if (start == -1 || end == -1) throw invalid_argument("A Missing vertex");

        // Distances vector: stores the shortest distance from the start vertex to each vertex
        vector<unsigned long> distances (vertices.size(), numeric_limits<unsigned long>::max()); // vector initialized with ULONG_MAX to compare distances
        
        // Previous vector: stores the previous vertex indexes in the shortest path
        vector<int> previous (vertices.size(), -1);
    
        // Priority queue (PQ) for Dijkstra's algorithm. lambda fuction as comparator for PQ
        auto compare = [&](int a, int b) { return distances[a] > distances[b]; };   // PQ comparator to implement a Min-Heap
        
        priority_queue<int, vector<int>, decltype(compare)> pq (compare);  // declare PQ as a Min-Heap with comparator- lambda compare()
    
        // Initialize distances and add start vertex to the priority queue
        distances[start] = 0;
        pq.push(start);             // push start vertex  
    
        // Dijkstra's algorithm uses BFS traversal with a priority-queue (PQ) to keep the minimal distance on the root
        
        while (!pq.empty()) { 
            int current = pq.top();
            pq.pop();               // pop vertex with shorted distance to discover its neighbors
            
            // If we reached the destination vertex, exit the loop
            if (current == end) break;
    
            // Iterate through neighbors of the current vertex
            for (const auto& neighborIndex : adjacencyList[current]) {  // find the current vertex neighbors in the adjacencyList
                
                unsigned long edgeWeight = 0;
                
                for (const auto& edge : edges) {        // find the edge weight between current vertex its neighbor in edge list
                    if ((edge->getVertexLabel1() == vertices[current]->getVertexLabel() && edge->getVertexLabel2() == vertices[neighborIndex]->getVertexLabel()) // {A, B, 5} or 
                    ||  (edge->getVertexLabel1() == vertices[neighborIndex]->getVertexLabel() && edge->getVertexLabel2() == vertices[current]->getVertexLabel())) // {B, A, 5}
                    {

                        edgeWeight = edge->getWeight();
                        
                        break;       // when an edge is found break
                    }
                }
    
                if (edgeWeight == 0) continue;                              // if no valid edge exists

                unsigned long newDist = distances[current] + edgeWeight;    // increment the distance through this neighbor path
                
                // If a shorter path is found, then
                if (newDist < distances[neighborIndex]) {
                    
                    distances[neighborIndex] = newDist;             // found a new min distance to the neighbor

 
                    previous[neighborIndex] = current;              // keep previous neighbor of the path

                    pq.push(neighborIndex);                         // push neighbor w/min dist to the PQ

                }
            }
        }
    
        // If the destination vertex is unreachable
        if (distances[end] == numeric_limits<unsigned long>::max()) {
            return numeric_limits<unsigned long>::max();                       // return max possible unsigned long - ULONG_MAX
        }
    
        // Construct the shortest path
        path.clear(); // clear this var before using it because it comes from outside... could have not necessary data that make noises to us
        
        for (int at = end; at != -1; at = previous[at]) {
            path.push_back(vertices[at]->getVertexLabel());
        }
        
        reverse(path.begin(), path.end());
    
        return distances[end];                  // return the min distance
        
    }
    
    
    /**
     * @brief Prints the adjacency list for debugging purposes.
     */
    void Graph::printAdjacencyList() const {
        
        for (size_t i = 0; i < adjacencyList.size(); ++i)
        {
            cout << vertices[i]->getVertexLabel() << ": ";
            
            for (const auto& neighbor : adjacencyList[i])
            {
                cout << vertices[neighbor]->getVertexLabel() << " ";
            }
            cout << endl;
        }
    }
    
