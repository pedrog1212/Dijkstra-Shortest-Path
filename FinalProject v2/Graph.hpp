#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <algorithm>
#include <tuple>
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
    explicit Vertex(const std::string& label) : vertexLabel(label) {}

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
    
    //friend class Graph;
};


/**
 * @brief Represents an edge in the graph.
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
    
    //friend class Graph;
    
};

/**
 * @brief Represents an undirected weighted graph.
 */
 
class Graph 
{
    
private:
    std::vector<Vertex*> vertices;                      // List of vertices
    
    std::vector<Edge*> edges;                           // List of edges
    
    std::unordered_map<std::string, int> labelToIndex;  // Map vertex labels to their index
    
    std::vector<std::vector<int>> adjacencyList;        // Adjacency list representation
    

    int getVertexIndex(const std::string& label) const {
        
        auto it = labelToIndex.find(label);
        
        if (it == labelToIndex.end()) {
            throw std::invalid_argument("Vertex not found");
        }
        
        return it->second;
    }

public:
    // Constructor
    Graph(const std::vector<std::string>& vertexLabels,
          const std::vector<std::tuple<std::string, std::string, unsigned long>>& edgeTuples )
    {
        for (const auto& label : vertexLabels) {   // add vertices to the adjacent list
            addVertex(label);
        }
        
        for (const auto& [label1, label2, weight] : edgeTuples) { // add edges to the adjacent list
            addEdge(label1, label2, weight);
        }
    }

    // Destructor
    ~Graph() {
        for (auto vertex : vertices) {   // remove all vertex instances
            delete vertex;
        }
        
        for (auto edge : edges) {   // remove all edges instances
            delete edge;
        }
        // clear containers
        vertices.clear();
        edges.clear();
        adjacencyList.clear();
        labelToIndex.clear();
    }

    /**
     * @brief Adds a vertex to the graph.
     * @param label The label of the vertex to add.
     * @throws std::invalid_argument if a vertex with the same label already exists.
     */
    void addVertex(const std::string& label) {
        
        if (labelToIndex.find(label) != labelToIndex.end()) {
            throw std::invalid_argument("Vertex already exists");
        }
        vertices.push_back(new Vertex(label));
        labelToIndex[label] = vertices.size() - 1;
        adjacencyList.emplace_back(); // Add an empty adjacency list for the new vertex
    }

    /**
     * @brief Removes a vertex from the graph.
     * @param label The label of the vertex to remove.
     * @throws std::invalid_argument if the vertex does not exist.
     */
    void removeVertex(const std::string& label) {
        int index = getVertexIndex(label);

        // Remove edges pointing to this vertex
        for (auto& neighbors : adjacencyList) {
            
            for (auto& n: neighbors)
                cout << n << endl;

            neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), index), neighbors.end());
        }

        // Remove the vertex and its adjacency list
        delete vertices[index];
        vertices.erase(vertices.begin() + index);
        adjacencyList.erase(adjacencyList.begin() + index);

        // Update label-to-index map and adjust adjacency list
        labelToIndex.erase(label);
        for (auto& pair : labelToIndex) {
            if (pair.second > index) {
                pair.second--;
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
    void addEdge(const std::string& label1, const std::string& label2, unsigned long weight) {
        int index1 = getVertexIndex(label1);
        int index2 = getVertexIndex(label2);

        // Check if the edge already exists
        for (const auto& neighbor : adjacencyList[index1]) {
            if (neighbor == index2) {
                throw std::invalid_argument("Edge already exists");
            }
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
    void removeEdge(const std::string& label1, const std::string& label2) {
        // Validate that both vertices exist
        int index1 = getVertexIndex(label1);
        int index2 = getVertexIndex(label2);

        // Remove the edge from adjacency list of index1 to index2
        auto& neighbors1 = adjacencyList[index1];
        auto it1 = std::find(neighbors1.begin(), neighbors1.end(), index2);
        if (it1 == neighbors1.end()) {
            throw std::invalid_argument("Edge does not exist between the specified vertices");
        }
        neighbors1.erase(it1);

        // Remove the edge from adjacency list of index2 to index1
        auto& neighbors2 = adjacencyList[index2];
        auto it2 = std::find(neighbors2.begin(), neighbors2.end(), index1);
        neighbors2.erase(it2);

        // Remove the edge from the edge list
        edges.erase(std::remove_if(edges.begin(), edges.end(),
                                   [&](Edge* edge) {
                                       return (edge->getVertexLabel1() == label1 && edge->getVertexLabel2() == label2) ||
                                              (edge->getVertexLabel1() == label2 && edge->getVertexLabel2() == label1);
                                   }),
                    edges.end());
    }

    /**
     * @brief Prints the adjacency list for debugging purposes.
     */
    void printAdjacencyList() const {
        
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
   
};
