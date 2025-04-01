#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <queue>
#include <limits>
#include <algorithm>

/**
 * @brief Class representing an edge in an undirected weighted graph.
 */
class Edge {
private:
    std::string vertex;      // The label of the connected vertex.
    unsigned long weight;    // The weight of the edge.

public:
    // Constructor
    Edge(const std::string& v, unsigned long w) : vertex(v), weight(w) {}

    // Getter for connected vertex
    const std::string& getVertex() const { return vertex; }

    // Getter for edge weight
    unsigned long getWeight() const { return weight; }
};

/**
 * @brief Class representing a vertex in an undirected weighted graph.
 */
class Vertex {
private:
    std::string label;              // The label of the vertex.
    std::vector<Edge> edges;        // Adjacency list for the vertex.

public:
    // Constructor
    Vertex(const std::string& lbl) : label(lbl) {}

    // Getter for label
    const std::string& getLabel() const { return label; }

    // Getter for edges
    const std::vector<Edge>& getEdges() const { return edges; }

    // Add an edge
    void addEdge(const std::string& vertex, unsigned long weight) {
        edges.emplace_back(vertex, weight);
    }

    // Remove an edge
    void removeEdge(const std::string& vertex) {
        edges.erase(std::remove_if(edges.begin(), edges.end(),
                                   [&](const Edge& e) { return e.getVertex() == vertex; }),
                    edges.end());
    }
};

/**
 * @brief Class representing an undirected weighted graph.
 */
class Graph {
private:
    std::unordered_map<std::string, Vertex*> vertices; // Map of vertex labels to vertex objects.

public:
    // Constructor
    Graph() = default;

    // Destructor
    ~Graph() {
        for (auto& pair : vertices) {
            delete pair.second;
        }
        vertices.clear();
    }

    /**
     * @brief Adds a vertex to the graph.
     * @param label The label of the vertex to add.
     * @throws std::invalid_argument if the vertex already exists.
     */
    void addVertex(const std::string& label) {
        if (vertices.find(label) != vertices.end()) {
            throw std::invalid_argument("Vertex already exists");
        }
        vertices[label] = new Vertex(label);
    }

    /**
     * @brief Removes a vertex from the graph, along with all its edges.
     * @param label The label of the vertex to remove.
     * @throws std::invalid_argument if the vertex does not exist.
     */
    void removeVertex(const std::string& label) {
        if (vertices.find(label) == vertices.end()) {
            throw std::invalid_argument("Vertex does not exist");
        }

        // Remove edges in other vertices pointing to this vertex.
        for (auto& pair : vertices) {
            pair.second->removeEdge(label);
        }

        // Remove the vertex itself.
        delete vertices[label];
        vertices.erase(label);
    }

    /**
     * @brief Adds an edge between two vertices.
     * @param label1 The label of the first vertex.
     * @param label2 The label of the second vertex.
     * @param weight The weight of the edge.
     * @throws std::invalid_argument if vertices do not exist, if an edge already exists, or if self-loop is attempted.
     */
    void addEdge(const std::string& label1, const std::string& label2, unsigned long weight) {
        if (label1 == label2) {
            throw std::invalid_argument("A vertex cannot have an edge to itself");
        }
        if (vertices.find(label1) == vertices.end() || vertices.find(label2) == vertices.end()) {
            throw std::invalid_argument("Both vertices must exist");
        }

        Vertex* v1 = vertices[label1];
        Vertex* v2 = vertices[label2];

        // Check if edge already exists
        for (const auto& edge : v1->getEdges()) {
            if (edge.getVertex() == label2) {
                throw std::invalid_argument("Edge already exists");
            }
        }

        v1->addEdge(label2, weight);
        v2->addEdge(label1, weight); // Undirected graph
    }

    /**
     * @brief Removes an edge between two vertices.
     * @param label1 The label of the first vertex.
     * @param label2 The label of the second vertex.
     * @throws std::invalid_argument if vertices or edge do not exist.
     */
    void removeEdge(const std::string& label1, const std::string& label2) {
        if (vertices.find(label1) == vertices.end() || vertices.find(label2) == vertices.end()) {
            throw std::invalid_argument("Both vertices must exist");
        }

        Vertex* v1 = vertices[label1];
        Vertex* v2 = vertices[label2];

        // Remove edge in both directions
        v1->removeEdge(label2);
        v2->removeEdge(label1);
    }

    /**
     * @brief Calculates the shortest path between two vertices using Dijkstra's Algorithm.
     * @param startLabel The label of the start vertex.
     * @param endLabel The label of the end vertex.
     * @param path Reference to a vector that will store the shortest path.
     * @return The total weight of the shortest path. Returns ULONG_MAX if no path exists.
     * @throws std::invalid_argument if vertices do not exist.
     */
    unsigned long shortestPath(const std::string& startLabel, const std::string& endLabel, std::vector<std::string>& path) {
        if (vertices.find(startLabel) == vertices.end() || vertices.find(endLabel) == vertices.end()) {
            throw std::invalid_argument("Both vertices must exist");
        }

        // Distance map and priority queue setup
        std::unordered_map<std::string, unsigned long> distances;
        std::unordered_map<std::string, std::string> previous;
        for (const auto& pair : vertices) {
            distances[pair.first] = std::numeric_limits<unsigned long>::max();
        }
        distances[startLabel] = 0;

        auto compare = [&](const std::string& a, const std::string& b) {
            return distances[a] > distances[b];
        };
        std::priority_queue<std::string, std::vector<std::string>, decltype(compare)> pq(compare);

        pq.push(startLabel);

        // Dijkstra's Algorithm
        while (!pq.empty()) {
            std::string current = pq.top();
            pq.pop();

            if (current == endLabel) {
                break;
            }

            for (const auto& edge : vertices[current]->getEdges()) {
                unsigned long newDist = distances[current] + edge.getWeight();
                if (newDist < distances[edge.getVertex()]) {
                    distances[edge.getVertex()] = newDist;
                    previous[edge.getVertex()] = current;
                    pq.push(edge.getVertex());
                }
            }
        }

        // Reconstruct path
        if (distances[endLabel] == std::numeric_limits<unsigned long>::max()) {
            return std::numeric_limits<unsigned long>::max(); // No path exists
        }

        path.clear();
        for (std::string at = endLabel; at != ""; at = previous[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());

        return distances[endLabel];
    }
};

#endif // GRAPH_HPP
