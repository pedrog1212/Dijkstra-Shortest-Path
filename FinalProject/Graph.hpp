#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <queue>
#include <limits>
#include <algorithm>

class Graph {
private:
    struct Edge {
        std::string vertex;
        unsigned long weight;

        Edge(const std::string& v, unsigned long w) : vertex(v), weight(w) {}
    };

    struct Vertex {
        std::string label;
        std::vector<Edge> edges;

        Vertex(const std::string& lbl) : label(lbl) {}
    };

    std::unordered_map<std::string, Vertex*> vertices;

public:
    Graph() = default;

    ~Graph() {
        for (auto& pair : vertices) {
            delete pair.second;
        }
        vertices.clear();
    }

    void addVertex(const std::string& label) {
        if (vertices.find(label) != vertices.end()) {
            throw std::invalid_argument("Vertex already exists");
        }
        vertices[label] = new Vertex(label);
    }

    void removeVertex(const std::string& label) {
        if (vertices.find(label) == vertices.end()) {
            throw std::invalid_argument("Vertex does not exist");
        }

        // Remove edges in other vertices pointing to this vertex
        for (auto& pair : vertices) {
            pair.second->edges.erase(
                std::remove_if(pair.second->edges.begin(), pair.second->edges.end(),
                               [&](Edge& e) { return e.vertex == label; }),
                pair.second->edges.end());
        }

        // Remove the vertex itself
        delete vertices[label];
        vertices.erase(label);
    }

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
        auto exists = std::find_if(v1->edges.begin(), v1->edges.end(),
                                   [&](Edge& e) { return e.vertex == label2; });
        if (exists != v1->edges.end()) {
            throw std::invalid_argument("Edge already exists");
        }

        v1->edges.emplace_back(label2, weight);
        v2->edges.emplace_back(label1, weight); // Undirected graph
    }

    void removeEdge(const std::string& label1, const std::string& label2) {
        if (vertices.find(label1) == vertices.end() || vertices.find(label2) == vertices.end()) {
            throw std::invalid_argument("Both vertices must exist");
        }

        Vertex* v1 = vertices[label1];
        Vertex* v2 = vertices[label2];

        auto it1 = std::remove_if(v1->edges.begin(), v1->edges.end(),
                                  [&](Edge& e) { return e.vertex == label2; });
        if (it1 == v1->edges.end()) {
            throw std::invalid_argument("Edge does not exist");
        }
        v1->edges.erase(it1, v1->edges.end());

        auto it2 = std::remove_if(v2->edges.begin(), v2->edges.end(),
                                  [&](Edge& e) { return e.vertex == label1; });
        v2->edges.erase(it2, v2->edges.end());
    }

    unsigned long shortestPath(const std::string& startLabel, const std::string& endLabel, std::vector<std::string>& path) {
        if (vertices.find(startLabel) == vertices.end() || vertices.find(endLabel) == vertices.end()) {
            throw std::invalid_argument("Both vertices must exist");
        }

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

        while (!pq.empty()) {
            std::string current = pq.top();
            pq.pop();

            if (current == endLabel) {
                break;
            }

            for (const auto& edge : vertices[current]->edges) {
                unsigned long newDist = distances[current] + edge.weight;
                if (newDist < distances[edge.vertex]) {
                    distances[edge.vertex] = newDist;
                    previous[edge.vertex] = current;
                    pq.push(edge.vertex);
                }
            }
        }

        if (distances[endLabel] == std::numeric_limits<unsigned long>::max()) {
            return std::numeric_limits<unsigned long>::max(); // No path
        }

        path.clear();
        for (std::string at = endLabel; at != ""; at = previous[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());

        return distances[endLabel];
    }
};
