/**************************************************************************************************/
// Test File for PP4
// Requires the Catch2 header file
// How to compile: g++ -std=c++17 -Wall -I$(CATCH_SINGLE_INCLUDE) (All cpp files)
// Example if Catch2 and source files are in this directory and at directory level: 
//    Example: g++ -std=c++17 -Wall *.cpp
// To see what tests were successful and failed, run your executable with the -s flag
//    Example: a.out -s
// A successful test should output: All tests passed (12 assertions in 1 test case)
/**************************************************************************************************/
/*
#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "Graph.hpp"

TEST_CASE("Graph Class Functionality - Provided Input", "[Graph]") {
    
    std::vector<std::string> vertexLabels = {"A", "B", "C", "D", "E", "F"};
    
    std::vector<std::tuple<std::string, std::string, unsigned long>> edges = {
        {"A", "B", 7}, {"A", "C", 9}, {"A", "F", 14},
        {"B", "C", 10}, {"B", "D", 15}, {"C", "D", 11},
        {"C", "F", 2}, {"D", "E", 6}, {"E", "F", 9}
    };

    Graph g(vertexLabels, edges);
    
    REQUIRE_NOTHROW(g.addVertex("H"));
    REQUIRE_NOTHROW(g.addVertex("I"));
    REQUIRE_NOTHROW(g.addEdge("H", "I", 5)); // Valid edge
    
    SECTION("Add and Remove Vertices") {
        SECTION("Add a new vertex") {
            REQUIRE_THROWS_AS(g.addVertex("A"), std::invalid_argument); // Duplicate vertex
        }

        SECTION("Remove an existing vertex") {
            REQUIRE_NOTHROW(g.removeVertex("A"));
            REQUIRE_THROWS_AS(g.removeVertex("G"), std::invalid_argument); // Nonexistent vertex
        }
    }

    SECTION("Add and Remove Edges") {
        SECTION("Add a new edge") {
            REQUIRE_NOTHROW(g.addEdge("B", "F", 5)); // Valid edge
            REQUIRE_NOTHROW(g.addEdge("E", "F", 5)); // Valid edge
            REQUIRE_THROWS_AS(g.addEdge("A", "B", 7), std::invalid_argument); // Duplicate edge
            REQUIRE_THROWS_AS(g.addEdge("A", "G", 10), std::invalid_argument); // Nonexistent vertex
        }

        SECTION("Remove an existing edge") {
            REQUIRE_NOTHROW(g.removeEdge("A", "B"));
            REQUIRE_THROWS_AS(g.removeEdge("A", "G"), std::invalid_argument); // Nonexistent edge
        }
    }

    SECTION("Shortest Path Calculations") {
        std::vector<std::string> path;

        SECTION("Shortest path between connected vertices") {
            REQUIRE(g.shortestPath("A", "E", path) == 20); // Expected path: A -> C -> F -> E
            REQUIRE(path == std::vector<std::string>{"A", "C", "F", "E"});
        }

        SECTION("Shortest path between directly connected vertices") {
            REQUIRE(g.shortestPath("A", "B", path) == 7); // Expected path: A -> B
            REQUIRE(path == std::vector<std::string>{"A", "B"});
        }

        SECTION("Shortest path between directly connected vertices") {
            REQUIRE(g.shortestPath("H", "I", path) == 5); // Expected path: H -> I
            REQUIRE(path == std::vector<std::string>{"H", "I"});
        }
        
        SECTION("Shortest path between directly connected vertices") {
            REQUIRE(g.shortestPath("H", "I", path) == 5); // Expected path: H -> I
            REQUIRE(path == std::vector<std::string>{"H", "I"});
        }

        SECTION("Shortest path when no path exists") {
            g.removeEdge("H", "I");
            REQUIRE(g.shortestPath("H", "I", path) == std::numeric_limits<unsigned long>::max());
            REQUIRE(path.empty()); 
        }
    }

    SECTION("Graph Initialization and Adjacency List") {
        std::stringstream adjacencyList;
        std::streambuf* originalBuffer = std::cout.rdbuf();
        std::cout.rdbuf(adjacencyList.rdbuf());

        g.printAdjacencyList();

        std::cout.rdbuf(originalBuffer); // Restore the original buffer

        REQUIRE(adjacencyList.str() == 
            "A: B C F \n"
            "B: A C D \n"
            "C: A B D F \n"
            "D: B C E \n"
            "E: D F \n"
            "F: A C E \n"
            "H: I \n"
            "I: H \n"
            );
    }

    SECTION("Edge Case Testing") {
        SECTION("Adding self-loops throws an exception") {
            REQUIRE_THROWS_AS(g.addEdge("A", "A", 10), std::invalid_argument);
        }

        SECTION("Removing non-existent vertices or edges throws exceptions") {
            REQUIRE_THROWS_AS(g.removeVertex("G"), std::invalid_argument);
            REQUIRE_THROWS_AS(g.removeEdge("A", "G"), std::invalid_argument);
        }
    }
}
*/
