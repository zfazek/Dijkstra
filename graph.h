#pragma once

#include <map>
#include <set>
#include <iostream>
#include "edge.h"

// Class contains the graph.
// Each graph has vertices, edges
class Graph {

    private:
        std::map<int, double> vertices;
        std::set<Edge*> edges;
        int numVertices;
        int inital;
        int dest;
        double infinity;

    public:

        // Constructor. Setting number of vertices, inital vertex and the 
        // maximum value
        Graph(int numVertices, int inital, double infinity);

        // Destructor. Frees memory allocated to edges object by new.
        ~Graph();

        // Returns inital vertex
        int getInital();

        // Adds one edge identified by source, dest vertices and value.
        void addEdge(int source, int dest, double value);

        // Prints all the edges of the graph
        void printEdges();

        // Returns the set of edges
        std::set<Edge*> getEdges();

        // Sets inital values for the vertex
        // Each vertex gets infinity as value
        // inital vertex gets 0 as value
        void initVertices();

        // Prints all the vertices
        void printVertices();

        // Return the number of vertices
        int getNumVertices();

        // Returns the vertex id which has minimal value
        int getVertexIdWithMinimalValue();

        // Returns the edges where either source or dest equals id
        std::set<Edge*> getOwnEdges(int id);

        // Returns if vertex id exists or not
        bool hasVertex(int id);

        // Returns the value of vertex id
        double getVertexValue(int id);

        // Sets the value of vertex id to value
        void setVertexValue(int id, double value);

        // Removes vertex id from vertices
        void removeVertex(int id);

};
