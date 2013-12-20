#include "graph.h"

using namespace std;

Graph::Graph(int numVertices, int inital, double infinity) :
    numVertices(numVertices), inital(inital), infinity(infinity) {
        initVertices();
    }

// Destructor. Frees memory allocated to edges object by new.
Graph::~Graph() {
    for (const auto &e : edges)
        delete e;
}

// Returns inital vertex
int Graph::getInital() {
    return inital;
}

// Adds one edge identified by source, dest vertices and value.
void Graph::addEdge(int source, int dest, double value) {
    Edge* e = new Edge(source, dest, value);
    edges.insert(e);
}

// Prints all the edges of the graph
void Graph::printEdges() {
    cout << endl << "Edges:\n";
    for (const auto &e : edges)
        cout << "(" << e->source << ", " << e->dest << "): " 
            << e->value << "\t";
    cout << endl;
}

// Returns the set of edges
set<Edge*> Graph::getEdges() {
    return edges;
}

// Sets inital values for the vertex
// Each vertex gets infinity as value
// inital vertex gets 0 as value
void Graph::initVertices() {
    for (int i = 0; i < numVertices; i++)
        vertices[i] = infinity;
    vertices[inital] = (double)0;
}

// Prints all the vertices
void Graph::printVertices() {
    for (const auto &v : vertices)
        cout << v.first << " " << v.second << endl;
}

// Return the number of vertices
int Graph::getNumVertices() {
    return vertices.size();
}

// Returns the vertex id which has minimal value
int Graph::getVertexIdWithMinimalValue() {
    int id = -1;
    double min = infinity;
    for (const auto &v : vertices)
        if (v.second < min) {
            id = v.first;
            min = v.second;
        }
    return id;
}

// Returns the edges where either source or dest equals id
set<Edge*> Graph::getOwnEdges(int id) {
    set<Edge*> ret;
    for (const auto &e : edges)
        if (e->source == id || e->dest == id)
            ret.insert(e);
    return ret;
}

// Returns if vertex id exists or not
bool Graph::hasVertex(int id) {
    return vertices.find(id) != vertices.end();
}

// Returns the value of vertex id
double Graph::getVertexValue(int id) {
    return vertices[id];
}

// Sets the value of vertex id to value
void Graph::setVertexValue(int id, double value) {
    vertices[id] = value;
}

// Removes vertex id from vertices
void Graph::removeVertex(int id) {
    vertices.erase(id);
}

