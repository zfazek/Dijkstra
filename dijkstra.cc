// Program for calculating shortest path of graphs using Dijkstra shortest
// past algorithm.
// Number of vertices, density of the graph, minimal and maximum values are set

#include <iostream>
#include <vector>
#include "edge.h"
#include "graph.h"

using namespace std;

// Class contains the Dijkstra algorithm for calculating
// the shortest path in a graph
class Dijkstra {

    private:
        Graph* graph;
        int numVertices;

    public:

        // Construts a new graph
        Dijkstra(int numVertices, int inital, double infinity):
            numVertices(numVertices) {
                graph = new Graph(numVertices, inital, infinity);
            }

        // Destructor. Frees memory allocated to graph object by new.
        ~Dijkstra() {
            delete graph;
        }

        // Adds all the edges according to density, min and max value
        void addEdges(double density, double min, double max) {
            double d;
            double n;
            int c = 0;
            for (int i = 0; i < numVertices; i++)
                for (int j = i; j < numVertices; j++)
                    if (i != j) {
                        d = ((double)rand()/(double)RAND_MAX);
                        if (d < density) {
                            n = min + ((double)rand()/(double)RAND_MAX) * 
                                (max - min);
                            graph->addEdge(i, j, n);
                            c++;
                        }
                    }
        }

        // Returns the graph
        Graph* getGraph() {
            return graph;
        }

        // Returns the shortest path size of the graph from vertex init to vertex dest
        double getPathSize(int init, int dest) {
            graph->initVertices();

            // At start each vertex is on the graph
            // If we calculate all the edges of the current vertex, we remove this
            // vertex from the set. Algorith goes until there is no vertex in the set
            while (graph->getNumVertices() > 0) {

                // Always deal with vertex with the minimal value
                // At start each vertex has infinity value but the inital one
                // Inital one has 0 value.
                // return -1 means no more vertex in the set, we calculated all of them
                // node variable contains the current vertex id
                int node = graph->getVertexIdWithMinimalValue();
                if (node == -1) {
                    break;
                }

                // get all the edges of current vertex. It could be as source and dest as well
                set<Edge*> edges = graph->getOwnEdges(node);

                // iterate over the edges
                for (const auto &e : edges) {

                    // if node is the source in the edge
                    if (node == e->source) {

                        // if current node value + edge value less than dest value ->
                        // we found a shorter path: update dest value
                        if (graph->hasVertex(e->dest) && 
                                graph->getVertexValue(e->dest) > 
                                graph->getVertexValue(node) + e->value) {
                            graph->setVertexValue(e->dest, 
                                    graph->getVertexValue(node) + e->value);
                        } else {}

                        // the same if the node is the dest in the edge
                    } else if (graph->hasVertex(e->source) && 
                            graph->getVertexValue(e->source) > 
                            graph->getVertexValue(node) + e->value) {
                        graph->setVertexValue(e->source, 
                                graph->getVertexValue(node) + e->value);
                    }
                }

                // if we calculated all the edges of the dest node the algorith is over
                if (node == dest) {
                    return graph->getVertexValue(node);
                }

                // if the current node is not the dest one, we remove it from the set
                // and take the next node with the minimal value
                graph->removeVertex(node);
            }

            // if we reach this point it means there is no way from init node to dest
            return -1;
        }

};

int main() {

    // init
    cout.precision(4);
    srand(time(NULL));
    int numVertices = 50;
    int init = 0;

    // infinity value technicaly any value than greater than the maximum value
    double infinity = 100.0;

    // Sets densities
    vector<double> densities;
    densities.push_back(0.2);
    densities.push_back(0.4);

    // Sets minimal and maximum values of the edges of the graph
    double min = 0.1;
    double max = 10.0;

    // Gets all the densities one by one
    for (const auto &d : densities) {

        // constructs the Dijkstra algorith to the given number of vertices and 
        // init node id.
        Dijkstra dijkstra(numVertices, init, infinity);

        // Adds edges with density, min and max value
        dijkstra.addEdges(d, min, max);

        // Gets shortest path value
        double value = 0;
        int weight = 0;

        // Prints inital values
        cout << endl << "density: " << d << ", number of edges: " 
            << dijkstra.getGraph()->getEdges().size() << endl << endl;
        dijkstra.getGraph()->printEdges();

        // Gets shortest paths from inital node to all the other nodes
        // id = 0 is the inital node
        for (int i = 0; i < numVertices; i++) {

            // we skip the path from init node to init node
            if (i != init) {
                double result = dijkstra.getPathSize(init, i);

                // Adds to result only if there is path from init to dest
                // result = -1 means there is no path from init to dest
                if (result > -1) {
                    weight++;
                    value += result;
                }

                // Prints shortest path from init to dest
                cout << "Shortest path value: " << init << " -> " << i << ": " 
                    << result << endl;
            }
        }

        // Prints the average value of all the shortest pahts from init vertex 
        // to all the other vertices
        cout << weight << " paths found, average shortest distance: " 
            << value/(double)weight << endl;
    }
    return 0;
}
