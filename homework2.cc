// Program for calculating shortest path of graphs using Dijkstra shortest
// past algorithm.
// Number of vertices, density of the graph, minimal and maximum values are set

#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <cstdlib>

using namespace std;

// Class storing the edges of the graph.
// Each edge has a source and a dest id and a value between source and dest vertex.
class Edge {
    
    // members are public just because easy access within the same project.
    // They should be private with proper getters and setters for real projects.
    public:
        int source;
        int dest;
        double value;

        // Constructor. Setting source, dest and value
        Edge(int source, int dest, double value) :
            source(source), dest(dest), value(value) {}

        // need this operator overloading to store Edge object in an ordered container
        bool operator<(const Edge& e) const {
            if (source < e.source)
                return true;
            if (source > e.source)
                return false;
            if (dest < e.dest)
                return true;
            return false;
        }
};

// Class contains the graph.
// Each graph has vertices, edges
class Graph {

    private:
        map<int, double> vertices;
        set<Edge*> edges;
        int numVertices;
        int inital;
        int dest;
        double infinity;

    public:

        // Constructor. Setting number of vertices, inital vertex and the maximum value
        Graph(int numVertices, int inital, double infinity) :
            numVertices(numVertices), inital(inital), infinity(infinity) {
//                vertices = map<int, double>();
//                edges = new set<Edge*>();
                initVertices();
            }

        // Destructor. Frees memory allocated to edges object by new.
        ~Graph() {
            for (set<Edge*>::const_iterator it = edges.begin(); it != edges.end(); it++) 
                delete *it;
        }

        // Returns inital vertex
        int getInital() {
            return inital;
        }

        // Adds one edge identified by source, dest vertices and value.
        void addEdge(int source, int dest, double value) {
            Edge* e = new Edge(source, dest, value);
            edges.insert(e);
        }

        // Prints all the edges of the graph
        void printEdges() {
            cout << endl << "Edges:\n";
            for (set<Edge*>::const_iterator it = edges.begin(); it != edges.end(); it++)
                cout << "(" << (*it)->source << ", " << (*it)->dest << "): " << (*it)->value << "\t";
            cout << endl;
        }

        // Returns the set of edges
        set<Edge*> getEdges() {
            return edges;
        }

        // Sets inital values for the vertex
        // Each vertex gets infinity as value
        // inital vertex gets 0 as value
        void initVertices() {
            for (int i = 0; i < numVertices; i++)
                vertices[i] = infinity;
            vertices[inital] = (double)0;
        }

        // Prints all the vertices
        void printVertices() {
            for (map<int, double>::const_iterator it = vertices.begin();
                    it != vertices.end();
                    it++)
                cout << it->first << " " << it->second << endl;
        }

        // Return the number of vertices
        int getNumVertices() {
            return vertices.size();
        }

        // Returns the vertex id which has minimal value
        int getVertexIdWithMinimalValue() {
            int id = -1;
            double min = infinity;
            for (map<int, double>::const_iterator it = vertices.begin(); it != vertices.end(); it++)
                if (it->second < min) {
                    id = it->first;
                    min = it->second;
                }
            return id;
        }

        // Returns the edges where either source or dest equals id
        set<Edge*> getOwnEdges(int id) {
            set<Edge*> ret;
            for (set<Edge*>::const_iterator it = edges.begin(); it != edges.end(); it++)
                if ((*it)->source == id || (*it)->dest == id)
                    ret.insert(*it);
            return ret;
        }

        // Returns if vertex id exists or not
        bool hasVertex(int id) {
            map<int, double>::const_iterator it = vertices.find(id);
            return it != vertices.end();
        }

        // Returns the value of vertex id
        double getVertexValue(int id) {
            return vertices[id];
        }

        // Sets the value of vertex id to value
        void setVertexValue(int id, double value) {
            vertices[id] = value;
        }

        // Removes vertex id from vertices
        void removeVertex(int id) {
            vertices.erase(id);
        }
};

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
                            n = min + ((double)rand()/(double)RAND_MAX) * (max - min);
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
                for (set<Edge*>::const_iterator it = edges.begin(); it != edges.end(); ++it) {

                    // if node is the source in the edge
                    if (node == (*it)->source) {

                        // if current node value + edge value less than dest value ->
                        // we found a shorter path: update dest value
                        if (graph->hasVertex((*it)->dest) && 
                                graph->getVertexValue((*it)->dest) > 
                                graph->getVertexValue(node) + (*it)->value) {
                            graph->setVertexValue((*it)->dest, 
                                    graph->getVertexValue(node) + (*it)->value);
                        } else {}

                        // the same if the node is the dest in the edge
                    } else if (graph->hasVertex((*it)->source) && 
                            graph->getVertexValue((*it)->source) > 
                            graph->getVertexValue(node) + (*it)->value) {
                        graph->setVertexValue((*it)->source, 
                                graph->getVertexValue(node) + (*it)->value);
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
    for (vector<double>::const_iterator it = densities.begin(); it != densities.end(); ++it) {

        // constructs the Dijkstra algorith to the given number of vertices and init node id.
        Dijkstra dijkstra(numVertices, init, infinity);

        // Adds edges with density, min and max value
        dijkstra.addEdges(*it, min, max);

        // Gets shortest path value
        double value = 0;
        int weight = 0;

        // Prints inital values
        cout << endl << "density: " << *it << ", number of edges: " << dijkstra.getGraph()->getEdges().size() << endl << endl;
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
                cout << "Shortest path value: " << init << " -> " << i << ": " << result << endl;
            }
        }

        // Prints the average value of all the shortest pahts from init vertex to all the other vertices
        cout << weight << " paths found, average shortest distance: " << value/(double)weight << endl;
    }
    return 0;
}
