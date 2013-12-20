#ifndef EDGE_H
#define EDGE_H

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



#endif
