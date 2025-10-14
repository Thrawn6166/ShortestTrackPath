#ifndef TRACK_
#define TRACK_
#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
class Track
{
    private:
    int K;   // nodes in direction of travel
    int H;    // nodes in direction of radius
    double I; // inner radius 
    double O; // outer radius

    class Node //internal class node
    {
        public:
        Node(double x_, double y_, int k_, int h_)
            : x(x_), y(y_), k(k_), h(h_) {}
        double x, y, k, h;
    };

    std::vector<Node> nodes; //

    public:
    Track(int K_, int H_, double I_, double O_); // constructor
    void generateNodes(); // generate the track made by node
    double Dijkstra(); // find the shortest path using Dijkstra algorithm
    inline int index(int k, int h) const; // auxiliar function to keep track the index of the track
    double EuclideanDistance(double ku, double hu, double kv, double hv) const; // auxiliar function to calculate distance between 2 nodes
    
    
};
#endif //Track