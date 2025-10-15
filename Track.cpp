#include "Track.h"

Track::Track(int K_, int H_, double I_, double O_)
    : K(K_), H(H_), I(I_), O(O_) {}

void Track::generateNodes()
{
    nodes.clear(); // in case i generate different track

    //creation of a vector that is being filed from outer to inner node from left to right
    for(int k=0; k<K; ++k)
    {
        double theta = M_PI * k / (K - 1); // from 0 to π
        for(int h=0; h<H; ++h)
        {
            double distance = I+(static_cast<double>(h)/(H-1))*(O-I); // from inner to outer board
            double x = distance*cos(theta);
            double y = distance*sin(theta);
            nodes.push_back({x, y, k, h});
        }
        
    }
}

// Dijkstra
double Track::Dijkstra() {
    const double INF = std::numeric_limits<double>::infinity();
    int totNodes = K * H;

    std::vector<double> dist(totNodes, INF); //keep track of of distance of node from start
    std::vector<bool> visited(totNodes, false); //keep track of visited node

    // Start node
    int start = index(0, H - 1);

    // End node
    int goal = index(K - 1, H - 1);

    dist[start] = 0.0;

    // priority queue <distance, node index>
    using Pair = std::pair<double,int>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> ShortestPathQueue; // a queue where the first element are the one whit shortest path
    ShortestPathQueue.push({0.0, start});

    // possible moviment (8 direction)
    int dk[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dh[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    while (!ShortestPathQueue.empty()) {

        //extraction of the node
        Pair pNode = ShortestPathQueue.top();
        double d = pNode.first; // current minimum distance
        int u = pNode.second; //index of current node
        ShortestPathQueue.pop();

        //check if the node is already visited
        if (visited[u])
            continue;
        visited[u] = true;

        if (u == goal)
            return d; // shortest path found, return distance 

        int ku = u / H; // current row index
        int hu = u % H; // current column index

        for (int i = 0; i < 8; ++i) // try all possible direction to move
        {
            int kv = ku + dk[i]; //  row index of next node
            int hv = hu + dh[i]; // column index of next node
            if (kv < 0 || kv >= K || hv < 0 || hv >= H) //check if try to go outside the board
                continue;

            int v = index(kv, hv); //index of next node

            double w=EuclideanDistance(ku,hu,kv,hv); // distance between current a next node

            // dijkstra relaxing, if the new distance is shorter, insert in the PriorityQueue
            if (dist[v] > dist[u] + w)
            {
                dist[v] = dist[u] + w;
                ShortestPathQueue.push({dist[v], v});
            }
        }
    }

    return INF; // no path find
}

//helper function
//keep track of the index
inline int Track::index(int k, int h) const
{ 
    return k * H + h;
}
//calculate distance between node
double Track::EuclideanDistance(double ku, double hu, double kv, double hv) const
{
    double dx = nodes[ku * H + hu].x - nodes[kv * H + hv].x;
    double dy = nodes[ku * H + hu].y - nodes[kv * H + hv].y;
    return std::sqrt(dx * dx + dy * dy);
}