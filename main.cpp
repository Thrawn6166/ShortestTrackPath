#include "Track.h"
int main() {

    Track t(128, 24, 7.0, 10.0); //creation of track
    t.generateNodes();
    std::cout << "the shortest parth of the track " << t.Dijkstra()  <<" metres ";
    return 0;
}