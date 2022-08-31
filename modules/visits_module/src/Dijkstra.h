#ifndef _dijkstra_h_
#define _dijkstra_h_

#include <stdio.h>
#include <string>
#include <map>
#include <ostream>
#include <iterator>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>

using namespace std;

class dijkstra{
    public:
        double dijkstraShortestPath(double **graph, int target, int dest, int dim);
};

#endif