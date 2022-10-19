#ifndef _shortest_path_h_
#define _shortest_path_h_

#include <stdio.h>
#include <string>
#include <map>
#include <ostream>
#include <vector>
#include <iterator>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <iostream>
#include <algorithm>
#include <stdlib.h>
#include "VisitSolver.h"

using namespace std;

class MyShortestPath{
    public:
        MyShortestPath();
        vector<int>* myShortestPath(map<string, vector<int>* >* paths, double **graph, vector<int>* currentPath, int destination, int dim, string pathID);
        vector<int>* dijkstraPath(double **graph, int target, int dest, int dim);
        int checkCollision(map<string, vector<int>* >* paths, vector<int>* currentPath, string pathID);
        double** updateGraphWithBusyNode(double **graph, vector<int>* pathTillCollision);
        double** iterationGraphInit(int dim, double **originalGraph);
        void lockCollidingNode(int dim, double **graph, int collidingNode);
};

#endif