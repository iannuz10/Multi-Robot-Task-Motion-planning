#include "MyShortestPath.h"
#include <algorithm>

#define INF INT_MAX

using namespace std;

MyShortestPath::MyShortestPath(){}

vector<int>* MyShortestPath::myShortestPath(map<string,vector<int>*> paths, double **graph, vector<int>* currentPath, int destination, int dim, string pathID){
    vector<int>* dijkstraNodes = new vector<int>;
    if(ExternalSolver::verbose){
        cout << "[myShortestPath]: Calling dijkstra on path: " << pathID << endl;
        cout << "[myShortestPath]: Current path is: " ;
        for(int i = 0; i < currentPath->size(); i++){
            cout << currentPath->at(i) << " ";
        }
        cout << endl;
    }
    dijkstraNodes = currentPath;
    vector<int>* tempDijPath = new vector<int>;

    // Running dijkstra
    tempDijPath = dijkstraPath(graph, currentPath->back(), destination, dim);
    if(ExternalSolver::verbose) cout << "[myShortestPath]: Back to myShortestPath... adding found path to nodes vector..." << endl;

    // Inserting in solution vector the result of dijkstra
    dijkstraNodes->insert(dijkstraNodes->end(), tempDijPath->begin(), tempDijPath->end());
    if(ExternalSolver::verbose) cout << "[myShortestPath]: Looking for collisions..." << endl;

    // Checking for colliding nodes
    int collisionIndex = checkCollision(paths, currentPath, pathID);
    if(ExternalSolver::verbose) cout << "[myShortestPath]: collision value: " << collisionIndex << endl;

    // Collision is detected
    if(collisionIndex != -1){
        cout << "[myShortestPath]: COLLISION DETECTED!" << endl;

        // Path is fine untill the colliding node
        dijkstraNodes->resize(collisionIndex);
        if(ExternalSolver::verbose) cout << "[myShortestPath]: Locking busy node..." << endl;

        // Creating a new graph that ignores colliding node
        double **graphIteration = iterationGraphInit(dim, graph);
        if(ExternalSolver::verbose) cout << "[myShortestPath]: The colliding node is " << dijkstraNodes->at(collisionIndex-1) << " at index " << collisionIndex-1 << endl;
        lockCollidingNode(dim, graphIteration, dijkstraNodes->at(collisionIndex-1));
        if(ExternalSolver::verbose) cout << "[myShortestPath]: New graph generated..." << endl;
        vector<int>* dijkstraIteration = new vector<int>;

        // searching a path from the last safe node with dijkstra ignoring the colliding one
        dijkstraIteration = dijkstraPath(graphIteration, currentPath->back(), destination, dim);

        // No alternative path found
        if(dijkstraIteration->empty()){
            multimap<double, int> adjacentNodes;

            // First attempt is to stay still and wait for the colliding node to free
            adjacentNodes.insert({0,dijkstraNodes->at(collisionIndex-1)});

            // Populate map with adjacent nodes ordered by cost
            for(int i = 0; i < dim; i++){
                if(graph[dijkstraNodes->at(collisionIndex-1)][i] != 0){
                    adjacentNodes.insert({graph[dijkstraNodes->at(collisionIndex-1)][i],i});
                }
            }

            multimap<double, int>::iterator adjacenceIterator;

            // Looking for a free node to perform the waiting movement, avoiding blocking previous robots paths
            for(adjacenceIterator = adjacentNodes.begin(); adjacenceIterator != adjacentNodes.end(); adjacenceIterator++){
                dijkstraNodes->push_back(adjacenceIterator->second);
                collisionIndex = checkCollision(paths, currentPath, pathID);
                    if(collisionIndex != -1){
                        dijkstraNodes->pop_back();
                    } else {
                        return myShortestPath(paths, graph, dijkstraNodes, destination, dim, pathID);
                    }
            }
        } else {
            // Alternative path found, adding result to solution vector
            vector<int>* tempCurrPath = new vector<int>;
            tempCurrPath->push_back(dijkstraNodes->back());
            tempCurrPath->insert(tempCurrPath->end(), dijkstraIteration->begin(), dijkstraIteration->end());
            return myShortestPath(paths, graph, tempCurrPath, destination, dim, pathID);
        }
    }else{
        // No collision is detected
        return dijkstraNodes;
    }

}

vector<int>* MyShortestPath::dijkstraPath(double **graph, int target, int dest, int dim){
    vector<int>* path = new vector<int>;
    struct{
        double cost;
        int next;
        bool def;
    } n[dim];

    int src = target;
    int i, min, indmin, iter, node;

    // Initialization
    for(i = 0; i < dim; i++){
        n[i].cost = INF;
        n[i].def = false;
        n[i].next = -1;
    }

    n[target].cost = 0;
    n[target].next = target;


    if(ExternalSolver::verbose) cout << "[dijkstra]: Starting dijkstra algorithm!" << endl;
    iter = 0;
    do{
        n[target].def = true;
        for(i = 0; i < dim; i++){
            if(graph[target][i] != 0){
            if((n[target].cost + graph[target][i]) < n[i].cost){
                n[i].cost = n[target].cost + graph[target][i];
                n[i].next = target;
            }
            }
        }
        min = INF;
        indmin = -1;
        for(i = 0; i < dim; i++){
            if(n[i].def == false){
            if(n[i].cost < min){
                min = n[i].cost;
                indmin = i;
            }
            }
        }
        target = indmin;
        iter++;
    } while(indmin != -1);

    if(ExternalSolver::verbose){    
        cout<<"[dijkstra]: Vertex\t\tDistance from source vertex"<<endl;
        for(int k = 0; k < dim; k++){ 
            cout << k << "\t\t\t" << n[k].cost << endl;
        }
    }

    node = dest;
    do{
        if(n[node].next == -1)  // Returning empty vector if no path is found
            return path;
        path->push_back(node);
        node = n[node].next;
    } while (node != src);
    reverse(path->begin(), path->end());

    if(ExternalSolver::verbose){
        cout << "[dijkstra]: Dijkstra found path long " << path->size() << endl;
        cout << "[dijkstra]: Path: " << endl;
        for(int i = 0; i < path->size(); i++){
            cout << path->at(i) << " " ;
        }
        cout << endl;
        cout << "[dijkstra]: Returning path to myShortestPath" << endl;
    }

    return path;
    }

// Checking for each previous planification if the current robots path collides
int MyShortestPath::checkCollision(map<string,vector<int> *> paths, vector<int>* currentPath, string pathID){
    if(ExternalSolver::verbose) cout << "[checkCollision]: Checking..." << endl;
    vector<int>::iterator currPathIter, otherPathsNodeIter;
    map<string, vector<int>*>::iterator otherPathsIter;
    int currPathNodeIndex;
    for(currPathIter = currentPath->begin(); currPathIter != currentPath->end(); currPathIter++){
        currPathNodeIndex = currPathIter - currentPath->begin();
        for(otherPathsIter = paths.begin(); otherPathsIter != paths.end(); otherPathsIter++){
            cout << "Current robots step: " << pathID[0] << "; other robot step: " << otherPathsIter->first[0] << endl;
            if(pathID[0] == otherPathsIter->first[0] && currPathNodeIndex < otherPathsIter->second->size() && *currPathIter == otherPathsIter->second->at(currPathNodeIndex))
                return currPathNodeIndex;
            
        }
    }
    return -1;
}

// Creates a copy of the adjacence matrix passed
double** MyShortestPath::iterationGraphInit(int dim, double **originalGraph){
    double **graphIteration = new double*[dim]; 
    for (int i = 0; i < dim; i++) 
    graphIteration[i] = new double[dim];

    for (int i = 0; i < dim; i++){
        for (int j = 0; j < dim; j++){
            graphIteration[i][j] = originalGraph[i][j];
        }
    }
    if(ExternalSolver::verbose) cout << "[iterationGraphInit]: Adjacence matrix copy succeded..." << endl;
    return graphIteration;
}

// Treats given colliding node as non reachable
void MyShortestPath::lockCollidingNode(int dim, double **graph, int collidingNode){
    if(ExternalSolver::verbose) cout << "[lockCollidingNode]: Locking node..." << endl;
    for(int i = 0; i < dim; i++){
        if(graph[i][collidingNode] != 0){
            graph[i][collidingNode] = 0;
            graph[collidingNode][i] = 0;
        }
    }
}