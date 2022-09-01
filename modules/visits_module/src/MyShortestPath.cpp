#include "MyShortestPath.h"
#include <algorithm>

#define INF INT_MAX

using namespace std;

MyShortestPath::MyShortestPath(){}

vector<int>* MyShortestPath::myShortestPath(map<string,vector<int>*> paths, double **graph, vector<int>* currentPath, int destination, int dim, string pathID){
    vector<int>* dijkstraNodes = new vector<int>;
    cout << "[myShortestPath]: Calling dijkstra on path: " << pathID << endl;
    cout << "[myShortestPath]: Current path is: " ;
    for(int i = 0; i < currentPath->size(); i++){
        cout << currentPath->at(i) << " ";
    }
    cout << endl;
    dijkstraNodes = currentPath;
    vector<int>* tempDijPath = new vector<int>;
    tempDijPath = dijkstraPath(graph, currentPath->back(), destination, dim);
    cout << "[myShortestPath]: Back to myShortestPath... adding found path to nodes vector..." << endl;
    dijkstraNodes->insert(dijkstraNodes->end(), tempDijPath->begin(), tempDijPath->end());

    cout << "[myShortestPath]: Looking for collisions..." << endl;
    int collisionIndex = checkCollision(paths, currentPath, pathID);

    cout << "[myShortestPath]: collision value: " << collisionIndex << endl;
    
    if(collisionIndex != -1){
        cout << "[myShortestPath]: COLLISION DETECTED!" << endl;
        dijkstraNodes->resize(collisionIndex);

        cout << "[myShortestPath]: Locking busy node..." << endl;
        double **graphIteration = iterationGraphInit(dim, graph);
        cout << "[myShortestPath]: The colliding node is " << dijkstraNodes->at(collisionIndex-1) << " at index " << collisionIndex-1 << endl;
        lockCollidingNode(dim, graphIteration, dijkstraNodes->at(collisionIndex-1));
        cout << "[myShortestPath]: New graph generated..." << endl;
        vector<int>* dijkstraIteration = new vector<int>;
        dijkstraIteration = dijkstraPath(graphIteration, currentPath->back(), destination, dim);
        if(dijkstraIteration->empty()){
            multimap<double, int> adjacentNodes;
            adjacentNodes.insert({0,dijkstraNodes->at(collisionIndex-1)});
            for(int i = 0; i < dim; i++){
                if(graph[dijkstraNodes->at(collisionIndex-1)][i] != 0){
                    adjacentNodes.insert({graph[dijkstraNodes->at(collisionIndex-1)][i],i});
                }
            }
            multimap<double, int>::iterator adjacenceIterator;
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
            vector<int>* tempCurrPath = new vector<int>;
            tempCurrPath->push_back(dijkstraNodes->back());
            tempCurrPath->insert(tempCurrPath->end(), dijkstraIteration->begin(), dijkstraIteration->end());
            return myShortestPath(paths, graph, tempCurrPath, destination, dim, pathID);
        }
    }else{
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

    iter = 0;
    cout << "[dijkstra]: Starting dijkstra algorithm!" << endl;
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

    cout<<"[dijkstra]: Vertex\t\tDistance from source vertex"<<endl;
    for(int k = 0; k < dim; k++){ 
        cout << k << "\t\t\t" << n[k].cost << endl;
    }

    node = dest;
    do{
        if(n[node].next == -1)  // NO PATH FOUND
            return path;
        path->push_back(node);
        node = n[node].next;
    } while (node != src);
    cout/* << node << " "*/ << endl;
    reverse(path->begin(), path->end());

    cout << "[dijkstra]: Dijkstra found path long " << path->size() << endl;
    cout << "[dijkstra]: Path: " << endl;
    for(int i = 0; i < path->size(); i++){
        cout << path->at(i) << " " ;
    }
    cout << endl;
    
    cout << "[dijkstra]: Returning path to myShortestPath" << endl;
    return path;
    // return n[dest].cost;
    }






int MyShortestPath::checkCollision(map<string,vector<int> *> paths, vector<int>* currentPath, string pathID){
    cout << "[checkCollision]: Checking..." << endl;
    vector<int>::iterator currPathIter, otherPathsNodeIter;
    map<string, vector<int>*>::iterator otherPathsIter;
    int currPathNodeIndex;
    for(currPathIter = currentPath->begin(); currPathIter != currentPath->end(); currPathIter++){
        currPathNodeIndex = currPathIter - currentPath->begin();
        for(otherPathsIter = paths.begin(); otherPathsIter != paths.end(); otherPathsIter++){
            if(currPathNodeIndex < otherPathsIter->second->size() && *currPathIter == otherPathsIter->second->at(currPathNodeIndex))
                return currPathNodeIndex;
            
        }
    }
    return -1;
}







double** MyShortestPath::iterationGraphInit(int dim, double **originalGraph){
    double **graphIteration = new double*[dim]; 
    for (int i = 0; i < dim; i++) 
    graphIteration[i] = new double[dim];

    for (int i = 0; i < dim; i++){
        for (int j = 0; j < dim; j++){
            graphIteration[i][j] = originalGraph[i][j];
        }
    }
    cout << "[iterationGraphInit]: Adjacence matrix copy succeded..." << endl;
    return graphIteration;
}







void MyShortestPath::lockCollidingNode(int dim, double **graph, int collidingNode){
    cout << "[lockCollidingNode]: Locking node..." << endl;
    for(int i = 0; i < dim; i++){
        if(graph[i][collidingNode] != 0){
            graph[i][collidingNode] = 0;
            graph[collidingNode][i] = 0;
        }
    }
}