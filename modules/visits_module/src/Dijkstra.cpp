#include <limits.h>
#include "Dijkstra.h"

#define INF INT_MAX

using namespace std;


double dijkstra::dijkstraShortestPath(double **graph, int target, int dest, int dim){
  struct{
    double cost;
    int next;
    bool def;
  } n[dim];

  int i, min, indmin, iter;

  // Initialization
  for(i = 0; i < dim; i++){
    n[i].cost = INF;
    n[i].def = false;
    n[i].next = -1;
  }

  n[target].cost = 0;
  n[target].next = target;

  iter = 0;
  do{
    cout << "Starting dijkstra algorithm!" << endl;
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

  cout<<"Vertex\t\tDistance from source vertex"<<endl;
  for(int k = 0; k < dim; k++){ 
    cout << k << "\t\t\t" << n[k].cost << endl;
  }

  return n[dest].cost;
}