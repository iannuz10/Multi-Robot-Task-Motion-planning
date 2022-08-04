    /*
     <one line to give the program's name and a brief idea of what it does.>
     Copyright (C) 2015  <copyright holder> <email>
     
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
     */


#include "VisitSolver.h"
#include "ExternalSolver.h"
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include "armadillo"
#include <initializer_list>

using namespace std;
using namespace arma;



//map <string, vector<double> > region_mapping;

extern "C" ExternalSolver* create_object(){
  return new VisitSolver();
}

extern "C" void destroy_object(ExternalSolver *externalSolver){
  delete externalSolver;
}

VisitSolver::VisitSolver(){
  
}

VisitSolver::~VisitSolver(){

}

void VisitSolver::loadSolver(string *parameters, int n){
  cout << "Initializing solver\n";

  starting_position = "r0";
  string Paramers = parameters[0];

  char const *x[]={"dummy"};
  char const *y[]={"act-cost","triggered"};
  parseParameters(Paramers);
  affected = list<string>(x,x+1);
  dependencies = list<string>(y,y+2);

  string waypoint_file = "/home/iannuz/popf-tif-v2/domains/visits_domain/waypoint.txt";   // change this to the correct path
  totalWaypoints = parseWaypoint(waypoint_file);
  cout << "A total of " << totalWaypoints << " waypoints have been counted." << endl;
  cout << "Waypoint vector content: " << endl;

  map<string, vector<double>>::iterator itr;
  for(itr=waypoint.begin();itr!=waypoint.end();itr++)
  {
    cout << itr->first << " ";
    
    vector<double> vec = itr->second;

    for (int i=0; i<vec.size();i++)
    {
        cout<<vec[i]<<" ";
    }
    cout << endl;
  }

  wpOccupation = new bool[totalWaypoints];
  for(int i = 0; i < totalWaypoints; i++){
    wpOccupation[i] = true;     // true = free, false = busy
  }

  string landmark_file = "/home/iannuz/popf-tif-v2/domains/visits_domain/landmark.txt";  // change this to the correct path
  parseLandmark(landmark_file);

  string edges_file = "/home/iannuz/popf-tif-v2/domains/visits_domain/edges.txt";  // change this to the correct path
  initAdjMatrix();
  parseEdges(edges_file);
  cout << "Adjacencies Matrix: " << endl;
  printAdjMatrix();
  weightAdjMatrix();
  cout << "\nWeighted Adjacencies Matrix: " << endl;
  printAdjMatrix();

  // Initializing shared space for robot location
  Context *context = new Context();

  // Getting and setting initial location of all robots
  InitParser parser(context);
  setContext(context);

  cout << "Init Parser completed\n";
  //startEKF();
}

map<string,double> VisitSolver::callExternalSolver(map<string,double> initialState,bool isHeuristic){

  map<string, double> toReturn;
  map<string, double>::iterator iSIt = initialState.begin();
  map<string, double>::iterator isEnd = initialState.end();
  double dummy;
  double act_cost;
  

  map<string, double> trigger;

  for(;iSIt!=isEnd;++iSIt){

    string parameter = iSIt->first;
    string function = iSIt->first;
    double value = iSIt->second;

    function.erase(0,1);
    function.erase(function.length()-1,function.length());
    int n=function.find(" ");
    cout << "Function found: " << function << endl;

    if(n!=-1){
      string arg=function;
      string tmp = function.substr(n+1,5);

      string robot = function.substr(n+7,function.length()-1);

      function.erase(n,function.length()-1);
      arg.erase(0,n+1);

      if(function=="triggered"){
        trigger[arg] = value>0?1:0;
        if (value>0){

          // Regions accepted: r0-r9
          string from = tmp.substr(0,2);   // from and to are regions, need to extract wps (poses)
          string to = tmp.substr(3,2);
          
          FromTo location(from,to);
          this->context->setLocation(robot,location);
          from.erase(0,1);
          to.erase(0,1);

          cout << "From: " << from << endl;
          cout << "To: " << to << endl;
          cost = dijkstraShortestPath(wpAdjMatrix, stoi(from), stoi(to));
          cout << "Cost from dijkstraAlgo: " << cost << endl;
          cout << "Path from source to destination" << endl;
          /*
          Connect wp to make edges (wpX, wpY)
          Calculate cost between edges
          Make weighted adjacency matrix to check node neighbours
          Search for minimun path (dijkstra(?))
          Change calculateExtern logic to match the value from minimum path search
          Check for collision avoidance (the wp must be used once ata a time): 
                    if the planning is done right then it means it is calculated in parallel
          */


           // distance_euc(from, to);



        }
      }
    } else {
      if(function=="dummy"){
        dummy = cost;
      } else if (function=="act-cost"){
        act_cost = value;
        context->printAll();
      } //else if(function=="dummy1"){
          //duy = value;              
          ////cout << parameter << " " << value << endl;
        //}
    }
  }


  double results = calculateExtern(dummy, act_cost);
  
  if (ExternalSolver::verbose){
    cout << "(dummy) " << results << endl;
  }

  toReturn["(dummy)"] = results;

  return toReturn;

}

list<string> VisitSolver::getParameters(){

  return affected;
}

list<string> VisitSolver::getDependencies(){

  return dependencies;
}


void VisitSolver::parseParameters(string parameters){

  int curr, next;
  string line;
  ifstream parametersFile(parameters.c_str());
  if (parametersFile.is_open()){
    while (getline(parametersFile,line)){
      curr=line.find(" ");
      string region_name = line.substr(0,curr).c_str();
      curr=curr+1;
      while(true ){
        next=line.find(" ",curr);
        region_mapping[region_name].push_back(line.substr(curr,next-curr).c_str());
        if (next ==-1)
          break;
          curr=next+1;

      }                
    }
  }
}

double VisitSolver::calculateExtern(double external, double total_cost){
  //float random1 = static_cast <float> (rand())/static_cast <float>(RAND_MAX);
  cout << endl << "calculateExtern received cost: " << external << endl;
  double cost = external;//random1;
  return cost;
}

int VisitSolver::parseWaypoint(string waypoint_file){

  int curr, next;
  int numberOfWaypoints = 0;
  string line;
  double pose1, pose2, pose3;
  ifstream parametersFile(waypoint_file);
  if (parametersFile.is_open()){
    while (getline(parametersFile,line)){
      if(curr=line.find("[")){
        string waypoint_name = line.substr(0,curr).c_str();

        curr=curr+1;
        next=line.find(",",curr);

        pose1 = (double)atof(line.substr(curr,next-curr).c_str());
        curr=next+1; next=line.find(",",curr);

        pose2 = (double)atof(line.substr(curr,next-curr).c_str());
        curr=next+1; next=line.find("]",curr);

        pose3 = (double)atof(line.substr(curr,next-curr).c_str());

        waypoint[waypoint_name] = vector<double> {pose1, pose2, pose3};

        numberOfWaypoints++;
      }
    }
  }
  return numberOfWaypoints;
}

void VisitSolver::parseLandmark(string landmark_file){

  int curr, next;
  string line;
  double pose1, pose2, pose3;
  ifstream parametersFile(landmark_file);
  if (parametersFile.is_open()){
    while (getline(parametersFile,line)){
      curr=line.find("[");
      string landmark_name = line.substr(0,curr).c_str();
      
      curr=curr+1;
      next=line.find(",",curr);

      pose1 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; next=line.find(",",curr);

      pose2 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; next=line.find("]",curr);

      pose3 = (double)atof(line.substr(curr,next-curr).c_str());

      landmark[landmark_name] = vector<double> {pose1, pose2, pose3};
    }
  }
}

void VisitSolver::parseEdges(string edges_file){
  int curr;
  string line;
  ifstream edgesFile(edges_file);
  if(edgesFile.is_open()){
    while (getline(edgesFile,line)){
      cout << "Printing line: " << line << endl;

      curr=line.find(",");

      string s = line.substr(2,curr-2).c_str();
      string f = line.substr(curr+3,line.length()).c_str();

      wpAdjMatrix[stoi(s)][stoi(f)] = 1;
      wpAdjMatrix[stoi(f)][stoi(s)] = 1;
    }
  }
}

void VisitSolver::initAdjMatrix(){
  wpAdjMatrix = new double*[totalWaypoints]; 
    for (int i = 0; i < totalWaypoints; i++) 
      wpAdjMatrix[i] = new double[totalWaypoints];

    for (int i = 0; i < totalWaypoints; i++){
      for (int j = 0; j < totalWaypoints; j++){
        wpAdjMatrix[i][j] = 0;
      }
    }
}

void VisitSolver::printAdjMatrix(){
  for (int i = 0; i < totalWaypoints; i++){
      for (int j = 0; j < totalWaypoints; j++){
        cout << wpAdjMatrix[i][j] << " ";
      }
      cout << endl;
    }
}

void VisitSolver::weightAdjMatrix(){
  cout << "Starting to weight Matrix" << endl;
  string firstWp,secondWp;
  map<string, vector<double>>::iterator itr1, itr2;
  for(itr1=waypoint.begin();itr1!=waypoint.end();itr1++){
    for(itr2=waypoint.begin();itr2!=waypoint.end();itr2++){
      if(itr1->first != itr2->first){
        vector<double> vec1 = itr1->second;
        vector<double> vec2 = itr2->second;

        firstWp = itr1->first;
        secondWp = itr2->first;
        
        firstWp.erase(0,2);
        secondWp.erase(0,2);
        
        wpAdjMatrix[stoi(firstWp)][stoi(secondWp)] *= sqrt(pow(vec1[0]-vec2[0],2)+pow(vec1[1]-vec2[1],2));
        wpAdjMatrix[stoi(secondWp)][stoi(firstWp)] *= sqrt(pow(vec1[0]-vec2[0],2)+pow(vec1[1]-vec2[1],2));;

      }
      
      cout << endl;
    }
  }
  cout << "Weighting complete!" << endl;
}

double VisitSolver::dijkstraShortestPath(double **am, int target, int dest){
  struct{
    double cost;
    int next;
    bool def;
  } n[totalWaypoints];
  int src = target;
  int i, min, indmin, iter;

  // Initialization
  for(i = 0; i < totalWaypoints; i++){
    n[i].cost = INT_MAX;
    n[i].def = false;
    n[i].next = -1;
  }

  n[target].cost = 0;
  n[target].next = target;

  cout << "Starting dijkstra algorithm!" << endl;
  iter = 0;
  do{
    n[target].def = true;

    for(i = 0; i < totalWaypoints; i++){
      if(wpAdjMatrix[target][i] != 0){
        if((n[target].cost + wpAdjMatrix[target][i]) < n[i].cost){
          n[i].cost = n[target].cost + wpAdjMatrix[target][i];
          n[i].next = target;
        }
      }
    }
    min = INT_MAX;
    indmin = -1;
    for(i = 0; i < totalWaypoints; i++){
      if(n[i].def == false){
        if((n[i].cost < min) && wpOccupation[i]){
          min = n[i].cost;
          indmin = i;
        }
      }
    }
    if(indmin != -1){
      wpOccupation[indmin] = false;
      wpOccupation[n[indmin].next] = true;
    }
    target = indmin;
    iter++;
  } while(indmin != -1);

  cout<<"Vertex\t\tDistance from source vertex"<<endl;
  for(int k = 0; k < totalWaypoints; k++){ 
    cout << k << "\t\t\t" << n[k].cost << endl;
  }

  // int node = dest;
  // do{
  //   wpOccupation[node] = false;
  //   node = n[node].next;
  // }while(node != src);

  return n[dest].cost;
}



//void VisitSolver::distance_euc( string from, string to){
//} 