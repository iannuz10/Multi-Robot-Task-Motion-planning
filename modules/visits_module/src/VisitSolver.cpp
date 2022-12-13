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
#include <cmath>
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
  if(ExternalSolver::verbose) cout << "Initializing solver\n";

  starting_position = "r0";
  string Paramers = parameters[0];

  char const *x[]={"dummy"};
  char const *y[]={"act-cost","triggered"};
  parseParameters(Paramers);
  affected = list<string>(x,x+1);
  dependencies = list<string>(y,y+2);

  string waypoint_file = "/home/iannuz/popf-tif-v2/domains/visits_domain/waypoint.txt";   // change this to the correct path
  totalWaypoints = parseWaypoint(waypoint_file);
  if(ExternalSolver::verbose){
    cout << "A total of " << totalWaypoints << " waypoints have been counted." << endl;
    cout << "Waypoint vector content: " << endl;
  }
  map<string, vector<double>>::iterator itr;
  for(itr = waypoint.begin(); itr != waypoint.end(); itr++){
    vector<double> vec = itr->second;
  }

  string landmark_file = "/home/iannuz/popf-tif-v2/domains/visits_domain/landmark.txt";  // change this to the correct path
  parseLandmark(landmark_file);

  string edges_file = "/home/iannuz/popf-tif-v2/domains/visits_domain/edges.txt";  // change this to the correct path
  initAdjMatrix();
  parseEdges(edges_file);
  if(ExternalSolver::verbose) cout << "Adjacencies Matrix: " << endl;
  printAdjMatrix();
  weightAdjMatrix();
  if(ExternalSolver::verbose) cout << "\nWeighted Adjacencies Matrix: " << endl;
  printAdjMatrix();

  // Initializing shared space for robot location
  Context *context = new Context();
  // Getting and setting initial location of all robots
  string prob_file = "/home/iannuz/popf-tif-v2/domains/visits_domain/prob1.pddl";
  initParser(context, prob_file);
  if(ExternalSolver::verbose) cout << "Init Parser completed" << endl;

  // Setting context with initial info
  setContext(context);

  // pathCostComputed = false;
  // semaphoreCounter = -totalRobots;

  // // Setting semaphore
  // sem.setCount(-(totalRobots));   // Makes sure that cost are sent to planner only after a definitive and consistent state
  // cout << "Semaphore is set!" << endl;

  //startEKF();
}

map<string,double> VisitSolver::callExternalSolver(map<string,double> initialState,bool isHeuristic, map<string, vector<int>*>* paths){
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
    
    function.erase(0,1);                                    // remove the first character
    function.erase(function.length()-1,function.length());  // remove the last character
    int n=function.find(" ");
    // if(ExternalSolver::verbose) cout << "Function found: " << function << endl;

    if(n!=-1){
      string arg=function;


      int curr=n+1;
      int next=function.find(" ",curr);
      curr=next+1;
      next=function.find(" ",curr);

      string tmp = function.substr(n+1,next-n); // get the second argument
      //cout << "Tmp is: " << tmp << endl;
      string robot = function.substr(next+1,function.length()-1); // get the third argument
      //cout << "Robot is: " << robot << endl;
      function.erase(n,function.length()-1);
      arg.erase(0,n+1);

      if(function=="triggered"){
        trigger[arg] = value>0?1:0;
        if (value>0){
          vector<int>* path = new vector<int>;

          n=tmp.find(" ");
          string from = tmp.substr(0,n);   // from and to are regions, need to extract wps (poses)
          //cout << "From is: " << from << endl;
          curr=n+1;
          next=tmp.find(" ",curr);
          string to = tmp.substr(curr,tmp.length()-1);
          //cout << "To is: " << to << endl;
          double tempCost = 0;
          FromTo location(from,to);
          this->context->setLocation(robot,location);
          from.erase(0,1);
          to.erase(0,1);

          if(ExternalSolver::verbose){
            cout << "From: " << from << endl;
            cout << "To: " << to << endl;
          }

          // Iterate every pathID of the paths map
          map<string, vector<int>*>::iterator it;
          int step = 0;
          if(paths->size() > 0){
            for(it = paths->begin(); it != paths->end(); it++){
              string ID = it->first;
              if(ExternalSolver::verbose) cout << "ID: " << ID << endl;
              // Erase from ID the step number
              int pos = ID.find("-");
              ID.erase(0,pos+1);
              if(ExternalSolver::verbose){
                cout << "Erased ID: " << ID << endl;
                cout << "Robot: " << robot << endl;
              }
              // If ID is equal to robot, count++
              if(ID == robot) step++;
            }
          } else {
            if(ExternalSolver::verbose) cout << "No paths found" << endl;
          }
          
          pathID = to_string(step)+ "-" + robot;
          
          // Compute minimum path
          auto it1 = paths->find(pathID);
          if(it1 == paths->end()){
            MyShortestPath pathFinder;

            // Passing vector containing only first node (from region)
            path->push_back(stoi(from));
            vector<int>* pathTemp = new vector<int>;
            if(ExternalSolver::verbose){
              cout << "PathTemp created for " << pathID << "..." << endl;
              cout << "Calling path finder..." << endl;
            }

            // Starting best path finder algorithm and getting the path
            pathTemp = pathFinder.myShortestPath(paths, wpAdjMatrix, path, stoi(to), totalWaypoints, pathID);
            
            if(ExternalSolver::verbose){
              cout << "Path creation succeded!" << endl;
              for(int i = 0; i < path->size(); i++){
                cout << path->at(i);
              } cout << endl;
              cout << "Insertion of new path succeded!" << endl;
            }
            // Computing cost of the path
            for(int i = 0; i < path->size()-1; i++){
              tempCost += wpAdjMatrix[path->at(i)][path->at(i+1)];
            }
            // tempCost = rand() % 10;
            cout << endl << "DijkstraShortestPath cost: " << tempCost << endl;
            

            // Inserting new path in the paths map
            string previousStepID;
            if(step > 0){ 
              previousStepID = to_string(step-1) + "-" + robot;       // This is necessary because the planner runs the triggered function
              auto it3 = paths->find(previousStepID);                 // one more time when the "at start" and "at end" conditions are not in series
              if(it3 != paths->end()){
                // confront current path with previous path    
                if((path->at(0) != it3->second->at(0)) && (path->at(path->size()-1) != it3->second->at(it3->second->size()-1))){
                  paths->insert({pathID,path});
                }
              
              } else {
                paths->insert({pathID,path});
              }
            }else{
              paths->insert({pathID,path});
            }
            // Inserting new path cost in the pathsCosts map
            auto it4 = pathsCosts.find(pathID);
            if(it4 != pathsCosts.end()){
              it4->second = tempCost;
            } else {
              pathsCosts.insert({pathID,tempCost});
            }
          }
          cost = pathsCosts[pathID];
          cout << "\nCost is: " << cost << endl;
          

          // Printing all paths
          if(ExternalSolver::verbose){
            cout << "Printing all paths" << endl;
            map<string, vector<int>*>::iterator it;
            std::vector<int>::iterator it2;
            for(it = paths->begin(); it != paths->end(); it++){   
              cout << "Checking path: " << it->first << endl; 
              for(it2 = it->second->begin(); it2 != it->second->end(); it2++){
                cout << *it2 << "\t";
              } cout << endl;
            }
          }
          
          // Printing all costs
          if(ExternalSolver::verbose){
            cout << "Printing all costs" << endl;
            for(auto x : pathsCosts){
              cout << "PathID: " << x.first << ". Cost: " << x.second << endl;
            }
          }
        }
      }
    } else {
      if(function=="dummy"){
        dummy = cost;
      } else if (function=="act-cost"){
        act_cost = value;
        if(ExternalSolver::verbose) context->printAll();
      }
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
      curr = curr+1;
      while(true){
        next = line.find(" ", curr);
        region_mapping[region_name].push_back(line.substr(curr,next-curr).c_str());
        if (next == -1)
          break;
          curr = next+1;

      }                
    }
  }
}

double VisitSolver::calculateExtern(double external, double total_cost){

  //float random1 = static_cast <float> (rand())/static_cast <float>(RAND_MAX);
  // double cost;
  // if(totalRobots > 1)
  //   cost = external/(totalRobots-1);//random1;
  // else
  double outputCost = pathsCosts[pathID];
  return outputCost;
}

void VisitSolver::initParser(Context* context, string fileName){
    totalRobots = 0;
    int curr;
    string line;
    string locationKeyword = "robot_in";
    ifstream fio(fileName.c_str());

    if(ExternalSolver::verbose) cout << "Opening file\n";
    if (fio.is_open()){
        if(ExternalSolver::verbose) cout << "File is open\n";
        while(getline(fio,line)){
            if(line.find(locationKeyword) != std::string::npos){    // Check if the keyword is contained in the line and ignores the others
                curr=line.find(locationKeyword); 

                // Cleaning string from useless chars
                line.erase(line.length()-1,line.length());
                line.erase(0,curr+locationKeyword.length()+1);
                
                string robotName = line.substr(0,line.find(" "));               // Gets robot name
                string from = line.substr(line.find(" ")+1,line.length()-1);    // Gets from region

                // Transforms to lowercase to match the name format used during execution
                transform(robotName.begin(), robotName.end(),robotName.begin(), ::tolower); 
                transform(from.begin(), from.end(),from.begin(), ::tolower);

                // Debug print
                if(ExternalSolver::verbose){
                  cout << "Robot name: " << robotName << endl;
                  cout << "From region: " << from << endl;
                }
                from.erase(0,1);
                totalRobots++;
                if(ExternalSolver::verbose) cout << "Found " << totalRobots << " robots." << endl;
                // Add initial positions into context
                FromTo initLocation(from," ");
                context->setLocation(robotName,initLocation);
            }
        }
    }
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
      if(ExternalSolver::verbose) cout << "Printing line: " << line << endl;

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
  if(ExternalSolver::verbose){
    cout << "\t";
    for (int i = 0; i < totalWaypoints; i++){
      cout << i << "\t";
    } cout << endl;
    for (int i = 0; i < totalWaypoints; i++){
      cout << i << "\t";
      for (int j = 0; j < totalWaypoints; j++){
        cout << wpAdjMatrix[i][j] << "\t";
      } cout << endl;
    } cout << endl;
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
        
        // Calculating distance between connected waypoints
        wpAdjMatrix[stoi(firstWp)][stoi(secondWp)] *= sqrt((pow((vec1[0]-vec2[0]),2))+(pow((vec1[1]-vec2[1]),2)));
        // if(vec1[1] > 2  || vec2[1] > 2 ){
        //   wpAdjMatrix[stoi(firstWp)][stoi(secondWp)] *= 10;
        // }
        // wpAdjMatrix[stoi(secondWp)][stoi(firstWp)] *= sqrt((pow((vec1[0]-vec2[0]),2))+(pow((vec1[1]-vec2[1]),2)));

      }
    }
  }
  if(ExternalSolver::verbose) cout << "Weighting complete!" << endl;
}