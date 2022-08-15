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


#ifndef TESTSOLVER_H
#define TESTSOLVER_H

#include "ExternalSolver.h"
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <queue>
#include <unordered_map>
#include "Context.h"
#include "FromTo.h"
#include "Semaphore.h"

using namespace std;

class VisitSolver : public ExternalSolver
{
public:
    VisitSolver();
    ~VisitSolver();
    virtual void loadSolver(string* parameters, int n);
    virtual map<string,double> callExternalSolver(map<string,double> initialState, bool isHeuristic);
    virtual  list<string> getParameters();
    virtual  list<string> getDependencies();
    map<string, vector<double>> waypoint;
    map<string, vector<double>> landmark;
   
    int parseWaypoint(string waypoint_file);
    void parseLandmark(string landmark_file);
    void parseEdges(string edges_file);
    void initParser(Context* context, string fileName);

    map<string, vector<string>> region_mapping;
    vector <string> source, target; 
    string starting_position;
    
    void setContext(Context* c){
        this->context = c;
    };
   
    void initAdjMatrix();
    void printAdjMatrix();
    void weightAdjMatrix();
    double dijkstraShortestPath(double **am, int target, int dest, string ID, bool collisionFlag, int collidingNode, int collidingNodeLevel); 

    void parseParameters(string parameters);

private:
    Context* context;   // All robot locations are stored here 
    double **wpAdjMatrix;
    int totalWaypoints, totalRobots;
    double cost;
    string pathID; 

    map<string, int> initRobotLocation;
    map<string, vector<int>> paths;
    map<string, double> pathsCosts;
    
    Semaphore sem;

    list<string> affected;
    list<string> dependencies;
      
    double calculateExtern(double external, double total_cost);
    //void localize(string from, string to);
    vector<string> findParameters(string line, int&n);

};

#endif // TESTSOLVER_H
