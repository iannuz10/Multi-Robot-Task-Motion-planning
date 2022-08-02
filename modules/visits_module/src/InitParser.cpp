#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <algorithm>
#include "InitParser.h"

using namespace std;

InitParser::InitParser(Context* context){
    int curr;
    string line;

    // Prob file location
    string fileName = "/home/iannuz/visits/visits_domain/prob1.pddl";
    ifstream fio(fileName.c_str());

    cout << "Opening file\n";
    if (fio.is_open()){
        cout << "File is open\n";
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
                cout << "Robot name: " << robotName << endl;
                cout << "From region: " << from << endl;

                // Add initial positions into context
                FromTo initLocation(from," ");
                context->setLocation(robotName,initLocation);
            }
        }
    }
}

InitParser::~InitParser()
{
}


