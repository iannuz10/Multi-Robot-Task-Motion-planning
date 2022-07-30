#ifndef _context_
#define _context_

#include <stdio.h>
#include <string>
#include <iostream>
#include "FromTo.cpp"
#include <unordered_map>
#include <list>

using namespace std;

class Context{
    private: 
        std::unordered_map<string, FromTo> robotLocation;
    public:
        Context(){};

        // Add or change entry
        void SetLocation(string robotName, FromTo location){
            auto it = robotLocation.find(robotName);
            if(it != robotLocation.end()){
                it->second = location;
            } else {
                robotLocation.insert({robotName,location});
            }
        };

        // Get location info of a given robot
        FromTo GetLocation(string robotName){
            auto it = robotLocation.find(robotName);
            return it->second;
        };

        // Print all context's content
        void PrintAll(){
            for(auto x : robotLocation){
                cout << "\n--------------------------------------------------------\n" << "Robot: " << x.first << ". Regions: " << x.second << "\n--------------------------------------------------------"<< endl;
            }
        }
};
#endif