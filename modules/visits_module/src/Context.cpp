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

        void SetLocation(string robotName, FromTo location){
            auto it = robotLocation.find(robotName);
            if(it != robotLocation.end()){
                it->second = location;
            } else {
                robotLocation.insert({robotName,location});
            }
        };

        FromTo GetLocation(string robotName){
            auto it = robotLocation.find(robotName);
            return it->second;
        };

        void PrintAll(){
            for(auto x : robotLocation){
                cout << "\n--------------------------------------------------------\n" << "Robot: " << x.first << ". Regions: " << x.second << "\n--------------------------------------------------------"<< endl;
            }
        }

        bool IsEmpty(){
            if(robotLocation.empty()){
                return 1;
            };
            return 0;
            
        }
};
#endif