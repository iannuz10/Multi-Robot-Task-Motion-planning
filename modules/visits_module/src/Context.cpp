#include <stdio.h>
#include <string>
#include <iostream>
#include <unordered_map>
#include <list>
#include "Context.h"

using namespace std;

Context::Context(){};

void Context::SetLocation(string robotName, FromTo location){
    auto it = robotLocation.find(robotName);
    if(it != robotLocation.end()){
        it->second = location;
    } else {
        robotLocation.insert({robotName,location});
    }
};

FromTo Context::GetLocation(string robotName){
    auto it = robotLocation.find(robotName);
    return it->second;
};

void Context::PrintAll(){
    for(auto x : robotLocation){
        cout << "\n--------------------------------------------------------\n" << "Robot: " << x.first << ". Regions: " << x.second << "\n--------------------------------------------------------"<< endl;
    }
}