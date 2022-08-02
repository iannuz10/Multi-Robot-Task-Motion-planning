#include <stdio.h>
#include <string>
#include <iostream>
#include <unordered_map>
#include <list>
#include "Context.h"

using namespace std;

Context::Context(){};

void Context::setLocation(string robotName, FromTo location){
    auto it = robotLocation.find(robotName);
    if(it != robotLocation.end()){
        it->second = location;
    } else {
        robotLocation.insert({robotName,location});
    }
};

FromTo Context::getLocation(string robotName){
    auto it = robotLocation.find(robotName);
    return it->second;
};

void Context::printAll(){
    for(auto x : robotLocation){
        cout << "\n--------------------------------------------------------\n" << "Robot: " << x.first << ". Regions: " << x.second << "\n--------------------------------------------------------"<< endl;
    }
}