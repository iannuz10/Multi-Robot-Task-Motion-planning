#include <stdio.h>
#include <string>
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
            robotLocation[robotName] = location;
        };

        FromTo GetLocation(string robotName){
            return robotLocation.at(robotName);
        };
};