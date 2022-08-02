#ifndef _context_
#define _context_

#include <stdio.h>
#include <string>
#include <iostream>
#include "FromTo.h"
#include <unordered_map>
#include <list>

using namespace std;

class Context{
    private: 
        std::unordered_map<string, FromTo> robotLocation;
    public:
        Context();

        // Add or change entry
        void setLocation(string robotName, FromTo location);

        // Get location info of a given robot
        FromTo getLocation(string robotName);

        // Print all context's content
        void printAll();
};
#endif