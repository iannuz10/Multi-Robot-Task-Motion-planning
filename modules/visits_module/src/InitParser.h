#ifndef _init_parser_h_
#define _init_parser_h_

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <algorithm>
#include "Context.h"
#include "FromTo.h"

using namespace std;

class InitParser
{
private:
    string locationKeyword = "robot_in";
public:
    InitParser(Context* context);
    ~InitParser();
};

#endif