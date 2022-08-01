#include <stdio.h>
#include <string>
#include <map>
#include <ostream>
#include <iterator>
#include "FromTo.h"

using namespace std;

FromTo::FromTo(string from, string to){
    this->from = from;
    this->to = to;
};

string FromTo::GetFrom(){
    return this->from;
};

string FromTo::GetTo(){
    return this->to;
};

void FromTo::SetFrom(string from){
    this->from = from;
};

void FromTo::SetTo(string to){
    this->to = to;
};