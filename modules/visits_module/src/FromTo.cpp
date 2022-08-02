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

string FromTo::getFrom(){
    return this->from;
};

string FromTo::getTo(){
    return this->to;
};

void FromTo::setFrom(string from){
    this->from = from;
};

void FromTo::setTo(string to){
    this->to = to;
};