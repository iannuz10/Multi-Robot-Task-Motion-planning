#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include "Context.cpp"
#include "FromTo.cpp"

using namespace std;


class InitParser
{
private:

public:
    InitParser();
    ~InitParser();

    bool ParseFile(fstream& fio, Context context);
};

InitParser::InitParser()
{
}

InitParser::~InitParser()
{
}

bool ParseFile(fstream& fio, Context context){
    string line;
    bool flag = true;
    fio.open("/home/iannuz/visits/visits_domain/prob1.pddl");
    while(fio){
        getline(cin, line);
        if(line.find(":init")){
            while(flag){
                getline(cin, line);
                if(line.find("robot_in")){
                    line.erase(0,1);
                    line.erase(line.length()-1,line.length());
                    int n=line.find(" ")+1;
                    line.erase(0,n);
                    string robotName = line.substr(0,line.find(" ")-1);
                    string from = line.substr(n+1,line.length());
                    FromTo initLocation(from," ");
                    context.SetLocation(robotName,initLocation);
                }else{
                    flag = false;
                    return true;
                }
            }
        }
    }
    return false;
}