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
    string locationKeyword = "robot_in";
    bool success = false;
public:
    InitParser(Context* context);
    ~InitParser();

    bool IsSuccessful(){
        return success;
    }

};

InitParser::InitParser(Context* context){
    int curr;
    string line;
    string fileName = "/home/iannuz/visits/visits_domain/prob1.pddl";
    ifstream fio(fileName.c_str());
    cout << "Opening file\n";
    if (fio.is_open()){
        cout << "File is open\n";
        while(getline(fio,line)){
            if(line.find(locationKeyword) != std::string::npos){
                curr=line.find(locationKeyword);
                line.erase(line.length()-1,line.length());
                line.erase(0,curr+locationKeyword.length()+1);
                
                string robotName = line.substr(0,line.find(" ")).c_str();
                string from = line.substr(line.find(" ")+1,line.length()-1).c_str();
                cout << "Robot name: " << robotName << endl;
                cout << "From region: " << from << endl;
                FromTo initLocation(from," ");
                context->SetLocation(robotName,initLocation);
            }
        }
        success=true;
    }
}

InitParser::~InitParser()
{
}


