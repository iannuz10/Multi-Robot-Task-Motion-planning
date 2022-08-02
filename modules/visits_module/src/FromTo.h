#ifndef _from_to_h_
#define _from_to_h_

#include <stdio.h>
#include <string>
#include <map>
#include <ostream>
#include <iterator>

using namespace std;

class FromTo{
    private:
        string from;
        string to;
    public:
        FromTo(string from, string to);
        string getFrom();
        string getTo();

        void setFrom(string from);
        void setTo(string to);

        friend ostream& operator<<(ostream& out, const FromTo& ft){
            out << "From: " << ft.from << ", To: " << ft.to;
            return out;
        }

};

#endif