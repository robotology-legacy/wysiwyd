#include <string>
#include <iostream>
#include <iomanip>
#include <vector>
#include "homeostasis.h"

using namespace std;




class HomeostasisManager
{

public:

    std::vector<Drive*> drives;

    void addDrive(Drive* D);
    /*Generates a drive in the homeostasis manager. 
    Homeostasis manager will take care of updating it. 
    Input must be a drive and its position*/
    void removeDrive(int D);
    /*Removes a drive in homeostasis manager. anything
    related to this drive outside here should also be 
    removed. This is specially useful for temporal 
    needs or subgoals. */

    void sleep(int D, double time);
    bool updateDrives(double t);
    

};
