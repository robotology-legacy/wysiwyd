#include "homeostasisManager.h"
#include <cmath>



bool homeostasisManager::updateDrives(double t)
{
    for (unsigned int i = 0; i < drives.size(); i++)
    {
        drives[i]->update();
    }
    return true;
}



void homeostasisManager::addDrive(Drive* D)
{
    drives.push_back(D);
}


