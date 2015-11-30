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

void homeostasisManager::removeDrive(int D)
{
	drives.erase(drives.begin()+D);
}

void homeostasisManager::sleep(int D, double time)
{
    drives[D]->sleep(time);
}

