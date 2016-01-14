#include "homeostasisManager.h"
#include <cmath>

bool HomeostasisManager::updateDrives(double t)
{
    for (unsigned int i = 0; i < drives.size(); i++)
    {
        drives[i]->update();
    }
    return true;
}

void HomeostasisManager::addDrive(Drive* D)
{
    drives.push_back(D);
}

void HomeostasisManager::removeDrive(int D)
{
    drives.erase(drives.begin()+D);
}

void HomeostasisManager::sleep(int D, double time)
{
    drives[D]->sleep(time);
}

