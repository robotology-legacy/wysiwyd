#include "homeostasisManager.h"
#include <cmath>

homeostasisManager::homeostasisManager(int n)
    {
        drives.resize(n);
        n_drives = n;
    }

bool homeostasisManager::updateDrives(double t)
{
    for (unsigned int i = 0; i < n_drives; i++)
    {
        drives[i]->update();
    }
    return true;
}



bool homeostasisManager::addDrive(Drive *D,unsigned int n)
{
    cout<<2<<endl;
    if (n>=n_drives)
    {
        
        n_drives +=1;
        drives.resize(n_drives);
        drive_values.resize(n_drives);
        drive_names.resize(n_drives);

    }
    drives[n] = D;
    
    drive_values[n] = &(D->value);
    drive_names[n] = &(D->name);
    return true;
}

vector<double*> homeostasisManager::getDriveStatus(int n = -1)
{   
    if (n==-1)
    {
        return drive_values;
    }else{
        vector<double*> val(0);
        val[0] = drive_values[n];
        return val;
    }
}


