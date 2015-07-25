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



bool homeostasisManager::addDrive(Drive *D)
{
    // if (n>=n_drives)
    // {
    //     cout<<"aaaaaaaaaaaaaaaaaAAA"<<endl;
        
    //     n_drives +=1;
    //     drives.resize(n_drives);
    //     drive_values.resize(n_drives);
    //     drive_names.resize(n_drives);

    // }
    cout << D << endl;
    drives.push_back(D);
    n_drives=drives.size();
    
    drive_values.push_back(&(D->value));
    drive_names.push_back(&(D->name));
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


