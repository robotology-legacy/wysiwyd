#include <string>
#include <iostream>
#include <iomanip>
#include <vector>
#include "homeostasis.h"

using namespace std;




class homeostasisManager
{


public:
	std::vector<Drive*> drives;

    //homeostasisManager(int n);
	//bool configHomeostatic(n_drives);
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
	/*Calls the update function for each drive. */
	
	// bool setDriveParameter(int d, int p, double val)
	// {
	// 	/*Allows to change any parameter from a drive, except name. 
	// 	Name must be defined on creation. */
	// 	switch (p)
	// 	{
	// 		case 0:
	// 			drives[d]->setValue(val);
	// 			break;			
	// 		case 1:
	// 			drives[d]->setDecay(val);
	// 			break;
	// 		case 2:
	// 			drives[d]->setHomeostasisMin(val);
	// 			break;
	// 		case 3:
	// 			drives[d]->setHomeostasisMax(val);
	// 			break;
	// 		default:
	// 			cout<<"ERROR!! Could not process parameter"<< p << endl;
	// 			cout<< "Parameter options are: " <<endl;
	// 			cout <<"0: Value"<<endl;
	// 			cout <<"1: Decay"<<endl;
	// 			cout << "2: HomeostasisMin"<<endl;
	// 			cout << "3: HomeostasisMax"<<endl;
	// 			cout << endl<<" Please revise the module. "<<endl;
	// 			return false;
	// 			break;

	// 	}
	// 	return true;
	// }

	// bool deltaDriveParameter(int d, int p, double delta)
	// {
	// 	/*Allows to change any parameter from a drive, except name. 
	// 	Name must be defined on creation. */
	// 	switch (p)
	// 	{
	// 		case 0:
	// 			drives[d]->deltaValue(delta);
	// 			break;			
	// 		case 1:
	// 			drives[d]->deltaDecay(delta);
	// 			break;
	// 		case 2:
	// 			drives[d]->deltaHomeostasisMin(delta);
	// 			break;
	// 		case 3:
	// 			drives[d]->deltaHomeostasisMax(delta);
	// 			break;
	// 		default:
	// 			cout<<"ERROR!! Could not process parameter"<< p << endl;
	// 			cout<< "Parameter options are: " <<endl;
	// 			cout <<"0: Value"<<endl;
	// 			cout <<"1: Decay"<<endl;
	// 			cout << "2: HomeostasisMin"<<endl;
	// 			cout << "3: HomeostasisMax"<<endl;
	// 			cout << endl<<" Please revise the module. "<<endl;
	// 			return false;
	// 			break;

	// 	}
	// 	return true;
	// }
	


	

};
