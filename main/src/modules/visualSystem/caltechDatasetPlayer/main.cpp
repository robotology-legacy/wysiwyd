/*
* Copyright(C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT - 612139
* Authors: Stephane Lallee
* email : stephane.lallee@gmail.com
* Permission is granted to copy, distribute, and / or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* wysiwyd / license / gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU General
* Public License for more details
*/

#include <iostream>
#if defined(WIN32)
    #pragma warning (disable : 4099)
    #pragma warning (disable : 4250)
    #pragma warning (disable : 4520)
#endif

#if defined(WIN32)
    #include "dirent.h"
    #undef max                  /*conflict with pango lib coverage.h*/
    #include <direct.h>
    #define GetCurrentDir _getcwd
#else
    #include <unistd.h>
    #include <dirent.h>
    #include <cerrno>
    #define GetCurrentDir getcwd
#endif

//#include "dirent.h"
//#undef max                  /*conflict with pango lib coverage.h*/
//#include <direct.h>
//#define GetCurrentDir _getcwd
//#include <stdio.h>              /* defines FILENAME_MAX */

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::file;
using namespace std;

class Player : public RFModule
{
    BufferedPort<ImageOf<PixelRgb> > portImg;
    Port portCategoryTag;
    Port portCategoryVector;
    map< string, vector< IplImage* > > buffer;
	double period;

public:

    bool configure(yarp::os::ResourceFinder &rf)
    {
		period = rf.check("period", Value(1.5)).asDouble();
        string name = rf.check("name", Value("caltechDatasetPlayer")).asString();
        string dsPath = rf.check("dataPath", Value(rf.findPath("default.ini").c_str())).asString();
        dsPath.erase(dsPath.end() - 11, dsPath.end());
        setName(name.c_str());

        //Load the dataset
        std::cout << "Looking for dataset in : " << dsPath << std::endl;
        string currentPath = dsPath;
        DIR *dirp = NULL;
        struct dirent *direntp = NULL;

        /* Check if file is opened */
        if ((dirp = opendir(currentPath.c_str())) == NULL)
        {
            fprintf(stdout, "Error( %d opening %s\n", errno, currentPath.c_str());
            return false;
        }

        while ((direntp = readdir(dirp)) != NULL)
        {
            /* Ignore special directories. */
            if ((strcmp(direntp->d_name, ".") == 0) ||
                (strcmp(direntp->d_name, "..") == 0))
                continue;

            string category = direntp->d_name;
            string fullPath = currentPath + "\\" + category;
            cout << "Category : " << category << endl;
            //if (
            //  category != "brain" &&
            //  category != "dolphin" &&
            //  category != "crab" &&
            //  category != "barrel" &&
            //  category != "faces"
            //  )continue;

            DIR *dirCat = opendir(fullPath.c_str());
            struct dirent *direntFile = NULL;
            int cntCategory = 0;
            while ((direntFile = readdir(dirCat)))
            {
                /* Ignore special directories. */
                if ((strcmp(direntFile->d_name, ".") == 0) ||
                    (strcmp(direntFile->d_name, "..") == 0))
                    continue;


                cout << ".";
                string fileName = direntFile->d_name;
                string fileNamePath = fullPath + "\\" + fileName;

                IplImage* pImg;
                IplImage* tmpImg = cvLoadImage(fileNamePath.c_str(), CV_LOAD_IMAGE_UNCHANGED);
                pImg = cvCreateImage(cvSize(300, 200), tmpImg->depth, tmpImg->nChannels);
                cvResize(tmpImg, pImg);
                buffer[category].push_back(pImg);
                cvReleaseImage(&tmpImg);
                cntCategory++;
            }
			cout << endl<< "Category : " << category << " has " << cntCategory << " items" << endl;
        }
        cout << "Total items : " << buffer.size() << endl;

        portImg.open("/" + name + "/image:o");
        portCategoryTag.open("/" + name + "/categoryTag:o");
        portCategoryVector.open("/" + name + "/categoryVector:o");

        return true;
    }

    bool updateModule()
    {       
        //Pick a random category
        int countIt = yarp::os::Random::uniform(0, buffer.size() - 1);
        map<string, vector< IplImage*> >::iterator it = buffer.begin();
        for (int i = 0; i < countIt; i++)
        {
            it++;
        }

        //Pick rnd image
        int rndImg = yarp::os::Random::uniform(0, it->second.size() - 1);

        //Send
        ImageOf<PixelRgb> &img = portImg.prepare();
        img.wrapIplImage(it->second[rndImg]);
        portImg.write();

        //Category tag
        Bottle botTag;
        //bot.addString(it->first);
        botTag.addString(it->first);
        portCategoryTag.write(botTag);

        Bottle botVect;
        for (int i = 0; i < buffer.size(); i++)
            botVect.addInt( countIt==i );
        portCategoryVector.write(botVect);
        return true;
    }

    bool close()
    {       
        //close ports
        portImg.close();
        portCategoryTag.close();
        portCategoryVector.close();

        //Release images
        for (map<string, vector< IplImage*> >::iterator it = buffer.begin(); it != buffer.end(); it++)
            for (int i = 0; i < it->second.size(); i++)
                cvReleaseImage(&it->second[i]);

        return true;
    }

	double getPeriod()
	{
		return period;
	}
};


int main(int argc, char *argv[]) 
{
    
    Network yarp;
    if (!Network::checkNetwork())
    {
        cout << "yarp network is not available!" << endl;
        return 0;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("caltechDataset");
    rf.setDefaultConfigFile("default.ini"); //overridden by --from parameter
    rf.configure(argc, argv);

    Player mod;
    mod.configure(rf);
    return mod.runModule();;
}
