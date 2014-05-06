

#ifndef __EFAA_OPCSAVE_H__
#define __EFAA_OPCSAVE_H__

#include <wrdac/clients/opcClient.h>
#include <wrdac/knowledge/representations.h>

#include <vector>
#include <ctime>
#include <time.h>
#include <yarp/os/all.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>

//#include <vector>
//#include <list>
//#include <map>
#include <algorithm>

namespace wysiwyd{namespace wrdac{
    /**
    * \ingroup efaa_opcears
    * Inner class to opcEars; save of one OPC
    */

    class opcSave {

    public:
        std::list<Entity*>    lEntities;
        std::list<Relation>   lRelations;
        std::list<Drive>      lDrives;
        std::list< std::pair <std::string, double > > lEmotions;

        /**
        * return the OPC under one Bottle of all the element (OPC format)
        */
        yarp::os::Bottle toBottle();
        
        /**
        * TODO
        */
        void fromBottle(yarp::os::Bottle bInput);

        opcSave();
        ~opcSave();
    };
}}

#endif


//----- end-of-file --- ( next line intentionally left blank ) ------------------

