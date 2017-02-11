#include <wrdac/clients/opcSave.h>
#include <algorithm>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wysiwyd::wrdac;

opcSave::opcSave()
{

}


opcSave::~opcSave()
{
    lEntities.clear();
}

Bottle opcSave::toBottle()
{
    Bottle bOutput, bTemp, bEntity, bRelation, bDrives;
    
    for (auto it = lEntities.begin(); it != lEntities.end(); it++)
    {
        bEntity.addList() = (*it)->asBottle();
    }

    for (list<Relation>::iterator it = lRelations.begin(); it != lRelations.end(); it++)
    {
        bRelation.addList() = it->asBottle();
    }

    for (list<Drive>::iterator it = lDrives.begin(); it != lDrives.end(); it++)
    {
        bDrives.addList() = it->asBottle();
    }


    bOutput.addList() = bEntity;
    bOutput.addList() = bRelation;
    bOutput.addList() = bDrives;

    return bOutput;
}

