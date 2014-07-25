#include "kp2j.h"

bool KP2JA::configure(ResourceFinder &rf)
{
    string name=rf.check("name",Value("kp2ja")).asString().c_str();
    string clientName = name;

    portVector.open( ("/" + clientName + "/state:o").c_str() );

    clientName += "/kinect";

    Property options;
    options.put("carrier","tcp");
    options.put("remote","kinectServer");
    options.put("local",clientName.c_str());
    options.put("verbosity",0);

    if (!client.open(options))
        return false;

    return true;
}


bool KP2JA::close()
{
    portVector.interrupt();
    portVector.close();
    client.close();
    return true;
}


double KP2JA::getPeriod()
{
    return 0.01;
}


bool KP2JA::updateModule()
{    
    bool isRefreshed = client.getPlayers(players);
    if (!isRefreshed)
        return true;

    bool tracked = client.getJoints(closestPlayer, KINECT_TAGS_CLOSEST_PLAYER);
    if (tracked)
    {
        cout<<"Tracked..."<<endl;
        /*
        Bottle& vect = portVector.prepare();
        vect.clear();

        vect.addDouble( 
        for(map<string,Joint>::iterator jnt = joint->skeleton.begin() ; jnt != joint->skeleton.end() ; jnt++)
        {
            Vector kPosition(4);
            kPosition[0] = jnt->second.x;
            kPosition[1] = jnt->second.y;
            kPosition[2] = jnt->second.z;
            kPosition[3] = 1;

            if (jnt->first == EFAA_OPC_BODY_PART_TYPE_HEAD)
            {
                partner->m_ego_position = irPos;
            }
            partner->m_body.m_parts[jnt->first] = irPos;
        }*/
    }
    else
    {
        cout<<"."<<endl;
    }
        
    return true;
}
