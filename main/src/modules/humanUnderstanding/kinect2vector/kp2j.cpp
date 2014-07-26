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
	options.put("verbosity", 0);
	options.put("noRPC", 1); //To work with the datasetplayer only

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


Vector getRelativeJointPosition(string jntA, string jntReference, map<string, Joint> jnts)
{
	Vector p(3);
	p[0] = jnts[jntA].x - jnts[jntReference].x;
	p[1] = jnts[jntA].y - jnts[jntReference].y;
	p[2] = jnts[jntA].z - jnts[jntReference].z;
	return p;
}

void addToBottle(const Vector &v, Bottle& b)
{
	for (int i = 0; i < v.size(); i++)
		b.addDouble(v[i]);
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
        
        Bottle& bottle = portVector.prepare();
        bottle.clear();
		string USED_REFERENCE = KINECT_TAGS_BODYPART_SHOULDER_C;
		addToBottle(getRelativeJointPosition(KINECT_TAGS_BODYPART_HEAD, USED_REFERENCE, closestPlayer.skeleton), bottle);
		addToBottle(getRelativeJointPosition(KINECT_TAGS_BODYPART_SHOULDER_L, USED_REFERENCE, closestPlayer.skeleton), bottle);
		addToBottle(getRelativeJointPosition(KINECT_TAGS_BODYPART_SHOULDER_R, USED_REFERENCE, closestPlayer.skeleton), bottle);
		addToBottle(getRelativeJointPosition(KINECT_TAGS_BODYPART_ELBOW_L, USED_REFERENCE, closestPlayer.skeleton), bottle);
		addToBottle(getRelativeJointPosition(KINECT_TAGS_BODYPART_ELBOW_R, USED_REFERENCE, closestPlayer.skeleton), bottle);
		addToBottle(getRelativeJointPosition(KINECT_TAGS_BODYPART_WRIST_L, USED_REFERENCE, closestPlayer.skeleton), bottle);
		addToBottle(getRelativeJointPosition(KINECT_TAGS_BODYPART_WRIST_R, USED_REFERENCE, closestPlayer.skeleton), bottle);
		addToBottle(getRelativeJointPosition(KINECT_TAGS_BODYPART_HAND_L, USED_REFERENCE, closestPlayer.skeleton), bottle);
		addToBottle(getRelativeJointPosition(KINECT_TAGS_BODYPART_HAND_R, USED_REFERENCE, closestPlayer.skeleton), bottle);
		
		//Wend as a vector
		portVector.write();
		cout << "Skeleton vector is : " << bottle.toString() << endl;
    }
    else
    {
        cout<<"."<<endl;
    }
        
    return true;
}
