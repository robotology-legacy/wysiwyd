/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer, Grégoire Pointeau, Maxime Petit
 * email:   t.fischer@imperial.ac.uk, greg.pointeau@gmail.com, m.petit@imperial.ac.uk
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * wysiwyd/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include "autobiographicalMemory.h"

#ifdef WIN32
#include <windows.h>
    #if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
        #define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
    #else
        #define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
    #endif
#else
#include <sys/time.h>
#endif

using namespace std;
using namespace yarp::os;
using namespace wysiwyd::wrdac;

/* transforms an result of query to a bottle */
Bottle autobiographicalMemory::restoBottle(ResultSet bResult)
{
    vector<string> vRow;

    //catch each row of the ResultSet
    Bottle bOutput;
    bool isResult = false;
    //output.addString("request");
    while (bResult.fetch(vRow))
    {
        isResult = true;
        Bottle bLineBottle;
        //parse the current row of the result and add each column values to a bottle
        for (size_t i = 0; i < vRow.size(); i++)
        {
            bLineBottle.addString(vRow[i].c_str());
        }

        //add the line bottle to the reply bottle
        //printf("Bottle lineBottle done : %s\n", lineBottle.toString().c_str());
        //cout << "lineBottle size : " << lineBottle.size() << endl;
        bOutput.addList() = bLineBottle;
    }
    if (!isResult)
    {
        bOutput.clear();
        bOutput.addString("NULL");
    }
    //cout << "output size : " << output.size() << endl;
    return bOutput;
}

/* Send a query to the DB. bRequest must be the complete request */
Bottle autobiographicalMemory::request(Bottle bRequest)
{
    database_mutex.lock();
    //prepare the ResultSet of the query and the reply
    ResultSet rs1;
    Bottle bReply;
    //cout << "Request : "<< bRequest.get(1).asString().c_str() << endl;

    //send the request to the database
    try
    {
        //verbose debug
        //cout << "Request : "<< bRequest.get(1).asString().c_str() << endl;

        *ABMDataBase << bRequest.get(1).asString().c_str(), rs1;
        bReply = restoBottle(rs1);

        //verbose print reply
        //cout << "bReply = " << bReply.toString().c_str() << endl ;
    }
    catch (DataBaseError& e)
    {
        if(strcmp(e.what(),"This command don't support results")!=0)
            cerr << "Exception during request: " << e.what() << endl;
        string sExcept = "Exception during request: "; sExcept += e.what();
        bReply.addString(sExcept.c_str());
    }
    database_mutex.unlock();

    return bReply;
}

Bottle autobiographicalMemory::requestFromString(string sInput)
{
    //send the SQL query within a bottle to autobiographicalMemory
    Bottle bQuery;
    bQuery.addString("request");
    bQuery.addString(sInput.c_str());
    return request(bQuery);
}

/* each interaction stored in the DB is save in a file text */
void autobiographicalMemory::writeInsert(string sRequest)
{
    ofstream file(savefile.c_str(), ios::out | ios::trunc);
    if (file) {
        file << sRequest << endl;
    }
    else {
        cout << "Error, can not save request in " << savefile << endl;
        return;
    }
    file.close();
}

/* read interactions from a file text */
bool autobiographicalMemory::readInsert()
{
    ifstream file(savefile.c_str(), ios::in);
    if (file) {
        cout << endl << "readFile of requests from " << savefile.c_str() << endl << endl;
    }
    else {
        cout << "Error, can not open " << savefile.c_str() << endl;
        return false;
    }

    string line;
    Bottle bRequest;
    // for each line of the file:
    while (getline(file, line))
    {
        bRequest.clear();
        bRequest.addString("request");
        bRequest.addString(line.c_str());
        bRequest = request(bRequest);
    }
    file.close();

    return true;
}

Bottle autobiographicalMemory::connectOPC(Bottle bInput)
{
    Bottle bOutput;

    if (bInput.size() != 2)
    {
        bOutput.addString("Error in connect, wrong number of input");
    }

    string OPC_name = s_real_OPC;
    if (!bInput.get(1).isString())
    {
        bOutput.addString("Error in connect, wrong format of input");
    }
    else
    {
        OPC_name = bInput.get(1).toString();
    }

    opcWorld = new OPCClient(getName().c_str());
    int iTry = 0;
    while (!opcWorld->isConnected())
    {
        cout << "ABM Connecting to " << OPC_name << "..." << opcWorld->connect(OPC_name) << endl;
        Time::delay(0.5);
        iTry++;
        if (iTry > 2)
        {
            bOutput.addString("Connection failed, please check your port");
            return bOutput;
        }
    }
    // opcWorld->checkout();
    // opcWorld->update();
    bOutput.addString("Connection done");
    return bOutput;
}

long autobiographicalMemory::getCurrentTimeInMS()
{
#ifdef WIN32
    FILETIME ft;
    unsigned __int64 tmpres = 0;

    GetSystemTimeAsFileTime(&ft);

    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;

    /*converting file time to unix epoch*/
    tmpres -= DELTA_EPOCH_IN_MICROSECS;
    tmpres /= 10;  /*convert into microseconds*/
    long iS = (long)(tmpres / 1000000UL);
    iS = iS * 1000000;
    long iUS = (long)(tmpres % 1000000UL);

    return iS + iUS;
#else
    struct timezone tz;
    struct timeval tv;
    gettimeofday(&tv, &tz);
    long iS = tv.tv_sec * 1000000;
    long iUS = tv.tv_usec;
    return iS + iUS;
#endif
}

string autobiographicalMemory::getCurrentTime()
{
    struct tm Time;
    time_t myTime;
    time(&myTime);                    // get unix time
    tm *t = localtime(&myTime);        // conversion in local time

#ifdef WIN32
    SYSTEMTIME st;
    GetSystemTime(&st);
    int iUS = st.wMilliseconds * 1000;
#else
    struct timezone tz;
    struct timeval tv;
    gettimeofday(&tv, &tz);
    int iUS = tv.tv_usec;
#endif

    // Sorry, this is a terrible hack to obtain a six digit microsecond string
    stringstream iUSTemp, iUSZero;
    iUSTemp << iUS;
    int zerosNeeded = 6 - iUSTemp.str().length();

    while (zerosNeeded) {
        iUSZero << '0';
        zerosNeeded--;
    }

    iUSZero << iUS;

    Time.tm_hour = (*t).tm_hour;
    Time.tm_min = (*t).tm_min;
    Time.tm_sec = (*t).tm_sec;
    Time.tm_mday = (*t).tm_mday;
    Time.tm_mon = (*t).tm_mon;
    Time.tm_year = (*t).tm_year;
    Time.tm_mday = (*t).tm_mday;
    Time.tm_mon = (*t).tm_mon;
    Time.tm_year = (*t).tm_year;

    ostringstream osTime;

    // get information
    int iHH, iMM, iSS, iMonth, iDay, iYear;
    iHH = Time.tm_hour;
    iMM = Time.tm_min;
    iSS = Time.tm_sec;
    iMonth = Time.tm_mon + 1;
    iDay = Time.tm_mday;
    iYear = Time.tm_year + 1900;
    osTime << iYear << "-" << iMonth << "-" << iDay << " " << iHH << ":" << iMM << ":" << iSS << "." << iUSZero.str();

    return osTime.str();
}

Bottle autobiographicalMemory::connect2reasoning()
{
    string name_abm2reasoning = "/" + getName() + "/to_reasoning";

    Network::connect(name_abm2reasoning.c_str(), "/abmReasoning/rpc");

    string state;
    isconnected2reasoning = Network::isConnected(name_abm2reasoning.c_str(), "/abmReasoning/rpc");
    (isconnected2reasoning ? state = "ABM is now connected to abmReasoning" : state = "ABM failed to connect to abmReasoning");

    Bottle bOutput;
    bOutput.addString(state.c_str());

    return bOutput;
}

Bottle autobiographicalMemory::detectFailed()
{
    return requestFromString("SELECT instance FROM main WHERE activitytype = 'actionfailed' AND begin = true");
}

/*
* return a tuple of 3 int from a string input
*/
vector<int> autobiographicalMemory::tupleIntFromString(string sInput)
{
    vector<int> tOutput;
    char *cInput;
    cInput = (char*)sInput.c_str();
    int iLevel = 0;
    unsigned int data = 0;
    string sX = "", sY = "", sZ = "";
    while (cInput[data] != '\0')
    {
        char cTemp = cInput[data];
        if (cTemp == ',')
        {
            iLevel++;
        }
        if (cTemp != '{' && cTemp != ',')
        {
            switch (iLevel)
            {
            case 0:
                sX += cTemp;
                break;
            case 1:
                sY += cTemp;
                break;
            case 2:
                sZ += cTemp;
                break;
            }
        }
        data++;
    }

    tOutput.push_back(atoi(sX.c_str()));
    tOutput.push_back(atoi(sY.c_str()));
    tOutput.push_back(atoi(sZ.c_str()));
    //get<0>(tOutput) = atoi(sX.c_str());
    //get<1>(tOutput) = atoi(sY.c_str());
    //get<2>(tOutput) = atoi(sZ.c_str());

    return tOutput;
}

/*
* return a tuple of 3 double from a string input
*/
vector<double> autobiographicalMemory::tupleDoubleFromString(string sInput)
{
    vector<double> tOutput;
    char *cInput;
    cInput = (char*)sInput.c_str();
    int iLevel = 0;
    unsigned int data = 0;
    string sX = "", sY = "", sZ = "";
    while (cInput[data] != '\0')
    {
        char cTemp = cInput[data];
        if (cTemp == ',')
        {
            iLevel++;
        }
        if (cTemp != '{' && cTemp != ',')
        {
            switch (iLevel)
            {
            case 0:
                sX += cTemp;
                break;
            case 1:
                sY += cTemp;
            case 2:
                sZ += cTemp;
                break;
            }
        }
        data++;
    }

    tOutput.push_back(atof(sX.c_str()));
    tOutput.push_back(atof(sY.c_str()));
    tOutput.push_back(atof(sZ.c_str()));
    //get<0>(tOutput) = atof(sX.c_str());
    //get<1>(tOutput) = atof(sY.c_str());
    //get<2>(tOutput) = atof(sZ.c_str());

    return tOutput;
}

/*Bottle autobiographicalMemory::getInfoAbout(string sName)
{
    Bottle bMessenger, bOutput;

    if (opcWorld->getEntity(sName) != NULL)
    {
        // The object is not in the OPC, we have to search in the memory to get the type.
        ostringstream osEntity;
        osEntity << "SELECT instance, opcid FROM entity WHERE name = '" << sName << "' ORDER BY instance DESC LIMIT 1";
        bMessenger = requestFromString(osEntity.str());

        cout << "bMessenger : " << bMessenger.toString() << endl;

        int Instance = atoi(bMessenger.get(0).asList()->get(0).toString().c_str()),
            Opcid = atoi(bMessenger.get(0).asList()->get(1).toString().c_str());

        osEntity.str("");
        osEntity << "SELECT subtype FROM contentopc WHERE instance = " << Instance << " AND opcid = " << Opcid;
        bMessenger = requestFromString(osEntity.str());

        string sSubType = bMessenger.get(0).asList()->get(0).toString();

        osEntity.str("");
        osEntity << "SELECT count(*) FROM contentarg WHERE argument = '" << sName << "'";
        bMessenger = requestFromString(osEntity.str());

        int iNbInteraction = atoi(bMessenger.get(0).asList()->get(0).toString().c_str());
        cout << "I have interacted with this " << sSubType << " " << iNbInteraction / 2 << " times ! " << endl;
    }

    return bOutput;
}*/
