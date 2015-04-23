/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
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

using namespace std;
using namespace yarp::os;

Bottle autobiographicalMemory::listDataStreamProviders()
{
    Bottle bReply;

    for (std::map<string, BufferedPort<Bottle>*>::const_iterator it = mapDataStreamInput.begin(); it != mapDataStreamInput.end(); ++it)
    {
        bReply.addString(it->first);
    }

    return bReply;
}

Bottle autobiographicalMemory::addDataStreamProvider(const string &portDataStreamProvider)
{
    Bottle bReply;

    if (mapDataStreamInput.find(portDataStreamProvider) == mapDataStreamInput.end()) //key not found
    {
        mapDataStreamInput[portDataStreamProvider] = new yarp::os::BufferedPort < yarp::os::Bottle > ;

        bReply.addString("[ack]");
    }
    else { //key found
        string error = "[addDataStreamProvider] " + portDataStreamProvider + " is already present!";
        cout << error << endl;
        bReply.addString(error);
    }

    return bReply;
}

Bottle autobiographicalMemory::removeDataStreamProvider(const string &portDataStreamProvider)
{
    Bottle bReply;

    if (mapDataStreamInput.find(portDataStreamProvider) == mapDataStreamInput.end()) { //key not found
        string error = "[removeDataStreamProvider]: " + portDataStreamProvider + ") is NOT present!";
        cout << error << endl;
        bReply.addString(error);
    }
    else { //key found
        mapDataStreamInput[portDataStreamProvider]->interrupt();
        mapDataStreamInput[portDataStreamProvider]->close();
        mapDataStreamInput.erase(portDataStreamProvider);
        bReply.addString("[ack]");
    }

    return bReply;
}

Bottle autobiographicalMemory::connectDataStreamProviders()
{
    Bottle bOutput;

    if (mapDataStreamInput.size() == 0){
        bOutput.addString("[connectDataStreamProviders] ERROR: the map is NULL");
        return bOutput;
    }

    for (std::map<string, BufferedPort<Bottle>*>::const_iterator it = mapDataStreamInput.begin(); it != mapDataStreamInput.end(); ++it)
    {
        string portDataStreamReceiver = "/" + getName() + "/proprioception" + it->first + "/in";
        it->second->open(portDataStreamReceiver);
        //it->first: port name of proprioceptive data Provider
        //it->second: portname of mapDataStreamReceiver which correspond to the label of dataStreamProvider
        //cout << "  [connectDataStreamProviders] : trying to connect " << it->first << " with " <<  it->second->getName() << endl ;
        if (!Network::isConnected(it->first, it->second->getName().c_str())) {
            //cout << "Port is NOT connected : we will connect" << endl ;
            if (!Network::connect(it->first, it->second->getName().c_str())) {
                cout << "Error: Connection could not be setup" << endl;
                bOutput.addString(it->first);
            }
            //cout << "[connectDataStreamProviders] Connection from : " << it->first << endl ;
            //cout << "[connectDataStreamProviders] Connection to   : " << it->second->getName() << endl;
        }
        else {
            //cout << "[connectDataStreamProviders] Error: Connection already present!" << endl ;
        }
    }

    if (bOutput.size() == 0){
        bOutput.addString("ack");
    }

    return bOutput;
}

Bottle autobiographicalMemory::disconnectDataStreamProviders()
{
    Bottle bOutput;
    bool isAllDisconnected = true;

    if (mapDataStreamInput.size() == 0){
        bOutput.addString("[disconnectDataStreamProviders] ERROR the map is NULL");
        return bOutput;
    }

    for (std::map<string, BufferedPort<Bottle>*>::const_iterator it = mapDataStreamInput.begin(); it != mapDataStreamInput.end(); ++it)
    {
        //it->first: port name of stream data Provider
        //it->second: port name of streamDataReceiver which correspond to the label of streamDataProvider
        Network::disconnect(it->first, it->second->getName().c_str());
        if (Network::isConnected(it->first, it->second->getName().c_str())) {
            cout << "[disconnectDataStreamProviders] ERROR " << it->first << " is NOT disconnected!";
            bOutput.addString(it->first);
            isAllDisconnected = false;
        }
        else {
            //cout << "[disconnectDataStreamProviders] " << it->first << " successfully disconnected!"  << endl ;

            //Have to close/interrupt each time otherwise the port is not responsive anymore
            it->second->interrupt();
            it->second->close();
        }
    }

    if (isAllDisconnected == true){
        bOutput.addString("ack");
    }

    //cout << "[disconnectDataStreamProviders] bOutput = {" << bOutput.toString().c_str() << "}" << endl ;
    return bOutput;
}

bool autobiographicalMemory::storeDataStreamAllProviders(const string &synchroTime) {
    bool doInsert = false;
    ostringstream osArg;

    osArg << "INSERT INTO proprioceptivedata(instance, subtype, frame_number, time, label_port, value) VALUES ";

    for (std::map<string, BufferedPort<Bottle>*>::const_iterator it = mapDataStreamInput.begin(); it != mapDataStreamInput.end(); ++it)
    {
        Bottle* lastReading = it->second->read(false); // (false) such that we do not wait until data arrives at port

        if(lastReading!=NULL) { // only proceed if we got something
            for(int subtype = 0; subtype < lastReading->size(); subtype++) {
                // go ahead if it is NOT a port related to skin OR it is a skin port and the value is bigger than 5.0
                if(it->first.find("skin") == std::string::npos || lastReading->get(subtype).asDouble() > 5.0) {
                    doInsert = true;

                    osArg << "(" << imgInstance << ", '" << subtype << "', '" << frameNb << "', '" << synchroTime << "', '" << it->first << "', '" << lastReading->get(subtype).asDouble() << "' ),";
                }
            }
        }
    }

    if(doInsert) {
        string strRequest = osArg.str(); // needed so we can cut off last character below, which is a ','
        if(processInsertDelayed) {
            requestInsertPushToQueue(strRequest.substr(0, strRequest.size() -1));
        } else {
            Bottle bRequest;
            bRequest.addString("request");
            bRequest.addString(strRequest.substr(0, strRequest.size() -1));
            request(bRequest);
        }
    }

    return true;
}

// From here on all send stream related
// if useRealiCub == true, the streams are sent to /icub/...
// if useRealiCub == false, the streams are sent to /icubSim/...
int autobiographicalMemory::openDataStreamPorts(int instance, bool useRealiCub) {
    Bottle bDistLabelPort;
    ostringstream osArg;

    bDistLabelPort.addString("request");
    osArg << "SELECT DISTINCT label_port FROM proprioceptivedata WHERE instance = " << instance << endl;
    bDistLabelPort.addString(osArg.str());
    bDistLabelPort = request(bDistLabelPort);

    for (int i = 0; i < bDistLabelPort.size() && bDistLabelPort.toString()!="NULL"; i++) {
        string dataStreamPortFrom = bDistLabelPort.get(i).asList()->get(0).asString();
        mapDataStreamPortOut[dataStreamPortFrom] = new yarp::os::BufferedPort < Bottle >;
        mapDataStreamPortOut[dataStreamPortFrom]->open((portPrefixForStreaming+dataStreamPortFrom).c_str());

        // in case of position commands, replace state:o with rpc:i; otherwise do nothing
        string toReplace="state:o";
        string dataStreamPortTo = dataStreamPortFrom;
        if(dataStreamPortTo.find(toReplace)!=string::npos) {
            dataStreamPortTo.replace(dataStreamPortTo.find(toReplace), toReplace.length(), "rpc:i");
        }

        // if memory was made not on same robot as replay should be made
        string robotToReplace;
        string robotReplacement;
        if(useRealiCub) {
            robotToReplace="/icubSim/";
            robotReplacement="/icub/";
        } else {
            robotToReplace="/icub/";
            robotReplacement="/icubSim/";
        }
        if(dataStreamPortTo.find(robotToReplace)!=string::npos) {
            dataStreamPortTo.replace(dataStreamPortTo.find(robotToReplace), robotToReplace.length(), robotReplacement);
        }

        Network::connect(portPrefixForStreaming+dataStreamPortFrom, dataStreamPortTo);
        if(Network::isConnected(portPrefixForStreaming+dataStreamPortFrom, dataStreamPortTo)) {
            cout << "Successfully connected " << portPrefixForStreaming+dataStreamPortFrom << " and " << dataStreamPortTo << endl;
        } else {
            cout << "NOT connected " << portPrefixForStreaming+dataStreamPortFrom << " and " << dataStreamPortTo << endl;
        }
    }

    cout << "[openDataStreamPorts] Just created " << mapDataStreamPortOut.size() << " ports." << endl;

    return mapDataStreamPortOut.size();
}

unsigned int autobiographicalMemory::getStreamDataProviderCount(int instance) {
    Bottle bRequest;
    ostringstream osArg;

    bRequest.addString("request");
    osArg << "SELECT DISTINCT label_port, subtype FROM proprioceptivedata WHERE instance = " << instance;
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);
    if(bRequest.size()>0 && bRequest.toString()!="NULL") {
        return bRequest.size();
    } else {
        return 0;
    }
}

long autobiographicalMemory::getTimeLastDataStream(int instance) {
    Bottle bRequest;
    ostringstream osArg;

    osArg << "SELECT CAST(EXTRACT(EPOCH FROM time-(SELECT time FROM proprioceptivedata WHERE instance = " << instance << " ORDER BY time LIMIT 1)) * 1000000 as INT) as time_difference FROM proprioceptivedata WHERE instance = " << instance << " ORDER BY time DESC LIMIT 1;";
    bRequest.addString("request");
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);
    if(bRequest.size()>0 && bRequest.toString()!="NULL") {
        return atol(bRequest.get(0).asList()->get(0).asString().c_str());
    } else {
        return 0;
    }
}

Bottle autobiographicalMemory::getStreamDataWithinEpoch(long updateTimeDifference, string port) {
    Bottle bListDataStream;
    bListDataStream.addString("request");
    ostringstream osArgDataStream;

    osArgDataStream << "SELECT * FROM (";
    osArgDataStream << "SELECT subtype, label_port, time, value, ";
    osArgDataStream << "CAST(EXTRACT(EPOCH FROM time-(SELECT time FROM proprioceptivedata WHERE instance = '" << imgInstance << "' ORDER BY time LIMIT 1)) * 1000000 as INT) as time_difference ";
    osArgDataStream << "FROM proprioceptivedata WHERE instance = '" << imgInstance << "' ORDER BY time) s WHERE ";

    if(port!="") {
        osArgDataStream << "label_port = '" << port << "' AND ";
    }

    if(realtimePlayback) {
        osArgDataStream << "time_difference <= " << updateTimeDifference << " and time_difference > " << timeLastImageSent << " ORDER BY time DESC, label_port, subtype::int ASC ";
    } else {
        osArgDataStream << "time_difference > " << timeLastImageSent << " ORDER BY time DESC, label_port, subtype::int ASC ";
    }

    if(port!="") {
        osArgDataStream << "LIMIT (SELECT COUNT(DISTINCT subtype) FROM proprioceptivedata WHERE instance = '" << imgInstance << "' AND label_port='" << port << "')";
    } else {
        osArgDataStream << "LIMIT " << streamDataProviderCount << ";";
    }

    bListDataStream.addString(osArgDataStream.str());
    return request(bListDataStream);
}
