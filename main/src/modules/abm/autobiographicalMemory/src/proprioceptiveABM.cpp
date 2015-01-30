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

Bottle autobiographicalMemory::addDataStreamProvider(const string &type, const string &portDataStreamProvider)
{
    Bottle bReply;

    if (mapDataStreamProvider.find(type) == mapDataStreamProvider.end()) //key not found
    {
        mapDataStreamProvider[type] = portDataStreamProvider;
        mapDataStreamReceiver[type] = new yarp::os::BufferedPort < yarp::os::Bottle > ;

        bReply.addString("[ack]");
    }
    else { //key found
        string error = "[addDataStreamProvider] " + type + " is already present!";
        cout << error << endl;
        bReply.addString(error);
    }

    return bReply;
}

Bottle autobiographicalMemory::removeDataStreamProvider(const string &label)
{
    Bottle bReply;

    if (mapDataStreamProvider.find(label) == mapDataStreamProvider.end()) { //key not found
        string error = "[removeDataStreamProvider]: " + label + ") is NOT present!";
        cout << error << endl;
        bReply.addString(error);
    }
    else { //key found
        mapDataStreamProvider.erase(label);
        mapDataStreamReceiver[label]->interrupt();
        mapDataStreamReceiver[label]->close();
        mapDataStreamReceiver.erase(label);
        bReply.addString("[ack]");
    }

    return bReply;
}

Bottle autobiographicalMemory::connectDataStreamProviders()
{
    Bottle bOutput;

    if (mapDataStreamProvider.size() == 0){
        bOutput.addString("[connectDataStreamProviders] ERROR: the map is NULL");
        return bOutput;
    }

    for (std::map<string, string>::const_iterator it = mapDataStreamProvider.begin(); it != mapDataStreamProvider.end(); ++it)
    {
        string portDataStreamReceiver = "/" + getName() + "/proprioception/" + it->first + "/in";
        mapDataStreamReceiver.find(it->first)->second->open(portDataStreamReceiver);
        //it->second: port name of proprioceptive data Provider
        //mapDataStreamReceiver.find(it->first)->second: portname of mapDataStreamReceiver which correspond to the label of dataStreamProvider
        //cout << "  [connectDataStreamProviders] : trying to connect " << it->second << " with " <<  mapDataStreamReceiver.find(it->first)->second->getName().c_str() << endl ;
        if (!Network::isConnected(it->second, mapDataStreamReceiver.find(it->first)->second->getName().c_str())) {
            //cout << "Port is NOT connected : we will connect" << endl ;
            if (!Network::connect(it->second, mapDataStreamReceiver.find(it->first)->second->getName().c_str())) {
                cout << "Error: Connection could not be setup" << endl;
                bOutput.addString(it->second);
            }
            //cout << "[connectDataStreamProviders] Connection from : " << it->second << endl ;
            //cout << "[connectDataStreamProviders] Connection to   : " << mapDataStreamReceiver.find(it->first)->second->getName().c_str() << endl;
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

    if (mapDataStreamProvider.size() == 0){
        bOutput.addString("[disconnectDataStreamProviders] ERROR the map is NULL");
        return bOutput;
    }

    for (std::map<string, string>::const_iterator it = mapDataStreamProvider.begin(); it != mapDataStreamProvider.end(); ++it)
    {
        //it->second: port name of stream data Provider
        //mapDataStreamReceiver.find(it->first)->second: port name of streamDataReceiver which correspond to the label of streamDataProvider
        Network::disconnect(it->second, mapDataStreamReceiver.find(it->first)->second->getName().c_str());
        if (Network::isConnected(it->second, mapDataStreamReceiver.find(it->first)->second->getName().c_str())) {
            cout << "[disconnectDataStreamProviders] ERROR " << it->second << " is NOT disconnected!";
            bOutput.addString(it->second);
            isAllDisconnected = false;
        }
        else {
            //cout << "[disconnectDataStreamProviders] " << it->second << " successfully disconnected!"  << endl ;

            //Have to close/interrupt each time otherwise the port is not responsive anymore
            mapDataStreamReceiver.find(it->first)->second->interrupt();
            mapDataStreamReceiver.find(it->first)->second->close();
        }
    }

    if (isAllDisconnected == true){
        bOutput.addString("ack");
    }

    //cout << "[disconnectDataStreamProviders] bOutput = {" << bOutput.toString().c_str() << "}" << endl ;
    return bOutput;
}

bool autobiographicalMemory::storeDataStream(int instance, int frame_number, const string &type, int subtype, const string &dataStreamTime, const string &dataStreamPort, double value)
{
    Bottle bRequest;
    ostringstream osArg;

    bRequest.addString("request");
    osArg << "INSERT INTO proprioceptivedata(instance, type, subtype, frame_number, time, label_port, value) VALUES (" << instance << ", '" << type << "', '" << subtype << "', '" << frame_number << "', '" << dataStreamTime << "', '" << dataStreamPort << "', '" << value << "' );";
    bRequest.addString(osArg.str());
    bRequest = request(bRequest);

    return true;
}

bool autobiographicalMemory::storeDataStreamAllProviders(const string &synchroTime) {
    for (std::map<string, BufferedPort<Bottle>*>::const_iterator it = mapDataStreamReceiver.begin(); it != mapDataStreamReceiver.end(); ++it)
    {
        string type = it->first;
        Bottle* lastReading = it->second->read();

        if(lastReading != NULL) {
            for(int subtype = 0; subtype < lastReading->size(); subtype++) {
                storeDataStream(imgInstance, frameNb, type, subtype, synchroTime, mapDataStreamProvider[it->first], lastReading->get(subtype).asDouble());
            }
        }
    }

    return true;
}

// From here on all send stream related
int autobiographicalMemory::openDataStreamPorts(int instance) {
    Bottle bDistLabelPort;
    ostringstream osArg;

    bDistLabelPort.addString("request");
    osArg << "SELECT DISTINCT label_port FROM proprioceptivedata WHERE instance = " << instance << endl;
    bDistLabelPort.addString(osArg.str());
    bDistLabelPort = request(bDistLabelPort);

    for (int i = 0; i < bDistLabelPort.size() && bDistLabelPort.toString()!="NULL"; i++) {
        string dataStreamPort = bDistLabelPort.get(i).asList()->get(0).asString();
        mapDataStreamPortOut[dataStreamPort] = new yarp::os::BufferedPort < Bottle >;
        mapDataStreamPortOut[dataStreamPort]->open((portPrefixForStreaming+dataStreamPort).c_str());
    }

    cout << "[openDataStreamPorts] Just created " << mapDataStreamPortOut.size() << " ports." << endl;

    return mapDataStreamPortOut.size();
}

unsigned int autobiographicalMemory::getStreamDataProviderCount(int instance) {
    Bottle bRequest;
    ostringstream osArg;

    bRequest.addString("request");
    osArg << "SELECT DISTINCT label_port, type, subtype FROM proprioceptivedata WHERE instance = " << instance;
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

Bottle autobiographicalMemory::getStreamDataWithinEpoch(long updateTimeDifference) {
    Bottle bListDataStream;
    bListDataStream.addString("request");
    ostringstream osArgDataStream;

    osArgDataStream << "SELECT * FROM (";
    osArgDataStream << "SELECT subtype, label_port, time, value, ";
    osArgDataStream << "CAST(EXTRACT(EPOCH FROM time-(SELECT time FROM proprioceptivedata WHERE instance = '" << imgInstance << "' ORDER BY time LIMIT 1)) * 1000000 as INT) as time_difference ";
    osArgDataStream << "FROM proprioceptivedata WHERE instance = '" << imgInstance << "' ORDER BY time) s ";
    if(realtimePlayback) {
        osArgDataStream << "WHERE time_difference <= " << updateTimeDifference << " and time_difference > " << timeLastImageSent << " ORDER BY time, label_port, subtype DESC LIMIT " << streamDataProviderCount << ";";
    } else {
        osArgDataStream << "WHERE time_difference > " << timeLastImageSent << " ORDER BY time, label_port, subtype ASC LIMIT " << streamDataProviderCount << ";";
    }

    bListDataStream.addString(osArgDataStream.str());
    return request(bListDataStream);
}
