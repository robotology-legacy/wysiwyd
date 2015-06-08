/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer, Maxime Petit
 * email:   t.fischer@imperial.ac.uk, m.petit@imperial.ac.uk
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

#ifndef TEMPLATES_H
#define TEMPLATES_H

class autobiographicalMemory;

template<typename T>
struct Trait
{
    typedef void value;
};

template<>
struct Trait < yarp::os::Bottle >
{
    typedef typename yarp::os::Bottle value;
};

template<>
struct Trait < yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
    typedef typename yarp::sig::ImageOf<yarp::sig::PixelRgb> value;
};

template<typename T>
yarp::os::Bottle autobiographicalMemory::listProviders(const std::map <std::string, yarp::os::BufferedPort<T>* > &map)
{
    yarp::os::Bottle bReply;

    for (typename std::map <std::string, yarp::os::BufferedPort<typename Trait<T>::value >* >::const_iterator it = map.begin(); it != map.end(); ++it)
    {
        bReply.addString(it->first);
    }

    return bReply;
}

template<typename T>
yarp::os::Bottle autobiographicalMemory::addStreamProvider(std::map <std::string, yarp::os::BufferedPort<T>* > &map, const std::string &portRemote)
{
    yarp::os::Bottle bReply;

    if (map.find(portRemote) == map.end()) //key not found
    {
        map[portRemote] = new yarp::os::BufferedPort < typename Trait<T>::value > ;
        bReply.addString("[ack]");

        std::string portLocal = "/" + getName() + portRemote + "/in";
        map[portRemote]->open(portLocal);
        if (!yarp::os::Network::isConnected(portRemote, portLocal)) {
            if (!yarp::os::Network::connect(portRemote, portLocal)) {
                std::string error = "[addStreamProvider] Error: Connection could not be setup from " + portRemote + " to " + portLocal;
                yError() << error;
                bReply.addString("nack");
                bReply.addString(error);
            }
            else {
                bReply.addString("[ack]");
            }
        }
        else {
            std::string warning = "[addStreamProvider] Error: Connection already present from " + portRemote + " to " + portLocal;
            bReply.addString("nack");
            yWarning() << warning;
        }
    }
    else { //key found
        std::string warning = "[addStreamProvider] " + portRemote + " is already present!";
        yWarning() << warning;
        bReply.addString("nack");
        bReply.addString(warning);
    }

    return bReply;
}

template<typename T>
yarp::os::Bottle autobiographicalMemory::removeStreamProvider(std::map <std::string, yarp::os::BufferedPort<T>* > &map, const std::string &portStreamProvider)
{
    yarp::os::Bottle bReply;

    if (map.find(portStreamProvider) == map.end()) { //key not found
        std::string warning = "[removeStreamProvider]: " + portStreamProvider + ") is NOT present!";
        yWarning() << warning;
        bReply.addString("nack");
        bReply.addString(warning);
    }
    else { //key found
        map[portStreamProvider]->interrupt();
        map[portStreamProvider]->close();
        map.erase(portStreamProvider);
        bReply.addString("[ack]");
    }

    return bReply;
}

template<typename T>
yarp::os::Bottle autobiographicalMemory::disconnectStreamProviders(std::map <std::string, yarp::os::BufferedPort<T>* > &map)
{
    yarp::os::Bottle bOutput;
    bool isAllDisconnected = true;

    if (map.size() == 0) {
        bOutput.addString("nack");
        bOutput.addString("[disconnectStreamProviders] ERROR the map has no elements");
        return bOutput;
    }

    for (typename std::map <std::string, yarp::os::BufferedPort<typename Trait<T>::value >* >::const_iterator it = map.begin(); it != map.end(); ++it)
    {
        yarp::os::Network::disconnect(it->first, it->second->getName().c_str());
        if (yarp::os::Network::isConnected(it->first, it->second->getName().c_str())) {
            std::string warning = "[disconnectStreamProviders] ERROR " + it->first + " is NOT disconnected!";
            yWarning() << warning;
            bOutput.addString("nack");
            bOutput.addString(warning);
            isAllDisconnected = false;
        }
        else {
            //Have to close/interrupt each time otherwise the port is not responsive anymore
            it->second->interrupt();
            it->second->close();
        }
    }

    if (isAllDisconnected == true){
        bOutput.addString("ack");
    }

    //yDebug() << "[disconnectStreamProviders] bOutput = {" << bOutput.toString().c_str() << "}";
    return bOutput;
}


#endif // TEMPLATES_H
