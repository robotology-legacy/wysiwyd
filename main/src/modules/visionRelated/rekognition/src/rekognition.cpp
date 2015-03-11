/*
 *
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer and Maxime Petit
 * email:   t.fischer@imperial.ac.uk and m.petit@imperial.ac.uk
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

#include <yarp/os/all.h>
#include "rekognition.h"

#include "RESTapi.h"
#include "json/json.h"
#include "Base64Codec.h"

#include <fstream>
#include <sstream>
#include <streambuf>

using namespace std;
using namespace yarp::os;

bool rekognition::configure(yarp::os::ResourceFinder &rf) {
    setName(rf.check("name", Value("rekognition"), "module name (string)").asString().c_str());
    cout << "Setting up module " << rf.check("name", Value("rekognition")).asString() << endl;

    api_addr_base = "https://rekognition.com/func/api/?";
    string handlerPortName = "/" + getName() + "/rpc";
    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    api_key = rf.check("api_key",Value("1234"), "api_key (string)").asString();
    api_secret = rf.check("api_secret",Value("5678"), "api_secret (string)").asString();

    cout << "Using key " << api_key << " with secret " << api_secret << endl;

    attach(handlerPort);

    return true;
}

bool rekognition::interrupt() {
    return true;
}

bool rekognition::close() {
    handlerPort.interrupt();
    handlerPort.close();

    return true;
}

bool rekognition::respond(const Bottle& cmd, Bottle& reply) {
    if (cmd.get(0).asString() == "recognizeFace") {
        string filepath = cmd.get(1).asString();

        Json::Value response;
        map<string, string> query_config;

        // for testing only, please use client specific key and secret!
        query_config["api_key"] = api_key;
        query_config["api_secret"] = api_secret;
        query_config["jobs"] = "face_recognize";
        query_config["name_space"] = "wysiwyd";
        query_config["user_id"] = "demo_user";

        std::ifstream t(filepath.c_str());
        std::string buff_str((std::istreambuf_iterator<char>(t)),
                             std::istreambuf_iterator<char>());

        rekognition_api::Base64Codec base64_codec;
        string encoded;
        base64_codec.Encode(buff_str, &encoded);
        query_config["base64"] = encoded;

        if (!rekognition_api::APICall(api_addr_base, query_config, &response)) {
            reply.addString("nack");
        } else {
            const Json::Value face_recognition = response["face_detection"];
            cout << "Detected " << face_recognition.size() << " faces." << endl;
            cout << response.toStyledString() << endl;
            for (unsigned int i = 0; i < face_recognition.size(); ++i) {
                double x, y, w, h, confidence;
                string name;
                x = face_recognition[i]["boundingbox"]["tl"]["x"].asDouble();
                y = face_recognition[i]["boundingbox"]["tl"]["y"].asDouble();
                w = face_recognition[i]["boundingbox"]["size"]["width"].asDouble();
                h = face_recognition[i]["boundingbox"]["size"]["height"].asDouble();

                int match_number = 0;
                name = face_recognition[i]["matches"][match_number]["tag"].asString();
                confidence = atof(face_recognition[i]["matches"][match_number]["score"].asString().c_str());

                cout << "Face " << i << ": [" << x << " " << y << " " << w << " " << h
                     << "]" << " ( " << name << " ) Confidence: " << confidence << endl;
            }
            reply.addString("ack");
        }
    } else if (cmd.get(0).asString() == "detectFaces") {
        string filepath = cmd.get(1).asString();

        Json::Value response;
        map<string, string> query_config;

        // for testing only, please use client specific key and secret!
        query_config["api_key"] = api_key;
        query_config["api_secret"] = api_secret;
        query_config["jobs"] = "face";

        std::ifstream t(filepath.c_str());
        std::string buff_str((std::istreambuf_iterator<char>(t)),
                             std::istreambuf_iterator<char>());

        rekognition_api::Base64Codec base64_codec;
        string encoded;
        base64_codec.Encode(buff_str, &encoded);
        query_config["base64"] = encoded;

        if (!rekognition_api::APICall(api_addr_base, query_config, &response)) {
            reply.addString("nack");
        } else {
            const Json::Value face_detection = response["face_detection"];
            cout << "Detected " << face_detection.size() << " faces." << endl;
            for (unsigned int i = 0; i < face_detection.size(); ++i) {
                double x, y, w, h;
                x = face_detection[i]["boundingbox"]["tl"]["x"].asDouble();
                y = face_detection[i]["boundingbox"]["tl"]["y"].asDouble();
                w = face_detection[i]["boundingbox"]["size"]["width"].asDouble();
                h = face_detection[i]["boundingbox"]["size"]["height"].asDouble();

                cout << "Face " << i << ": [" << x << " " << y << " " << w << " " << h
                     << "]" << endl;
            }
            reply.addString("ack");
        }
    } else if (cmd.get(0).asString() == "tagObject") {
        string filepath = cmd.get(1).asString();

        Json::Value response;
        map<string, string> query_config;

        // for testing only, please use client specific key and secret!
        query_config["api_key"] = api_key;
        query_config["api_secret"] = api_secret;
        query_config["jobs"] = "scene_understanding_3";

        std::ifstream t(filepath.c_str());
        std::string buff_str((std::istreambuf_iterator<char>(t)),
                             std::istreambuf_iterator<char>());

        rekognition_api::Base64Codec base64_codec;
        string encoded;
        base64_codec.Encode(buff_str, &encoded);
        query_config["base64"] = encoded;

        if (!rekognition_api::APICall(api_addr_base, query_config, &response)) {
            reply.addString("nack");
        } else {
            const Json::Value scene_tags = response["scene_understanding"]["matches"];
            for (unsigned int i = 0; i < scene_tags.size(); ++i) {
                double score;
                string tag;
                tag = scene_tags[i]["tag"].asString();
                score = scene_tags[i]["score"].asDouble();
                cout << "Scene " << i << ": [" << tag << " " << score << "]" << endl;
            }
            reply.addString("ack");
        }
    } else {
        reply.addString("nack");
    }

    return true;
}
