// Author      : Meng Wang (mwang@orbe.us)
// Copyright   : Orbeus 2013 All right reserved
// Description : A test of C++ rekognition SDK.

#include "RESTapi.h"
#include "json/json.h"
#include "Base64Codec.h"

#include <fstream>
#include <sstream>
#include <streambuf>

using namespace std;

int main(int argc, char **argv) {
	Json::Value response;
	const string api_addr_base = "https://rekognition.com/func/api/?";
	map<string, string> query_config;

	// for testing only, please use client specific key and secret!
	query_config["api_key"] = "1234";
	query_config["api_secret"] = "5678";
	query_config["jobs"] = "scene_understanding_3";
	//query_config["urls"] = "http://rekognition.com/static/img/people.jpg";

	std::ifstream t("/home/maxime/Human_Hand.jpg");
	std::string buff_str((std::istreambuf_iterator<char>(t)),
			std::istreambuf_iterator<char>());

	rekognition_api::Base64Codec base64_codec;
	string encoded;
	base64_codec.Encode(buff_str, &encoded);
	query_config["base64"] = encoded;

	if (!rekognition_api::APICall(api_addr_base, query_config, &response)) {
		cerr << "API call failure!" << endl;
		return -1;
	} else {
                cout << "API call OK" << endl;
        }

	// For the format of the results, please refer to our doc:
	// http://rekognition.com/docs/
	/*const Json::Value face_detection = response["face_detection"];
        cout << "Detected " << face_detection.size() << " faces." << endl;
	for (unsigned int i = 0; i < face_detection.size(); ++i) {
		double x, y, w, h;
		x = face_detection[i]["boundingbox"]["tl"]["x"].asDouble();
		y = face_detection[i]["boundingbox"]["tl"]["y"].asDouble();
		w = face_detection[i]["boundingbox"]["size"]["width"].asDouble();
		h = face_detection[i]["boundingbox"]["size"]["height"].asDouble();

		cout << "face " << i << ": [" << x << " " << y << " " << w << " " << h
				<< "]" << endl;
	}*/

	const Json::Value face_detection = response["scene_understanding"]["matches"];
        cout << "Detected " << face_detection.size() << " faces." << endl;
	for (unsigned int i = 0; i < face_detection.size(); ++i) {
		double score;
		string tag;
		tag = face_detection[i]["tag"].asString();
		score = face_detection[i]["score"].asDouble();
		cout << "scene" << i << ": [" << tag << " " << score << "]" << endl;
	}

	return 0;
}

