// Author      : Meng Wang (mwang@orbe.us)
// Copyright   : Orbeus 2013 All right reserved
// Description : This C++ SDK is for rekognition RESTful API.
// Usage:
// 1. Create response json object.
// Json::Value response;
//
// 2. Specify API request in a map object.
// const string api_addr_base = "https://rekognition.com/func/api/?";
// map<string, string> query_config;
// query_config["api_key"] = "YOUR_API_KEY";
// query_config["api_secret"] = "YOUR_API_SECRET";
// query_config["jobs"] = "face_part";
// query_config["urls"] = "http://rekognition.com/static/img/people.jpg";
//
// 3. Call API with APICall function.
// rekognition_api::APICall(api_addr_base, query_config, &response);

#ifndef RESTAPI_H_
#define RESTAPI_H_

#include <string>
#include <map>

namespace Json {
	class Value;
}  // namespace

namespace rekognition_api {

bool APICall(const std::string& api_addr_base,
						 const std::map<std::string,
						 std::string>& query_config,
						 Json::Value* response);

}  // namespace rekognition_api
#endif // RESTAPI_H_
