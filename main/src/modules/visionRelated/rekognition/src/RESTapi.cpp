// Author      : Meng Wang (mwang@orbe.us)
// Copyright   : Orbeus 2013 All right reserved
// Description :

#include "RESTapi.h"

#include <curl/curl.h>
#include <iostream>

#include "json/json.h"

using namespace std;

namespace rekognition_api {

namespace {
static int writer(char *data, size_t size, size_t nmemb,
		std::string *writerData) {
	if (writerData == NULL) {
		std::cout << "error copy html!" << std::endl;
		return 0;
	}
	writerData->append(data, size * nmemb);

	return size * nmemb;
}
} // namespace

bool APICall(const string& api_addr_base,
		const map<string, string>& query_config, Json::Value* response) {
	CURL *curl;
	curl = curl_easy_init();
	if (!curl) {
		cerr << "CURL init error!" << endl;
		return false;
	}

	curl_easy_setopt(curl, CURLOPT_POST, 1L);
	curl_easy_setopt(curl, CURLOPT_HEADER, 0L);
	curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
	curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, false);
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
	curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);
	curl_easy_setopt(curl, CURLOPT_AUTOREFERER, 1L);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3);
	curl_easy_setopt(curl, CURLOPT_USERAGENT,
			"Mozilla/5.0 (Windows; U; Windows NT 6.1; en-US) "
			"AppleWebKit/534.7 (KHTML, like Gecko) Chrome/7.0.517.41 Safari/534.7");
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);
	curl_easy_setopt(curl, CURLOPT_URL, api_addr_base.c_str());

	string post_fields = "";
	for (map<string, string>::const_iterator iter = query_config.begin();
			iter != query_config.end(); ++iter) {
		post_fields.append("&");
		post_fields.append(iter->first);
		post_fields.append("=");
		post_fields.append(iter->second);
	}

	curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_fields.c_str());

	string photos_page;
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &photos_page);

	try {
		if (curl_easy_perform(curl) != CURLE_OK) {
			cerr << "CURL calling error!" << endl;
			curl_easy_cleanup(curl);
			return false;
		}
	} catch (...) {
		cerr << "CURL calling error!" << endl;
		curl_easy_cleanup(curl);
		return false;
	}

	Json::Reader reader;
	if (!reader.parse(photos_page, *response)) {
		cout << "Failed to parse response." << endl;
		curl_easy_cleanup(curl);
		return false;
	}
	curl_easy_cleanup(curl);

	return true;
}

} // namespace rekognition_api
