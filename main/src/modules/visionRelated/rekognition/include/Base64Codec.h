/*
 * Base64Codec.h
 *
 *  Created on: Apr 27, 2014
 *      Author: orbeus
 *      This is a modified version of base64 encoder, used for url posting.
 */

#ifndef BASE64CODEC_H_
#define BASE64CODEC_H_

#include <string>

namespace rekognition_api {

class Base64Codec {
public:
	Base64Codec();
	virtual ~Base64Codec();

	bool Encode(const std::string& input, std::string* output);
	bool Decode(const std::string& input, std::string* output);
};

} // namespace rekognition_api
#endif // BASE64CODEC_H_
