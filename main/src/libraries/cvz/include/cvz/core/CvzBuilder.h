#ifndef __CVZ_BUILDER_H__
#define __CVZ_BUILDER_H__

#include "ICvz.h"
#include "CvzMMCM.h"
#include "CvzMLP.h"

namespace cvz {
	namespace core {

const std::string TYPE_ICVZ("icvz");
const std::string TYPE_MLP("mlp");
const std::string TYPE_MMCM("mmcm");

class CvzBuilder
{

public:
	static std::string helpMessage()
	{
		std::string s = "CvzBuilder knows the following types : \n";
		s += TYPE_ICVZ;
		s +=" (Convergence Zone Interface)" + '\n';
		s += TYPE_MMCM;
		s += " (Multi Modal Convergence Map)" + '\n';
		s += TYPE_MLP;
		s += " (Multiple Layered Perceptron)" + '\n';
		return s;
	}

	static bool allocate(IConvergenceZone** ptr, const std::string &type)
	{
		if (type == TYPE_ICVZ)
			(*ptr) = new IConvergenceZone();
		else if (type == TYPE_MMCM)
			(*ptr) = new CvzMMCM();
		else if (type == TYPE_MLP)
			(*ptr) = new CvzMLP();
		else
			return false;
		return true;
	}
};

}
}
#endif