#ifndef __CVZ_BUILDER_H__
#define __CVZ_BUILDER_H__

#include "ICvz.h"
#include "GuiICvz.h"
#define TYPE_ICVZ "icvz"

#include "CvzMMCM.h"
#include "GuiMMCM.h"
#define TYPE_MMCM "mmcm"

#include "CvzMLP.h"
//#include "GuiICvz.h"
#define TYPE_MLP "mlp"

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

class CvzGuiBuilder
{
public:
	static std::string helpMessage()
	{
		std::string s = "CvzGuiBuilder knows the following types : \n";
		s += TYPE_ICVZ;
		s += " (Convergence Zone Interface)" + '\n';
		s += TYPE_MMCM;
		s += " (Multi Modal Convergence Map)" + '\n';
		s += TYPE_MLP;
		s += " (Multiple Layered Perceptron)" + '\n';
		return s;
	}

	static bool allocate(GuiICvz** ptr, const std::string &type, IConvergenceZone* ptrCvz)
	{
		if (type == TYPE_ICVZ)
			(*ptr) = new GuiICvz(ptrCvz);
		else if (type == TYPE_MMCM)
			(*ptr) = new GuiMMCM(ptrCvz);
		else if (type == TYPE_MLP)
			(*ptr) = new GuiICvz(ptrCvz);
		else
			return false;
		return true;
	}
};

#endif