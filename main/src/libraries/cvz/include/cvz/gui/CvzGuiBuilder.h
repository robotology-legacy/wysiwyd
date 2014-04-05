#ifndef __CVZ_BUILDERGUI_H__
#define __CVZ_BUILDERGUI_H__

#include "cvz/core/CvzBuilder.h"
#include "cvz/gui/GuiICvz.h"

#include "cvz/core/CvzMMCM.h"
#include "cvz/gui/GuiMMCM.h"

#include "cvz/core/CvzMLP.h"

namespace cvz {
	namespace gui {


class CvzGuiBuilder
{
public:
	static std::string helpMessage()
	{
		std::string s = "CvzGuiBuilder knows the following types : \n";
		s += core::TYPE_ICVZ;
		s += " (Convergence Zone Interface)" + '\n';
		s += core::TYPE_MMCM;
		s += " (Multi Modal Convergence Map)" + '\n';
		s += core::TYPE_MLP;
		s += " (Multiple Layered Perceptron)" + '\n';
		return s;
	}

	static bool allocate(cvz::gui::GuiICvz** ptr, const std::string &type, core::IConvergenceZone* ptrCvz)
	{
		if (type == cvz::core::TYPE_ICVZ)
			(*ptr) = new cvz::gui::GuiICvz(ptrCvz);
		else if (type == cvz::core::TYPE_MMCM)
			(*ptr) = new cvz::gui::GuiMMCM(ptrCvz);
		else if (type == cvz::core::TYPE_MLP)
			(*ptr) = new cvz::gui::GuiICvz(ptrCvz);
		else
			return false;
		return true;
	}
};
}
}
#endif