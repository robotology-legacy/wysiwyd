#ifndef __CVZ_BUILDERGUI_H__
#define __CVZ_BUILDERGUI_H__

#include "cvz/core/CvzBuilder.h"
#include "cvz/gui/GuiICvz.h"
#include "cvz/gui/GuiMMCM.h"
#include "cvz/gui/GuiESOM.h"
#include "cvz/gui/GuiTemplate.h"
#include "cvz/gui/GuiNN.h"


namespace cvz {
    namespace gui {


class CvzGuiBuilder
{
public:
    static std::string helpMessage()
    {
        std::string s = "CvzGuiBuilder knows the following types : \n";
        s += core::TYPE_ICVZ;
        s += " (Convergence Zone Interface)\n";
        s += core::TYPE_MMCM;
        s += " (Multi Modal Convergence Map)\n" ;
        s += core::TYPE_MLP;
        s += " (Multiple Layered Perceptron)\n";
        s += core::TYPE_ESOM;
        s += " (Evolving Self Organizing Map)\n";
        s += core::TYPE_TEMPLATE;
        s += " (Dummy Template)\n";
        s += core::TYPE_NN;
        s += " (Neural Model Test)\n";
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
        else if (type == cvz::core::TYPE_ESOM)
            (*ptr) = new cvz::gui::GuiESOM(ptrCvz);
        else if (type == cvz::core::TYPE_TEMPLATE)
            (*ptr) = new cvz::gui::GuiTemplate(ptrCvz);
        else if (type == cvz::core::TYPE_NN)
            (*ptr) = new cvz::gui::GuiNN(ptrCvz);
        else
            return false;
        return true;
    }
};
}
}
#endif
