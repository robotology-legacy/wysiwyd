#ifndef __CVZ_GUITEMPLATE_H__
#define __CVZ_GUITEMPLATE_H__

#include "GuiICvz.h"
#include "GuiHelpers.h"
#include "cvz/core/CvzTemplate.h"
#include "cvz/helpers/YarpImage2Pixbuf.h"

namespace cvz {
	namespace gui {

		class GuiTemplate : public GuiICvz
		{
			void initElements()
			{
				this->GuiICvz::initElements();
				core::CvzTemplate* m = getCvz();
				gtk_label_set_text(GTK_LABEL(label_type), "Cvz Type - Template");
                //Create your widgets
			}

            //Refresh your controls/widgets
			virtual void refreshElements()
			{
				this->GuiICvz::refreshElements();
				core::CvzTemplate* m = getCvz();
                //Be carefull with semaphores (access data through methods of the CVZ that are using mutex to not overlap with the computePrediction() method.
			}

            //This is handy to get a good cast of your CVZ type
            core::CvzTemplate* getCvz() { return (core::CvzTemplate *)myCvz; }

		public:

            GuiTemplate(core::IConvergenceZone* linkedCvz) :GuiICvz(linkedCvz)
			{
			}

		};
	}
}
#endif
