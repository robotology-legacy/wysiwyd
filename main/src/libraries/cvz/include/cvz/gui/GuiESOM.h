#ifndef __CVZ_GUIESOM_H__
#define __CVZ_GUIESOM_H__

#include "GuiICvz.h"
#include "GuiHelpers.h"
#include "cvz/core/CvzESOM.h"
#include "cvz/helpers/YarpImage2Pixbuf.h"

namespace cvz {
	namespace gui {


		class GuiESOM : public GuiICvz
		{
			GtkWidget *label_dimension;

			void initElements()
			{
				this->GuiICvz::initElements();

                core::CvzESOM* m = getCvz();
				//Add suplementary info to the infobox
				
                std::string dim = "Status: \n \t Nodes=" + helpers::int2str(m->getNodesCount()) + "\n \t Connections:" + helpers::int2str(m->getConnectionsCount());
				label_dimension = gtk_label_new(dim.c_str());
				gtk_label_set_text(GTK_LABEL(label_dimension), dim.c_str());

				gtk_box_pack_start(GTK_BOX(boxMainInfo), label_dimension, FALSE, FALSE, 0);
				gtk_widget_show(label_dimension);

			}

			virtual void refreshElements()
			{
				this->GuiICvz::refreshElements();
                core::CvzESOM* m = getCvz();
                std::string dim = "Status: \n \t Nodes=" + helpers::int2str(m->getNodesCount()) + "\n \t Connections:" + helpers::int2str(m->getConnectionsCount());
                gtk_label_set_text(GTK_LABEL(label_dimension), dim.c_str());
			}

            core::CvzESOM* getCvz() { return (core::CvzESOM *)myCvz; }

		public:

			GuiESOM(core::IConvergenceZone* linkedCvz) :GuiICvz(linkedCvz)
			{
			}

		};
	}
}
#endif
