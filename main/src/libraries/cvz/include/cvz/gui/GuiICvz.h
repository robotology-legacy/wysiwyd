#ifndef __CVZ_GUITHREAD_H__
#define __CVZ_GUITHREAD_H__

#include <gtk/gtk.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "cvz/core/ICvz.h"
#include "GuiIModality.h"
#include "cvz/helpers/YarpImage2Pixbuf.h"
namespace cvz {
	namespace gui {
		static gboolean delete_event(GtkWidget *widget,
			GdkEvent  *event,
			gpointer   data)
		{
			return FALSE;
		}

		static void destroy(GtkWidget *widget,
			gpointer   data)
		{
			gtk_main_quit();
		}


		class GuiICvz : public yarp::os::Thread
		{

		protected:
			GtkWidget *mainWindow;
			GtkWidget *boxTabs;
			GtkWidget *boxMainInfo;
			GtkWidget *boxModalities;
			std::map<core::IModality*, GuiIModality* > wModalities;
			GtkWidget *label_name;
			GtkWidget *label_type;
			core::IConvergenceZone* myCvz;

			virtual void initElements()
			{
				mainWindow = gtk_window_new(GTK_WINDOW_TOPLEVEL);
				gtk_window_set_title(GTK_WINDOW(mainWindow), "Convergence Zone GUI");

				/* When the window is given the "delete-event" signal (this is given
				* by the window manager, usually by the "close" option, or on the
				* titlebar), we ask it to call the delete_event () function
				* as defined above. The data passed to the callback
				* function is NULL and is ignored in the callback function. */
				g_signal_connect(mainWindow, "delete-event",
					G_CALLBACK(delete_event), NULL);

				/* Here we connect the "destroy" event to a signal handler.
				* This event occurs when we call gtk_widget_destroy() on the window,
				* or if we return FALSE in the "delete-event" callback. */
				g_signal_connect(mainWindow, "destroy",
					G_CALLBACK(destroy), NULL);

				gtk_container_set_border_width(GTK_CONTAINER(mainWindow), 10);

				boxTabs = gtk_hbox_new(FALSE, 0);
				gtk_container_add(GTK_CONTAINER(mainWindow), boxTabs);
				gtk_widget_show(boxTabs);

				boxMainInfo = gtk_vbox_new(FALSE, 0);
				gtk_box_pack_start(GTK_BOX(boxTabs), boxMainInfo, FALSE, FALSE, 0);
				gtk_widget_show(boxMainInfo);

				boxModalities = gtk_vbox_new(FALSE, 0);
				gtk_box_pack_start(GTK_BOX(boxTabs), boxModalities, FALSE, FALSE, 50);
				gtk_widget_show(boxModalities);

				//Basic info in label form
				label_name = gtk_label_new(myCvz->getName().c_str());
				gtk_box_pack_start(GTK_BOX(boxMainInfo), label_name, FALSE, FALSE, 0);
				gtk_widget_show(label_name);

				label_type = gtk_label_new("Cvz Type - ICvz");
				gtk_box_pack_start(GTK_BOX(boxMainInfo), label_type, FALSE, FALSE, 0);
				gtk_widget_show(label_type);

                
                //Display the fixed parameters
                yarp::os::Bottle bFixedParameters;
                bFixedParameters.read(myCvz->getParametersStartTime());

                for (int i = 0; i < bFixedParameters.size(); i++)
                {
                    std::string key = bFixedParameters.get(i).asList()->get(0).asString();
                    yarp::os::Value value = bFixedParameters.get(i).asList()->get(1);

                    GtkWidget *tmpWidget = gtk_label_new( (key + " : " + value.toString().c_str()).c_str());
                    gtk_box_pack_start(GTK_BOX(boxMainInfo), tmpWidget, TRUE, TRUE, 0);
                    gtk_widget_show(tmpWidget);
                }
                //Display the runtime parameters
                yarp::os::Bottle bVariableParameters;
                bVariableParameters.read(myCvz->parametersRuntime);
                for (int i = 0; i < bVariableParameters.size(); i++)
                {
                    std::string key = bVariableParameters.get(i).asList()->get(0).asString();
                    yarp::os::Value value = bVariableParameters.get(i).asList()->get(1);
                    if (value.isDouble() || value.isInt())
                    {                     
                        LabelledSlider* sliderTmp = new LabelledSlider();
                        sliderTmp->allocate(key.c_str(), &myCvz->parametersRuntime, 0.0, 1.0, 0.1);
                        gtk_box_pack_start(GTK_BOX(boxMainInfo), sliderTmp->box, FALSE, FALSE, 0);
                    }
                }
                
				GtkWidget *labelModTitle = gtk_label_new("<--Modalities Bottom Up-->");
				gtk_box_pack_start(GTK_BOX(boxModalities), labelModTitle, FALSE, FALSE, 10);
				gtk_widget_show(labelModTitle);

				for (std::map<std::string, core::IModality*>::iterator it = myCvz->modalitiesBottomUp.begin(); it != myCvz->modalitiesBottomUp.end(); it++)
				{
					GuiIModality* mW = new GuiIModality();
					GtkWidget* boxMod = mW->allocate(it->second);
					wModalities[it->second] = mW;
					gtk_box_pack_start(GTK_BOX(boxModalities), boxMod, FALSE, FALSE, 10);
				}

				GtkWidget *labelModTDTitle = gtk_label_new("<--Modalities Top Down-->");
				gtk_box_pack_start(GTK_BOX(boxModalities), labelModTDTitle, FALSE, FALSE, 10);
				gtk_widget_show(labelModTDTitle);

				for (std::map<std::string, core::IModality*>::iterator it = myCvz->modalitiesTopDown.begin(); it != myCvz->modalitiesTopDown.end(); it++)
				{
					GuiIModality* mW = new GuiIModality();
					GtkWidget* boxMod = mW->allocate(it->second);
					wModalities[it->second] = mW;
					gtk_box_pack_start(GTK_BOX(boxModalities), boxMod, FALSE, FALSE, 10);
				}
                
                //Add the influence/learning slider to every modality
                for (std::map < core::IModality*, GuiIModality*>::iterator wMod = wModalities.begin(); wMod != wModalities.end(); wMod++)
                {
                    GtkWidget* paramModBox = gtk_vbox_new(FALSE, 0);
                    gtk_box_pack_start(GTK_BOX(wMod->second->box), paramModBox, FALSE, FALSE, 0);

                    LabelledSlider sliderInf;
                    sliderInf.allocate("Influence", &(myCvz->modalitiesInfluence[wMod->first]), 0.0, 1.0, 0.1);
                    gtk_box_pack_start(GTK_BOX(paramModBox), sliderInf.box, FALSE, FALSE, 0);
                    LabelledSlider sliderLear;
                    sliderLear.allocate("Learning", &(myCvz->modalitiesLearning[wMod->first]), 0.0, 1.0, 0.1);
                    gtk_box_pack_start(GTK_BOX(paramModBox), sliderLear.box, FALSE, FALSE, 0);

                    gtk_widget_show(paramModBox);
                }
			}

			virtual void refreshElements()
			{
				for (std::map<core::IModality*, GuiIModality* >::iterator it = wModalities.begin(); it != wModalities.end(); it++)
				{
					yarp::sig::ImageOf<yarp::sig::PixelRgb> imgR = it->first->getVisualization();
					helpers::yarpImage2Pixbuf(&imgR, it->second->pxBufReal);
					gtk_widget_queue_draw(it->second->frameReal);
					yarp::sig::ImageOf<yarp::sig::PixelRgb> imgP = it->first->getVisualization(true);
					helpers::yarpImage2Pixbuf(&imgP, it->second->pxBufPredicted);
					gtk_widget_queue_draw(it->second->framePredicted);
				}
			}

		private:

			bool threadInit()
			{
				gtk_init(0, NULL);
				gdk_threads_init();
				gdk_threads_enter();

				//Init the subclass elements
				initElements();

				gdk_threads_leave();

				//RefreshElements();

				gtk_widget_show(mainWindow);
				return true;
			}

			void threadRelease()
			{
				gtk_main_quit();
			}

			void run()
			{
				gtk_main();
			}

		public:

			GuiICvz(core::IConvergenceZone* linkedCvz)
			{
				myCvz = linkedCvz;
			}

			virtual void RefreshElements()
			{
				gdk_threads_enter();
				if (myCvz == NULL)
					return;
				refreshElements();
				gdk_threads_leave();
			}

		};

		class GuiThread : public yarp::os::RateThread
		{
			GuiICvz* gui;

		public:

			GuiThread(GuiICvz* _gui, int _period) :yarp::os::RateThread(_period)
			{
				gui = _gui;
			}

			void run()
			{
				gui->RefreshElements();
			}

			void threadRelease()
			{
				gtk_main_quit();
			}
		};
	}
}
#endif
