#ifndef __CVZ_GUIHELPERS_H__
#define __CVZ_GUIHELPERS_H__

#include <gtk/gtk.h>
#include <string>
namespace cvz {
	namespace gui {
		static void Gui_onSliderChange(GtkWidget *widget, gpointer data)
		{
			if (data == NULL)
				return;
			double* inf = (double*)data;
			double iValue = gtk_range_get_value(GTK_RANGE(widget));
			(*inf) = iValue;
		}
        static void Gui_onSliderChangeProperty(GtkWidget *widget, gpointer data);

		class LabelledSlider
		{
        public:
			GtkWidget* box;
			GtkWidget* label;
			GtkWidget* slider;
            
            std::string parameterName;
            yarp::os::Property* parameters;

            LabelledSlider(){};
            ~LabelledSlider(){};

			GtkWidget* allocate(std::string lab, double *val, double min = 0.0, double max = 1.0, double step = 0.1)
            {
                if (*val < step)
                    step = *val;

                parameterName = lab;
				box = gtk_hbox_new(FALSE, 0);
				label = gtk_label_new(lab.c_str());;

				slider = gtk_hscale_new_with_range(min, max, step);
                //slider->name = (gchar*)lab.c_str();
				gtk_scale_set_value_pos(GTK_SCALE(slider), GTK_POS_LEFT);
				gtk_range_set_value(GTK_RANGE(slider), *val);
				g_signal_connect(G_OBJECT(slider), "value-changed", G_CALLBACK(Gui_onSliderChange), val);

				gtk_box_pack_start(GTK_BOX(box), label, FALSE, FALSE, 0);
				gtk_box_pack_start(GTK_BOX(box), slider, FALSE, FALSE, 0);

				gtk_widget_set_size_request(label, 100, 15);
				gtk_widget_set_size_request(slider, 100, 15);
				gtk_widget_show(label);
				gtk_widget_show(slider);
				gtk_widget_show(box);
				return box;
			}

            GtkWidget* allocate(std::string lab, yarp::os::Property *val, double min = 0.0, double max = 1.0, double step = 0.1)
            {
                double currentValue = val->find(lab).asDouble();
                if (val->find(lab).isInt())
                {
                    min = 0;
                    max = 100;
                    step = 1;
                    currentValue = val->find(lab).asInt();
                }
                if (currentValue < step)
                    step = currentValue;

                parameterName = lab;
                parameters = val;
                box = gtk_hbox_new(FALSE, 0);
                label = gtk_label_new(lab.c_str());;

                slider = gtk_hscale_new_with_range(min, max, step);
                //slider->name = (gchar*)lab.c_str();
                gtk_scale_set_value_pos(GTK_SCALE(slider), GTK_POS_LEFT);
                gtk_range_set_value(GTK_RANGE(slider), currentValue);
                g_signal_connect(G_OBJECT(slider), "value-changed", G_CALLBACK(Gui_onSliderChangeProperty), this);

                gtk_box_pack_start(GTK_BOX(box), label, FALSE, FALSE, 0);
                gtk_box_pack_start(GTK_BOX(box), slider, FALSE, FALSE, 0);

                gtk_widget_set_size_request(label, 100, 15);
                gtk_widget_set_size_request(slider, 100, 15);
                gtk_widget_show(label);
                gtk_widget_show(slider);
                gtk_widget_show(box);
                return box;
            }
		};

        static void Gui_onSliderChangeProperty(GtkWidget *widget, gpointer data)
        {
            if (data == NULL)
                return;
            cvz::gui::LabelledSlider * inf = (cvz::gui::LabelledSlider *)data;
            double iValue = gtk_range_get_value(GTK_RANGE(widget));
            inf->parameters->put(inf->parameterName, iValue);
        }

	}
}

#endif