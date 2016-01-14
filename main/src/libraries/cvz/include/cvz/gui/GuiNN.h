#ifndef __CVZ_GUINN_H__
#define __CVZ_GUINN_H__

#include "GuiICvz.h"
#include "GuiHelpers.h"
#include "cvz/core/CvzNN.h"
#include "cvz/helpers/YarpImage2Pixbuf.h"

namespace cvz {
    namespace gui {
#define NN_GUI_ACTIVITY_H 200
#define NN_GUI_ACTIVITY_W 200

        static gboolean paintActivityNN(GtkWidget *widget, GdkEventExpose *event, gpointer data)
        {
            GdkPixbuf* pxBuf = (GdkPixbuf*)data;
            GdkPixbuf* scaledFrame = gdk_pixbuf_scale_simple(pxBuf, NN_GUI_ACTIVITY_W, NN_GUI_ACTIVITY_H, GDK_INTERP_BILINEAR); // Best quality

            guchar *pixels = gdk_pixbuf_get_pixels(scaledFrame);
            int rowstride = gdk_pixbuf_get_rowstride(scaledFrame);
            gdk_draw_rgb_image(widget->window,
                widget->style->black_gc,
                0, 0,
                NN_GUI_ACTIVITY_W, NN_GUI_ACTIVITY_H,
                GDK_RGB_DITHER_NORMAL,
                pixels,
                rowstride);

            g_object_unref(scaledFrame);
            return true;
        }

        class GuiNN : public GuiICvz
        {
            GtkWidget *box_activity;
            //GtkWidget *slider_sigma;
            GtkWidget* net_activity;
             GdkPixbuf* net_pxBuf_activity;

            void initElements()
            {
                this->GuiICvz::initElements();
                core::CvzNN* m = getCvz();
                gtk_label_set_text(GTK_LABEL(label_type), "Cvz Type - NN");
                //Create your widgets
                //Display the map activity for every layer
                box_activity = gtk_hbox_new(TRUE, 0);
                gtk_box_pack_start(GTK_BOX(boxMainInfo), box_activity, FALSE, FALSE, 0);

                net_pxBuf_activity = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, m->W(), m->H());
                net_activity = gtk_drawing_area_new();
                g_signal_connect(net_activity, "expose-event", G_CALLBACK(paintActivityNN), net_pxBuf_activity);
                gtk_widget_set_size_request(net_activity, NN_GUI_ACTIVITY_W, NN_GUI_ACTIVITY_H);
                gtk_box_pack_start(GTK_BOX(box_activity), net_activity, TRUE, TRUE, 5);
                gtk_widget_show(net_activity);
                gtk_widget_show(box_activity);
            }

            //Refresh your controls/widgets
            virtual void refreshElements()
            {
                this->GuiICvz::refreshElements();
                
                core::CvzNN* m = getCvz();
                //Be carefull with semaphores (access data through methods of the CVZ that are using mutex to not overlap with the computePrediction() method.
                yarp::sig::ImageOf<yarp::sig::PixelRgb> img = m->getNetworkActivity();
                helpers::yarpImage2Pixbuf(&img, net_pxBuf_activity);
                gtk_widget_queue_draw(net_activity);
            }

            //This is handy to get a good cast of your CVZ type
            core::CvzNN* getCvz() { return (core::CvzNN *)myCvz; }

        public:

            GuiNN(core::IConvergenceZone* linkedCvz) :GuiICvz(linkedCvz)
            {
            }

        };
    }
}
#endif
