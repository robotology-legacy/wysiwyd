#ifndef __CVZ_GUIIMODALITY_H__
#define __CVZ_GUIIMODALITY_H__

#include <gtk/gtk.h>
#include "cvz/core/IModality.h"

namespace cvz {
    namespace gui {
#define CVZ_GUI_MODALITY_DIM_W 130
#define CVZ_GUI_MODALITY_DIM_H 10 //In the case of a vector being displayed. Else this will be calculated from the width and the visualization ratio

        static gboolean paintModalities(GtkWidget *widget, GdkEventExpose *event, gpointer data)
        {
            GdkPixbuf* pxBuf = (GdkPixbuf*)data;
            double displayRatio = (double)gdk_pixbuf_get_width(pxBuf) / (double)gdk_pixbuf_get_height(pxBuf);
            int modWidth = CVZ_GUI_MODALITY_DIM_W;
            int modHeight = (int)(CVZ_GUI_MODALITY_DIM_W / displayRatio);
            if (gdk_pixbuf_get_height(pxBuf) == 1)
                modHeight = CVZ_GUI_MODALITY_DIM_H;

            GdkPixbuf* scaledFrame = gdk_pixbuf_scale_simple(pxBuf, modWidth, modHeight, GDK_INTERP_BILINEAR); // Best quality

            guchar *pixels = gdk_pixbuf_get_pixels(scaledFrame);
            int rowstride = gdk_pixbuf_get_rowstride(scaledFrame);
            gdk_draw_rgb_image(widget->window,
                widget->style->black_gc,
                0, 0,
                modWidth, modHeight,
                GDK_RGB_DITHER_NORMAL,
                pixels,
                rowstride);

            g_object_unref(scaledFrame);
            return true;
        }


        class GuiIModality
        {

        public:
            GtkWidget* box;
            GtkWidget* frameReal;
            GtkWidget* framePredicted;
            GdkPixbuf* pxBufReal;
            GdkPixbuf* pxBufPredicted;

            GuiIModality()
            {

            }


            GtkWidget* allocate(core::IModality* m)
            {
                box = gtk_hbox_new(FALSE, 0);

                std::string modInfStr = "Modality: ";
                modInfStr += m->Name() + "\n(Size = " + helpers::int2str(m->Size()) + ")";
                GtkWidget *labelModInfo = gtk_label_new(modInfStr.c_str());;
                gtk_widget_show(labelModInfo);
                gtk_box_pack_start(GTK_BOX(box), labelModInfo, FALSE, FALSE, 0);

                GtkWidget* boxModReal = gtk_hbox_new(FALSE, 0);
                gtk_box_pack_start(GTK_BOX(box), boxModReal, FALSE, FALSE, 0);
                gtk_widget_show(boxModReal);
                GtkWidget *labelReal = gtk_label_new("Real :");
                gtk_widget_set_size_request(labelReal, 100, 10);
                gtk_widget_show(labelReal);
                gtk_box_pack_start(GTK_BOX(boxModReal), labelReal, FALSE, FALSE, 0);

                yarp::sig::ImageOf<yarp::sig::PixelRgb> imgTest = m->getVisualization();
                int widthTested = imgTest.width();
                int heightTested = imgTest.height();
                double displayRatio = (double)widthTested / (double)heightTested;
                int modWidth = CVZ_GUI_MODALITY_DIM_W;
                int modHeight = (int)(CVZ_GUI_MODALITY_DIM_W / displayRatio);
                if (heightTested==1)
                    modHeight = CVZ_GUI_MODALITY_DIM_H;

                pxBufReal = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, widthTested, heightTested);
                frameReal = gtk_drawing_area_new();
                g_signal_connect(frameReal, "expose-event", G_CALLBACK(paintModalities), pxBufReal);
                
                gtk_widget_set_size_request(frameReal, modWidth, modHeight);

                gtk_box_pack_start(GTK_BOX(boxModReal), frameReal, FALSE, FALSE, 0);
                gtk_widget_show(frameReal);

                GtkWidget* boxModPred = gtk_hbox_new(FALSE, 0);
                gtk_box_pack_start(GTK_BOX(box), boxModPred, FALSE, FALSE, 0);
                gtk_widget_show(boxModPred);
                GtkWidget *labelPred = gtk_label_new("Predicted :");
                gtk_widget_set_size_request(labelPred, 100, 10);
                gtk_widget_show(labelPred);
                gtk_box_pack_start(GTK_BOX(boxModPred), labelPred, FALSE, FALSE, 0);
                pxBufPredicted = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, widthTested, heightTested);
                framePredicted = gtk_drawing_area_new();
                g_signal_connect(framePredicted, "expose-event", G_CALLBACK(paintModalities), pxBufPredicted);
                gtk_widget_set_size_request(framePredicted, modWidth, modHeight);
                gtk_box_pack_start(GTK_BOX(boxModPred), framePredicted, FALSE, FALSE, 0);
                gtk_widget_show(framePredicted);

                gtk_widget_show(box);
                return box;
            }
        };
    }
}
#endif