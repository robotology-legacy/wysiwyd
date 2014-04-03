#ifndef __CVZ_GUIMMCM_H__
#define __CVZ_GUIMMCM_H__

#include "GuiICvz.h"
#include "GuiHelpers.h"
#include "CvzMMCM.h"

#define MMCM_GUI_ACTIVITY_H 200
#define MMCM_GUI_ACTIVITY_W 200

static void GuiMMCM_onLearningChange(GtkWidget *widget, gpointer data)
{
	if (data == NULL)
		return;
	CvzMMCM* m = (CvzMMCM*)data;
	double iValue = gtk_range_get_value(GTK_RANGE(widget));
	m->lRate = iValue;
}

static void GuiMMCM_onSigmaChange(GtkWidget *widget, gpointer data)
{
	if (data == NULL)
		return;
	CvzMMCM* m = (CvzMMCM*)data;
	double iValue = gtk_range_get_value(GTK_RANGE(widget));
	m->sigmaH = iValue;
}

static void GuiMMCM_onInfluenceChange(GtkWidget *widget, gpointer data)
{
	if (data == NULL)
		return;
	double* inf = (double*)data;
	double iValue = gtk_range_get_value(GTK_RANGE(widget));
	(*inf) = iValue;
}

static gboolean paintActivity(GtkWidget *widget, GdkEventExpose *event, gpointer data)
{
	GdkPixbuf* pxBuf = (GdkPixbuf*)data;
	GdkPixbuf* scaledFrame = gdk_pixbuf_scale_simple(pxBuf, MMCM_GUI_ACTIVITY_W, MMCM_GUI_ACTIVITY_H, GDK_INTERP_BILINEAR); // Best quality

	guchar *pixels = gdk_pixbuf_get_pixels(scaledFrame);
	int rowstride = gdk_pixbuf_get_rowstride(scaledFrame);
	gdk_draw_rgb_image(widget->window,
		widget->style->black_gc,
		0, 0,
		MMCM_GUI_ACTIVITY_W, MMCM_GUI_ACTIVITY_H,
		GDK_RGB_DITHER_NORMAL,
		pixels,
		rowstride);

	g_object_unref(scaledFrame);
	return true;
}


class GuiMMCM : public GuiICvz
{
	GtkWidget *label_dimension;
	GtkWidget *box_activity;
	GtkWidget *box_parameters;
	GtkWidget *slider_learning;
	GtkWidget *slider_sigma;
	vector< GtkWidget* > frames_activity;
	vector< GdkPixbuf* > pxBuf_activity;
	GuiIModality* recurrentGuiIModality;

	void initElements()
	{
		this->GuiICvz::initElements();

		CvzMMCM* m = getCvz();
		gtk_label_set_text(GTK_LABEL(label_type), "Cvz Type - MMCM");
		//Add suplementary info to the infobox
		std::string dim = "Dimension (" + int2str(m->W()) + "x" + int2str(m->H()) + "x" + int2str(m->L()) + ")";
		label_dimension = gtk_label_new(dim.c_str());
		gtk_label_set_text(GTK_LABEL(label_dimension), dim.c_str());

		gtk_box_pack_start(GTK_BOX(boxMainInfo), label_dimension, FALSE, FALSE, 0);
		gtk_widget_show(label_dimension);

		//Add box for parameters manipulation
		box_parameters = gtk_vbox_new(FALSE, 10);
		gtk_box_pack_start(GTK_BOX(boxMainInfo), box_parameters, FALSE, FALSE, 0);
		LabelledSlider sliderLearning;
		sliderLearning.allocate("Learning Rate", &(m->lRate), 0.0, 1.0, 0.001);
		gtk_box_pack_start(GTK_BOX(box_parameters), sliderLearning.box, FALSE, FALSE, 0);
		LabelledSlider sliderSigma;
		sliderSigma.allocate("Sigma", &(m->sigmaH), 0.5, sqrt(pow(m->W(), 2.0) + pow(m->H(), 2.0)), 0.5);
		gtk_box_pack_start(GTK_BOX(box_parameters), sliderSigma.box, FALSE, FALSE, 0);

		gtk_widget_show(box_parameters);
		//Display the reccurent layer
		if (m->recurrentModality)
		{
			recurrentGuiIModality = new GuiIModality();
			GtkWidget* boxMod = recurrentGuiIModality->allocate(m->recurrentModality);
			gtk_box_pack_start(GTK_BOX(boxModalities), boxMod, FALSE, FALSE, 0);	
			wModalities[m->recurrentModality] = recurrentGuiIModality;
		}

		//Add the influence/learning slider to every modality
		for (std::map < IModality*, GuiIModality*>::iterator wMod = wModalities.begin(); wMod != wModalities.end(); wMod++)
		{
			GtkWidget* paramModBox = gtk_vbox_new(FALSE, 0);
			gtk_box_pack_start(GTK_BOX(wMod->second->box), paramModBox, FALSE, FALSE, 0);

			LabelledSlider sliderInf;
			sliderInf.allocate("Influence", &(m->modalitiesInfluence[wMod->first]), 0.0, 1.0, 0.1);
			gtk_box_pack_start(GTK_BOX(paramModBox), sliderInf.box, FALSE, FALSE, 0);
			LabelledSlider sliderLear;
			sliderLear.allocate("Learning", &(m->modalitiesLearning[wMod->first]), 0.0, 1.0, 0.1);
			gtk_box_pack_start(GTK_BOX(paramModBox), sliderLear.box, FALSE, FALSE, 0);

			gtk_widget_show(paramModBox);
		}

		//Display the map activity for every layer
		box_activity = gtk_hbox_new(TRUE, 0);
		gtk_box_pack_start(GTK_BOX(boxMainInfo), box_activity, FALSE, FALSE, 0);
		frames_activity.resize(m->L());
		pxBuf_activity.resize(m->L());

		for (int i = 0; i < frames_activity.size(); i++)
		{
			pxBuf_activity[i] = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, m->W(), m->H());
			frames_activity[i] = gtk_drawing_area_new();
			g_signal_connect(frames_activity[i], "expose-event", G_CALLBACK(paintActivity), pxBuf_activity[i]);
			gtk_widget_set_size_request(frames_activity[i], 200, 200);
			gtk_box_pack_start(GTK_BOX(box_activity), frames_activity[i], TRUE, TRUE, 5);
			gtk_widget_show(frames_activity[i]);
		}
		gtk_widget_show(box_activity);
	}

	virtual void refreshElements()
	{
		this->GuiICvz::refreshElements();
		CvzMMCM* m = getCvz();
		//In all cases we refresh their activity
		for (int i = 0; i < frames_activity.size(); i++)
		{
			yarp::sig::ImageOf<yarp::sig::PixelRgb> img = m->getLayerActivity(i);
			yarpImage2Pixbuf(&img, pxBuf_activity[i]);
			gtk_widget_queue_draw(frames_activity[i]);
		}
	}

	CvzMMCM* getCvz() { return (CvzMMCM *)myCvz; }

public:

	GuiMMCM(IConvergenceZone* linkedCvz) :GuiICvz(linkedCvz)
	{
	}

};

#endif
