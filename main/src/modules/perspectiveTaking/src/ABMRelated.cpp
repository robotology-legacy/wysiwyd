#include "PerspectiveTaking.h"

using namespace std;
using namespace yarp::os;

bool perspectiveTaking::sendImagesToPorts() {
    cv::Mat screen = mapBuilder->getScreen();

    cv::Mat selfPersp, partnerPersp;
    // self perspective = left side of screenshot
    // partner perspective = right side of screenshot
    selfPersp=screen(cv::Rect(0,0,screen.cols/2,screen.rows));
    partnerPersp=screen(cv::Rect(screen.cols/2,0,screen.cols/2,screen.rows));
    //cv::imshow("Self Perspective", selfPersp);
    //cv::imshow("Partner Perspective", partnerPersp);
    //cv::waitKey(50);

    // convert cv::Mat to IplImage, and copy IplImage to ImageOf<PixelRGB>
    IplImage* partnerPersp_ipl = new IplImage(partnerPersp);
    ImageOf<PixelRgb> &partnerPers_yarp = partnerPerspImgPort.prepare();
    partnerPers_yarp.resize(partnerPersp_ipl->width, partnerPersp_ipl->height);
    cvCopy(partnerPersp_ipl, (IplImage *)partnerPers_yarp.getIplImage());

    IplImage* selfPersp_ipl = new IplImage(selfPersp);
    ImageOf<PixelRgb> &selfPers_yarp = selfPerspImgPort.prepare();
    selfPers_yarp.resize(selfPersp_ipl->width, selfPersp_ipl->height);
    cvCopy(selfPersp_ipl, (IplImage *)selfPers_yarp.getIplImage());

    //send the images
    selfPerspImgPort.write();
    partnerPerspImgPort.write();

    return true;
}

bool perspectiveTaking::addABMImgProvider(const string &portName, bool addProvider) {
    Bottle bCmd, bReply;
    if(addProvider) {
        bCmd.addString("addImgStreamProvider");
    } else {
        bCmd.addString("removeImgStreamProvider");
    }
    bCmd.addString(portName);

    abm.write(bCmd, bReply);

    if(bReply.get(0).toString()=="[ack]") {
        return true;
    } else {
        return false;
    }
}
