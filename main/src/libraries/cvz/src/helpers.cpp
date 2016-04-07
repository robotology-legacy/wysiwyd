#include "cvz/helpers/helpers.h"
namespace cvz {
    namespace helpers {

std::string int2str(int i)
{
    return std::to_string(i);
}

yarp::sig::PixelRgb double2RGB(double value)
{
    yarp::sig::PixelRgb px;
    double redValue = 1.0;
    double greenValue = 0.5;
    double blueValue = 0.0;

    if (value >= blueValue && value <= greenValue)
    {
        px.r = (unsigned char)(0);
        px.g = (unsigned char)(255 * (value - blueValue) / (greenValue - blueValue));
        px.b = (unsigned char)(255 * (value - greenValue) / (blueValue - greenValue));
    }
    else if (value > greenValue && value <= redValue)
    {
        px.r = (unsigned char)(255 * (value - greenValue) / (redValue - greenValue));
        px.g = (unsigned char)(255 * (value - redValue) / (greenValue - redValue));
        px.b = (unsigned char)(0);
    }
    else {    // should never happen - value > 1
        px.r = 0;
        px.g = 0;
        px.b = 0;
        std::cout << "Warning : You will take the black..." << std::endl;
    }
    return px;
}
/*
template<class pxT>
yarp::sig::ImageOf<pxT> getImgSubRegion(const yarp::sig::ImageOf<pxT> &img, int ulx, int uly, int lrx, int lry)
{
    yarp::sig::ImageOf<pxT> out;
    int w = abs(ulx - lrx);
    int h = abs(lry - uly);
    out.resize(w, h);
    for (int x = 0; x < w; x++)
    {
        for (int y = 0; y < h; y++)
        {
            out.pixel(x, y) = img.pixel(ulx + x, uly + y);
        }
    }
    return out;
}
*/
double MexicanHat(const double &x, const double &sigma)
{
    return
        1 / (sqrt(2 * My_PI*pow(sigma, 3.0))) *
        (1 - pow(x, 2.0) / pow(sigma, 2.0))  *
        exp(-pow(x, 2.0) / (2 * pow(sigma, 2.0)));
}

double GaussianBell(const double &x, const double &sigma)
{
    return
        exp(-pow(x, 2.0) / sigma);
}

double tanhx(double x) {
    return (exp(2 * x) - 1) / (exp(2 * x) + 1);
}

double sigmoidFunction(double x){
    return 1 / (1 + exp(-x));
}

double sigmoidFunction(double x, double alpha, double beta){
    return 1 / (1 + exp( (x - alpha ) * beta));
}

void Clamp(double &value, const double &_min, const double &_max)
{
    value = std::min(_max, std::max(_min, value));
}
    }
}
