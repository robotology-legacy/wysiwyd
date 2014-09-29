#ifndef __CVZ_HELPERS_H__
#define __CVZ_HELPERS_H__

#include<sstream>
#include<vector>
#include<math.h>
#include<algorithm>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

namespace cvz {
	namespace helpers {
		#define My_PI 3.14159265358979323846
		std::string int2str(int i);
		yarp::sig::PixelRgb double2RGB(double value);
		double MexicanHat(const double &x, const double &sigma);
		double GaussianBell(const double &x, const double &sigma);
		double tanhx(double x);
		double sigmoidFunction(double x);
		double sigmoidFunction(double x, double alpha, double beta);
		void Clamp(double &value, const double &_min, const double &_max);
		
		//template<class pxT>
		//yarp::sig::ImageOf<pxT> getImgSubRegion(const yarp::sig::ImageOf<pxT> &img, int ulx, int uly, int lrx, int lry);
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

        /*
        void fillRnd(yarp::sig::Matrix &m)
        {
            for (int x = 0; x < m.rows(); x++)
            {
                for (int y = 0; y < m.cols(); y++)
                {
                    m[x][y] = yarp::os::Random::uniform();
                }
            }
        }

        yarp::sig::Vector std2yarpVector(std::vector<double> v)
        {
            yarp::sig::Vector v2(v.size());
            for (int i = 0; i < v.size(); i++)
                v2[i] = v[i];
            return v2;
        }

        std::vector<double> yarp2stdVector(yarp::sig::Vector v)
        {
            std::vector<double> v2(v.size());
            for (int i = 0; i < v.size(); i++)
                v2[i] = v[i];
            return v2;
        }*/

		struct Cube : public std::vector< std::vector< std::vector< double > > >
		{
			Cube(){}

			Cube(int w, int h, int l)
			{
				allocate(w, h, l);
			}

			void allocate(int w, int h, int l)
			{
				this->resize(w);
				for (int x = 0; x < w; x++)
				{
					this->operator[](x).resize(h);
					for (int y = 0; y < h; y++)
					{
						this->operator[](x)[y].resize(l);
					}
				}
			}

			void randomize(const double &min = 0.0, const double &max = 1.0)
			{
				for (unsigned int x = 0; x < this->size(); x++)
				{
					for (unsigned int y = 0; y < this->operator[](x).size(); y++)
					{
						for (unsigned int z = 0; z < this->operator[](x)[y].size(); z++)
						{
							this->operator[](x)[y][z] = yarp::os::Random::uniform() * (max - min) - min;
						}
					}
				}
			}

			void operator=(const double &a)
			{
				for (unsigned int x = 0; x < this->size(); x++)
				{
					for (unsigned int y = 0; y < this->operator[](x).size(); y++)
					{
						for (unsigned int z = 0; z < this->operator[](x)[y].size(); z++)
						{
							this->operator[](x)[y][z] = a;
						}
					}
				}
			}
		};
	}
}
#endif
