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
		void Clamp(double &value, const double &_min, const double &_max);

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
