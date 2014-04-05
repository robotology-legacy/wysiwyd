#ifndef __CVZ_HELPERS_H__
#define __CVZ_HELPERS_H__

#include<sstream>
#include<vector>
#include<math.h>
#include<algorithm>
#include <yarp/os/all.h>

namespace cvz {
	namespace helpers {
#define My_PI 3.14159265358979323846

		std::string int2str(int i)
		{
			std::stringstream ss;
			ss << i;
			return ss.str();
			//char buffer[255];
			//itoa(i, buffer, 10);
			//return buffer;
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
						this->operator[](x)[y].resize(h);
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

		void Clamp(double &value, const double &_min, const double &_max)
		{
			value = std::min(_max, std::max(_min, value));
		}
	}
}
#endif
