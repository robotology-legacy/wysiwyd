#ifndef __CVZ_IMODALITY_H__
#define __CVZ_IMODALITY_H__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <vector>
#include <string>
#include <iostream>
#include "helpers.h"

class IModality
{
protected:
	std::string name;
	int size;
	yarp::os::Semaphore mutex;

	std::vector<double> minBound;
	std::vector<double> maxBound;
	virtual bool input(){ return true;/*overload this*/ };
	virtual void output(){/*overload this*/ };

	std::vector<double> scaledValueReal;
	std::vector<double> scaledValuePrediction;
	std::vector<double> valueReal;
	std::vector<double> valuePrediction;

public:
	std::string Name() { return name; }
	int Size() { return size; }
	std::vector<double> GetValueReal() { return scaledValueReal; }
	std::vector<double> GetValuePrediction() { return scaledValuePrediction; }

	void SetValueReal(std::vector<double> b) {
		mutex.wait(); 
		scaledValueReal = b;
		mutex.post();
	}

	void SetValuePrediction(std::vector<double> b) 
	{ 
		mutex.wait(); 
		scaledValuePrediction = b;
		mutex.post();
	}

	IModality(std::string _name, int _size, std::vector<double> min, std::vector<double> max)
	{
		name = _name;
		size = _size;
		minBound = min;
		maxBound = max;
		scaledValueReal.resize(size);
		scaledValuePrediction.resize(size);
		valueReal.resize(size);
		valuePrediction.resize(size);
	}

	void Input()
	{
		mutex.wait();
		if (input())
		{
			//Scaling in [0,1]
			for (int i = 0; i < size; i++)
			{
				scaledValueReal[i] = (valueReal[i] - minBound[i]) / (maxBound[i] - minBound[i]);
				Clamp(scaledValueReal[i], 0.0, 1.0); //Clamp when we read out of boundaries values
			}
		}
		mutex.post();
	}

	void Output()
	{		
		mutex.wait();
		//Scaling from [0,1]
		for(int i=0;i<size;i++)
		{
			valuePrediction[i] = scaledValuePrediction[i] * (maxBound[i] - minBound[i]) + minBound[i];
			Clamp(valuePrediction[i], minBound[i], maxBound[i]); //useless in theory
		}

		output();
		mutex.post();
	}
	virtual void Close() {};
	virtual void Interrupt() {};

	virtual yarp::sig::ImageOf<yarp::sig::PixelRgb> getVisualization(bool getPredicted = false)
	{
		mutex.wait();
		yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
		img.resize(size, 1);
		for (int x = 0; x < size; x++)
		{
			if (getPredicted)
				img.pixel(x, 0) = double2RGB(scaledValuePrediction[x]);
			else
				img.pixel(x, 0) = double2RGB(scaledValueReal[x]);

		}
		mutex.post();
		//Console.WriteLine("Visualization computed in " + (time2 - time1).ToString());
		return img;
	}
};

template <class T>
class ModalityBufferedPort: public IModality
{
	std::vector<bool> mask;
	yarp::os::BufferedPort<T> portReal;
	yarp::os::BufferedPort<T> portPrediction;
	yarp::os::BufferedPort<yarp::os::Bottle> portError;
    bool isBlocking;

public:

    ModalityBufferedPort(std::string _name, int _size, std::vector<double> min, std::vector<double> max, std::vector<bool> _mask , bool _isBlocking = false):IModality(_name, _size,min, max)
	{
        isBlocking = _isBlocking;
		mask=_mask;
		//if no mask is provided we build one that allows all
		if(mask.size()==0)
			for(int i=0;i<size;i++)
				mask.push_back(true);

		valueReal.resize(size);
		valuePrediction.resize(size);
		std::string pName = name;
		pName += "/real:i";
		portReal.open(pName.c_str());
		
		pName =  name;
		pName += "/prediction:o";
		portPrediction.open(pName.c_str());

		pName = name;
		pName += "/error:o";
		portError.open(pName.c_str());
	}

	bool ConnectInput(std::string from)
	{
		return yarp::os::Network::connect(from.c_str(), portReal.getName());
	}

	bool ConnectOutput(std::string to)
	{
		return yarp::os::Network::connect(portPrediction.getName(), to.c_str());
	}

	void Interrupt() { portReal.interrupt(); portPrediction.interrupt(); }
	void Close() { portReal.close(); portPrediction.close(); }

	bool input()
	{
        T* in = portReal.read(isBlocking);
		if(in)
		{
			valueReal = Vectorize(in);
			return true;
		}
		return false;
	}

	void output()
	{
		T& bOut = portPrediction.prepare();
		bOut = Unvectorize(valuePrediction);
		portPrediction.write();

		//Compute the error
		double error = 0.0;
		
		for (int i = 0; i < scaledValuePrediction.size(); i++)
		{
			error += fabs(scaledValuePrediction[i] - scaledValueReal[i]);
		}
		error /= scaledValuePrediction.size();
		yarp::os::Bottle &bError = portError.prepare();
		bError.clear();
		bError.addDouble(error);
		portError.write();
	}

	T Unvectorize(std::vector<double>)
	{
		std::cout<<portReal.getName()<<"-----> Warning: this output type is not implemented."<<std::endl;
		T dummyReturn;
		return dummyReturn;
	}

	std::vector<double> Vectorize(T* input)
	{
		std::cout<<portReal.getName()<<"-----> Warning: this input type is not implemented."<<std::endl;
		std::vector<double> dummyReturn;
		dummyReturn.resize(size);
		return dummyReturn;
	}

	virtual yarp::sig::ImageOf<yarp::sig::PixelRgb> getVisualization(bool getPredicted)
	{
		mutex.wait();
		yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
		img.resize(size, 1);
		for (int x = 0; x < size; x++)
		{
			if (getPredicted)
				img.pixel(x, 0) = double2RGB(scaledValuePrediction[x]);
			else
				img.pixel(x, 0) = double2RGB(scaledValueReal[x]);

		}
		mutex.post();
		//Console.WriteLine("Visualization computed in " + (time2 - time1).ToString());
		return img;
	}
};

//[specialization] Bottle 
template<>
yarp::os::Bottle ModalityBufferedPort<yarp::os::Bottle>::Unvectorize(std::vector<double> output)
{
	yarp::os::Bottle b;
	int cnt = 0;
	for (int i = 0; i<mask.size(); i++)
	{
		if (mask[i])
		{
			b.addDouble(output[cnt]);
			cnt++;
		}
	}
	return b;
}

template<>
std::vector<double> ModalityBufferedPort<yarp::os::Bottle>::Vectorize(yarp::os::Bottle* input)
{
	std::vector<double> v;
	v.resize(size);

	int cnt = 0;
	for (int i = 0; i<mask.size(); i++)
	{
		if (i>input->size())
		{
			std::cout << portReal.getName() << "-----> Warning: received smaller input..." << std::endl;
			return v;
		}
		if (mask[i])
		{
			v[cnt] = input->get(i).asDouble();
			cnt++;
		}
	}
	return v;
}

//[specialization] ImageOf<PixelRGB>
template<>
yarp::sig::ImageOf<yarp::sig::PixelRgb> ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::Unvectorize(std::vector<double> output)
{
	yarp::sig::ImageOf<yarp::sig::PixelRgb> img;// = portPrediction.prepare();
	int desiredWidth = (int)sqrt((double)size);
	img.resize(desiredWidth, desiredWidth);

	int cnt = 0;
	for (int i = 0; i<mask.size(); i++)
	{
		if (mask[i])
		{
			int x = i % img.width();
			int y = i / img.width();
			if (x < img.width() && y < img.height())
			{
				img.pixel(x, y).r = (unsigned char)(output[cnt] * 255.0);
				img.pixel(x, y).g = (unsigned char)(output[cnt] * 255.0);
				img.pixel(x, y).b = (unsigned char)(output[cnt] * 255.0);
			}
			else
			{
				//std::cout << "Debug: cropping a part of the image?" << std::endl;
			}
			cnt++;
		}
	}
	return img;
}

template<>
std::vector<double> ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::Vectorize(yarp::sig::ImageOf<yarp::sig::PixelRgb>* input)
{
	std::vector<double> v;
	v.resize(size);
	yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
	int desiredWidth = (int)sqrt((double)size);
	img.copy(*input, desiredWidth, desiredWidth);
	//input->resize(size / 2, size - size / 2);

	int cnt = 0;
	for (int i = 0; i<mask.size(); i++)
	{
		if (i>img.width() * img.height())
		{
			std::cout << portReal.getName() << "-----> Warning: This should not happen (img resolution is too large)" << std::endl;
			return v;
		}
		if (mask[i])
		{
			int x = i % img.width();
			int y = i / img.width();
			if (x < img.width() && y < img.height())
			{
				v[cnt] = (img.pixel(x, y).r + img.pixel(x, y).g + img.pixel(x, y).b) / (3 * 255.0);
			}
			else
			{
				//std::cout << "Debug: cropping a part of the image?" << std::endl;
			}
			cnt++;
		}
	}
	return v;
}

template<>
yarp::sig::ImageOf<yarp::sig::PixelRgb> ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::getVisualization(bool getPredicted)
{
	mutex.wait();
	yarp::sig::ImageOf<yarp::sig::PixelRgb> img;

	if (getPredicted)
		img = Unvectorize(valuePrediction);
	else
		img = Unvectorize(valueReal);
	mutex.post();
	//Console.WriteLine("Visualization computed in " + (time2 - time1).ToString());
	return img;
}

//[specialization] ImageOf<PixelFloat>
template<>
yarp::sig::ImageOf<yarp::sig::PixelFloat> ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >::Unvectorize(std::vector<double> output)
{
	yarp::sig::ImageOf<yarp::sig::PixelFloat> img;// = portPrediction.prepare();
	img.resize(size / 2, size - size / 2);

	int cnt = 0;
	for (int i = 0; i<mask.size(); i++)
	{
		if (mask[i])
		{
			img.pixel(i%img.width(), i / img.width()) = output[cnt];
			cnt++;
		}
	}
	return img;
}

template<>
std::vector<double> ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >::Vectorize(yarp::sig::ImageOf<yarp::sig::PixelFloat>* input)
{
	std::vector<double> v;
	v.resize(size);
	yarp::sig::ImageOf<yarp::sig::PixelFloat> img;
	img.copy(*input, size / 2, size - size / 2);

	int cnt = 0;
	for (int i = 0; i<mask.size(); i++)
	{
		if (i>input->width() * input->height())
		{
			std::cout << portReal.getName() << "-----> Warning: This should not happen (img resolution is too large)" << std::endl;
			return v;
		}
		if (mask[i])
		{
			int x = i%input->width();
			int y = i / input->width();
			v[cnt] = input->pixel(x, y);
			cnt++;
		}
	}
	return v;
}
#endif
