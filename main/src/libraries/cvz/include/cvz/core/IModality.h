#ifndef __CVZ_IMODALITY_H__
#define __CVZ_IMODALITY_H__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <vector>
#include <string>
#include <iostream>
#include "cvz/helpers/helpers.h"
namespace cvz {
    namespace core {

#define MASKED_ELEMENT_VALUE 0.0

        /**
        * \ingroup cvz_library
        * Class representing an abstract modality. Consists of two vectors of double, one for the real input and one for the predicted output.
        */
        class IModality
        {
        protected:
            std::string name;
            int size;
            yarp::os::Semaphore mutex;

            std::vector<double> minBound;
            std::vector<double> maxBound;
			bool autoScale;

            std::vector<double> scaledValueReal;
            std::vector<double> scaledValuePrediction;
            std::vector<double> valueReal;
            std::vector<double> valuePrediction;
			
            /**
            * virtual method : Defines what a modality does upon input signal. Should be overloaded and is called by IModality::Input()
            * @return true/false in case of success failure
            */
            virtual bool input(){ return true;/*overload this*/ };

            /**
            * virtual method : Defines what a modality does upon output signal. Should be overloaded and is called by IModality::Output()
            * @return true/false in case of success failure
            */
            virtual void output(){/*overload this*/ };

        public:
            /**
            * Retrieve the name of the module. Including the name of the CVZ it belongs to (e.g /cvzName/modalityName)
            */
            std::string Name() { return name; }

            /**
            * Retrieve the name of the modality. Including the name of the CVZ it belongs to (e.g /cvzName/modalityName)
            */
            virtual std::string GetFullName(){ return name; }

            /**
            * Retrieve the name of the modality/real. Including the name of the CVZ it belongs to (e.g /cvzName/modalityName/real)
            * In case of using a BufferedPort modality this command gives your the port name.
            */
            virtual std::string GetFullNameReal(){ return name + "/real"; }

            /**
            * Retrieve the name of the modality/prediction. Including the name of the CVZ it belongs to (e.g /cvzName/modalityName/prediction).
            * In case of using a BufferedPort modality this command gives your the port name.
            */
            virtual std::string GetFullNamePrediction(){ return name + "/prediction"; }

            /**
            * Retrieve the size of the modality.
            */
            int Size() { return size; }

            /**
            * Retrieve the current value of the real input.
            * @return A vector of double
            */
            std::vector<double> GetValueReal() { return scaledValueReal; }

            /**
            * Retrieve the current value of the prediction output.
            * @return A vector of double
            */
            std::vector<double> GetValuePrediction() { return scaledValuePrediction; }

            /**
            * Set the current value of the real input. This method can be used to work with IModality from the code.
            * @param b A vector of double. 
            */
            void SetValueReal(std::vector<double> b) {
                mutex.wait();
                scaledValueReal = b;
                mutex.post();
            }

            /**
            * Set the current value of the prediction output.
            * @parameter b A vector of double
            */
            void SetValuePrediction(std::vector<double> b)
            {
                mutex.wait();
                scaledValuePrediction = b;
                mutex.post();
            }

            /**
            * Instantiate a new IModality.
            * @parameter _name The name of the modality, including any prefix you want to use (like the cvz name)
            * @parameter _size The size of the modality (i.e the number of components of the vectors)
            * @parameter min The minimum limits of the input space, are used internally for scaling in [0,1].
            * @parameter max The maximum limits of the modality, are used internally for scaling in [0,1].
            * @parameter _autoScale If true the maximum/minimum boudaries will adapt to the input.
            */
            IModality(std::string _name, int _size, std::vector<double> min, std::vector<double> max, bool _autoScale)
            {
                name = _name;
                size = _size;
                minBound = min;
                maxBound = max;
                scaledValueReal.resize(size);
                scaledValuePrediction.resize(size);
                valueReal.resize(size);
                valuePrediction.resize(size);
				autoScale = _autoScale;
            }

			/**
			* Apply the boundaries provided during the construction to scale a vector in [0,1].
			*/
			std::vector<double> scale(std::vector<double> tmp)
			{
				std::vector<double> sVal(tmp.size());
				if (tmp.size() != (unsigned int) size)
					return sVal;

				for (int i = 0; i < size; i++)
				{
					sVal[i] = (tmp[i] - minBound[i]) / (maxBound[i] - minBound[i]);
					helpers::Clamp(sVal[i], 0.0, 1.0); //Clamp when we read out of boundaries values
				}
				return sVal;
			}

			/**
			* Apply the boundaries provided during the construction to unscale a vector.
			*/
			std::vector<double> unscale(std::vector<double> tmp)
			{
				std::vector<double> sVal(tmp.size());
				if (tmp.size() != (unsigned int) size)
					return sVal;

				for (int i = 0; i < size; i++)
				{
					sVal[i] = tmp[i] * (maxBound[i] - minBound[i]) + minBound[i];
					helpers::Clamp(sVal[i], minBound[i], maxBound[i]); //useless in theory
				}
				return sVal;
			}

            /**
            * Trigger the Input mechanism of a modality. 
            * Calls the specific input() method (e.g calling read() in case of a BufferePort modality)
            * Handles the scaling of the input given the limits provided in the constructor.
            */
            void Input()
            {
                mutex.wait();
                if (input())
                {
                    //Scaling in [0,1]
                    for (int i = 0; i < size; i++)
                    {
						if (autoScale && valueReal[i]>maxBound[i])
                        {
							maxBound[i] = valueReal[i];
                            std::cout<<name<<" updating maximum boundary of component "<<i<<" to "<<maxBound[i]<<std::endl;
                        }
						if (autoScale && valueReal[i]<minBound[i])
                        {
							minBound[i] = valueReal[i];
                            std::cout<<name<<" updating minimum boundary of component "<<i<<" to "<<minBound[i]<<std::endl;
                        }
                        scaledValueReal[i] = (valueReal[i] - minBound[i]) / (maxBound[i] - minBound[i]);
                        helpers::Clamp(scaledValueReal[i], 0.0, 1.0); //Clamp when we read out of boundaries values
                    }
                }
                mutex.post();
            }

            /**
            * Trigger the Output mechanism of a modality.
            * Calls the specific output() method (e.g calling write() in case of a BufferePort modality)
            * Handles the scaling of the output given the limits provided in the constructor.
            */
            void Output()
            {
                mutex.wait();
                //Scaling from [0,1]
                for (int i = 0; i < size; i++)
                {
                    valuePrediction[i] = scaledValuePrediction[i] * (maxBound[i] - minBound[i]) + minBound[i];
                    helpers::Clamp(valuePrediction[i], minBound[i], maxBound[i]); //useless in theory
                }

                output();
                mutex.post();
            }

            /**
            * Closes modality.
            * This is the place to clean the stuff (close ports)
            */
            virtual void Close() {};

            /**
            * Interrupts modality.
            * This is the place to clean the stuff (interrupt ports)
            */
            virtual void Interrupt() {};

			/**
			* Returns an image of the modality current value to be displayed.
			* Specialization can be provided to customize the display (e.g display the image in unvectorized form)
			* @param getPredicted Choose if you want the real value (false) or the predicted value (true)
			* @return An image representing the vector with 0 being blue, 0.5 being green and 1.0 being red.
			*/
			yarp::sig::ImageOf<yarp::sig::PixelRgb> getVisualization(bool getPredicted = false)
			{
				if (getPredicted)
					return getVisualizationFromVector(scaledValuePrediction);
				else
					return getVisualizationFromVector(scaledValueReal);
			}

			/**
			* Returns an image of an arbitrary vector to be displayed.
			* Specialization can be provided to customize the display (e.g display the image in unvectorized form)
			* @param values, the values to be displayed
			* @return An image representing the vector with 0 being blue, 0.5 being green and 1.0 being red.
			*/
			virtual yarp::sig::ImageOf<yarp::sig::PixelRgb> getVisualizationFromVector(std::vector<double> values)
			{
				mutex.wait();
				yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
				img.resize(size, 1);
				for (int x = 0; x < size; x++)
				{
					img.pixel(x, 0) = helpers::double2RGB(values[x]);
				}
				mutex.post();
				//Console.WriteLine("Visualization computed in " + (time2 - time1).ToString());
				return img;
			}
        };

        /**
        * \ingroup cvz_library
        * Template class representing an modality using BufferedPort<T>.
        * Open 2 buffered ports, one for the real input and one for the prediction output.
        */
        template <class T>
        class ModalityBufferedPort : public IModality
        {
            std::vector<bool> mask;
            yarp::os::BufferedPort<T> portReal;
            yarp::os::BufferedPort<T> portPrediction;
            yarp::os::BufferedPort<yarp::os::Bottle> portError;
            bool isBlocking;

        public:

            /**
            * Retrieve the name of the modality/real port.
            */
            virtual std::string GetFullNameReal(){ return portReal.getName(); }

            /**
            * Retrieve the name of the modality/prediction port.
            */
            virtual std::string GetFullNamePrediction(){ return portPrediction.getName(); }

            /**
            * Instantiate a new BufferedPortModality<T>.
            * @parameter _name The name of the modality, including any prefix you want to use (like the cvz name)
            * @parameter _size The size of the modality (i.e the number of components of the vectors)
            * @parameter min The minimum limits of the input space, are used internally for scaling in [0,1].
            * @parameter max The maximum limits of the modality, are used internally for scaling in [0,1].
            * @parameter _mask Apply a mask on the input got through input(). The number of True of the mask should match the size of the modality. Providing an empty vector (size==0) will ensure that your do not apply any mask and get everything.
            * @parameter _autoScale Should the boudaries auto adapt to the input?.           
			* @parameter _isBlocking Specify is the ports will use blocking read in the input() method.
            */
			ModalityBufferedPort(std::string _name, int _size, std::vector<double> min, std::vector<double> max, std::vector<bool> _mask, bool _autoScale, bool _isBlocking = false) :IModality(_name, _size, min, max, _autoScale)
            {
                isBlocking = _isBlocking;
                mask = _mask;
                //if no mask is provided we build one that allows all
                if (mask.size() == 0)
                for (int i = 0; i < size; i++)
                    mask.push_back(true);

                valueReal.resize(size);
                valuePrediction.resize(size);
                std::string pName = name;
                pName += "/real:i";
                portReal.open(pName.c_str());

                pName = name;
                pName += "/prediction:o";
                portPrediction.open(pName.c_str());

                pName = name;
                pName += "/error:o";
                portError.open(pName.c_str());
            }

            /**
            * Connects a port to the real input port
            * @param from The name of the port you want to read from
            * @return true/false in case of success/failure
            */
            bool ConnectInput(std::string from)
            {
                return yarp::os::Network::connect(from.c_str(), portReal.getName());
            }

            /**
            * Connects the prediction output to another port
            * @param to The name of the port you want to write to
            * @return true/false in case of success/failure
            */
            bool ConnectOutput(std::string to)
            {
                return yarp::os::Network::connect(portPrediction.getName(), to.c_str());
            }

            /**
            * Interrupts modality (interrupts ports)
            */
            void Interrupt() { portReal.interrupt(); portPrediction.interrupt(); }

            /**
            * Closes modality (closes ports)
            */
            void Close() { portReal.close(); portPrediction.close(); }

            /**
            * Call the read method on the input port and vectorize the value read.
            * @return true/false in case of data/nodata available
            */
            bool input()
            {
                T* in = portReal.read(isBlocking);
                if (in)
                {
                    valueReal = Vectorize(in);
                    return true;
                }
                return false;
            }

            /**
            * Unvectorize the predicted value and call the write method on the output port.
            */
            void output()
            {
                T& bOut = portPrediction.prepare();
                bOut = Unvectorize(valuePrediction);
                portPrediction.write();

                //Compute the error
                double error = 0.0;

                for (unsigned int i = 0; i < scaledValuePrediction.size(); i++)
                {
                    error += fabs(scaledValuePrediction[i] - scaledValueReal[i]);
                }
                error /= scaledValuePrediction.size();
                yarp::os::Bottle &bError = portError.prepare();
                bError.clear();
                bError.addDouble(error);
                portError.write();
            }

            /**
            * Unvectorize a vector to the template type used. Providing back the original format from the "neural representation"
            * A specialization of this method has to be implemented for this specific type.
            * @param output The vector to be transformed to the template T
            * @return A representation of type T.
            */
            T Unvectorize(std::vector<double> output)
            {
                std::cout << portReal.getName() << "-----> Warning: this output type is not implemented." << std::endl;
                T dummyReturn;
                return dummyReturn;
            }

            /**
            * Vectorize the template type used, providing a "neural representation" as a vector of double (e.g linearize an image)
            * A specialization of this method has to be implemented for this specific type.
            * @param input An instance of the template T to be vectorized
            * @return A vector of double equivalent to the input.
            */
            std::vector<double> Vectorize(T* input)
            {
                std::cout << portReal.getName() << "-----> Warning: this input type is not implemented." << std::endl;
                std::vector<double> dummyReturn;
                dummyReturn.resize(size);
                return dummyReturn;
            }

			/**
			* Returns an image of an arbitrary vector to be displayed.
			* Specialization can be provided to customize the display (e.g display the image in unvectorized form)
			* @param values, the values to be displayed
			* @return An image representing the vector with 0 being blue, 0.5 being green and 1.0 being red.
			*/
			virtual yarp::sig::ImageOf<yarp::sig::PixelRgb> getVisualizationFromVector(std::vector<double> values)
			{
				mutex.wait();
				yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
				img.resize(size, 1);
				for (int x = 0; x < size; x++)
				{
					img.pixel(x, 0) = helpers::double2RGB(values[x]);
				}
				mutex.post();
				//Console.WriteLine("Visualization computed in " + (time2 - time1).ToString());
				return img;
			}
        };

        /**
        * Unvectorize() specialization for yarp::os::Bottle
        */
        template<>
        yarp::os::Bottle ModalityBufferedPort<yarp::os::Bottle>::Unvectorize(std::vector<double> output)
        {
            yarp::os::Bottle b;
            int cnt = 0;
            for (unsigned int i = 0; i < mask.size(); i++)
            {
                if (mask[i])
                {
                    b.addDouble(output[cnt]);
                    cnt++;
                }
                else
                {
                    b.addDouble(MASKED_ELEMENT_VALUE); // This is a value by default for the masked elements
                }
            }
            return b;
        }

        /**
        * Vectorize() specialization for yarp::os::Bottle
        */
        template<>
        std::vector<double> ModalityBufferedPort<yarp::os::Bottle>::Vectorize(yarp::os::Bottle* input)
        {
            std::vector<double> v;
            v.resize(size);

            int cnt = 0;
            for (unsigned int i = 0; i<mask.size(); i++)
            {
                if (i>(unsigned int)input->size())
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

        /**
        * Unvectorize() specialization for yarp::sig::Imageof<yarp::sig::<PixelRgb> >
        */
        template<>
        yarp::sig::ImageOf<yarp::sig::PixelRgb> ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::Unvectorize(std::vector<double> output)
        {
            yarp::sig::ImageOf<yarp::sig::PixelRgb> img;// = portPrediction.prepare();
            int desiredWidth = (int)sqrt((double)size);
            img.resize(desiredWidth, desiredWidth);

            int cnt = 0;
            for (unsigned int i = 0; i < mask.size(); i++)
            {
                int x = i % img.width();
                int y = i / img.width();
                if (mask[i])
                {
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
                else
                {
                    img.pixel(x, y).r = (unsigned char)(MASKED_ELEMENT_VALUE);
                    img.pixel(x, y).g = (unsigned char)(MASKED_ELEMENT_VALUE);
                    img.pixel(x, y).b = (unsigned char)(MASKED_ELEMENT_VALUE);
                }
            }
            return img;
        }

        /**
        * Vectorize() specialization for yarp::sig::Imageof<yarp::sig::<PixelRgb> >
        */
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
            for (unsigned int i = 0; i<mask.size(); i++)
            {
                if (i> (unsigned int) (img.width() * img.height()) )
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

        /**
        * getVisualization() specialization for yarp::sig::Imageof<yarp::sig::<PixelRgb> >
        */
        template<>
		yarp::sig::ImageOf<yarp::sig::PixelRgb> ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::getVisualizationFromVector(std::vector<double> values)
        {
			return Unvectorize(values);
        }

        /**
        * Unvectorize() specialization for yarp::sig::Imageof<yarp::sig::<PixelFloat> >
        */
        template<>
        yarp::sig::ImageOf<yarp::sig::PixelFloat> ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >::Unvectorize(std::vector<double> output)
        {
            yarp::sig::ImageOf<yarp::sig::PixelFloat> img;// = portPrediction.prepare();
            img.resize(size / 2, size - size / 2);

            int cnt = 0;
            for (unsigned int i = 0; i < mask.size(); i++)
            {
                if (mask[i])
                {
                    img.pixel(i%img.width(), i / img.width()) = output[cnt];
                    cnt++;
                }
                else
                {
                    img.pixel(i%img.width(), i / img.width()) = MASKED_ELEMENT_VALUE;
                }
            }
            return img;
        }

        /**
        * Vectorize() specialization for yarp::sig::Imageof<yarp::sig::<PixelFloat> >
        */
        template<>
        std::vector<double> ModalityBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >::Vectorize(yarp::sig::ImageOf<yarp::sig::PixelFloat>* input)
        {
            std::vector<double> v;
            v.resize(size);
            yarp::sig::ImageOf<yarp::sig::PixelFloat> img;
            img.copy(*input, size / 2, size - size / 2);

            int cnt = 0;
            for (unsigned int i = 0; i<mask.size(); i++)
            {
                if (i>(unsigned int)(input->width() * input->height()))
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

        /**
        * Unvectorize() specialization for yarp::sig::Sound
        */
        template<>
        yarp::sig::Sound ModalityBufferedPort<yarp::sig::Sound>::Unvectorize(std::vector<double> output)
        {
            yarp::sig::Sound s;
            s.resize(size);
            s.setFrequency(4400);

            int cnt = 0;
            for (unsigned int i = 0; i < mask.size(); i++)
            {
                if (mask[i])
                {
                    s.set((int)(output[cnt] * 65535.0), i);
                    cnt++;
                }
                else
                {
                    s.set((int)(MASKED_ELEMENT_VALUE * 65535.0), i);
                }
            }
            return s;
        }

        /**
        * Vectorize() specialization for yarp::os::Bottle
        */
        template<>
        std::vector<double> ModalityBufferedPort<yarp::sig::Sound >::Vectorize(yarp::sig::Sound* input)
        {
            unsigned int samplesReceived = input->getSamples();
            unsigned int channels = input->getChannels();
            if (channels != 1)
                std::cout << portReal.getName() << "-----> Warning: received a sound on more than 1 channel. Only first one will be used." << std::endl;
            
            int bytePerSample = input->getBytesPerSample();
            if (bytePerSample != 2)
                std::cout << portReal.getName() << "-----> Warning: the samples are not encoded on 2 bytes." << std::endl;

            //We restrict to only one channel
            std::vector<double> v;
            v.resize(size);

            int cnt = 0;
            for (unsigned int i = 0; i<mask.size(); i++)
            {
                if (i>samplesReceived)
                {
                    std::cout << portReal.getName() << "-----> Warning: received smaller input..." << std::endl;
                    return v;
                }
                if (mask[i])
                {
                    v[cnt] = input->get(i) / 65535.0; // Encoded on 2 bytes
                    cnt++;
                }
            }
            return v;
        }
    }

}
#endif
