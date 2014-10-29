#ifndef __CVZ_CVZTEMPLATE_H__
#define __CVZ_CVZTEMPLATE_H__

#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <queue>
#include <float.h>

#include "ICvz.h"
#include "cvz/helpers/helpers.h"

namespace cvz {
    namespace core {

        class CvzTemplate : public IConvergenceZone
        {
            yarp::os::Semaphore mutex;

        public:

            virtual std::string getType() { return cvz::core::TYPE_MMCM; };


            virtual bool close()
            {
                bool ok = this->IConvergenceZone::close();
                return ok;
            }

            virtual bool configure(yarp::os::Property &rf)
            {
                //Call the base class configure
                this->IConvergenceZone::configure(rf);
                return true;
            }

            virtual void performPeriodicAction(const int &cyclesElapsed)
            {
                this->IConvergenceZone::performPeriodicAction(cyclesElapsed);
                if (cyclesElapsed % 500 == 0)
                {
                    std::cout << "Do something on 500 step" << std::endl;
                }
            }

            virtual void ComputePrediction()
            {
                mutex.wait();

                //------------------------------------------------------------------------------------------------------------------------
                //Set the predicted values 

                ////feedback
                //for (std::map<std::string, IModality*>::iterator it = modalitiesBottomUp.begin(); it != modalitiesBottomUp.end(); it++)
                //    predictModality(it->second, algorithm);
                ////feedforward
                //for (std::map<std::string, IModality*>::iterator it = modalitiesTopDown.begin(); it != modalitiesTopDown.end(); it++)
                //    predictModality(it->second, algorithm);

                mutex.post();
            }
        };
    }
}
#endif
