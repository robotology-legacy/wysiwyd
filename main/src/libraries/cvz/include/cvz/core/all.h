#include "IModality.h"
#include "ICvz.h"
#include "CvzMMCM.h"
#include "CvzESOM.h"
#include "CvzMLP.h"
#include "CvzBuilder.h"
#include "CvzStack.h"
#include "CvzFiber.h"

/**
*
* @defgroup cvz_library Convergence Zone Library
* @ingroup wysiwyd_libraries 
*  
* This library contains a set of classes for dealing with the Convergence Zone Framework.
* It basically allows to run an abstract convergence point of several modalities (i.e yarp ports)
* and then apply an algorithm to predict back those modalities.
* The algorithm to be applied is defined by inheriting from the abstract IConvergenceZone class.
* \n 
*/