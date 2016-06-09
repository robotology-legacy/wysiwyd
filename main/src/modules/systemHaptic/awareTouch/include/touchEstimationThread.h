/* 
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Hector Barron-Gonzalez (ported by Mathew Evans and Uriel Martinez)
 * email:   mat.evans@sheffield.ac.uk
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * wysiwyd/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#if __clang__
#pragma clang diagnostic pop
#endif
#include <iCub/skinDynLib/skinContactList.h>
#include <iCub/learningMachine/LSSVMLearner.h>
#include <iCub/learningMachine/FixedRangeScaler.h>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <time.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::learningmachine;

#define THREAD_STATUS_UNTOUCHED             1
#define THREAD_STATUS_FEELING               2
#define THREAD_STATUS_TOUCHED               3

// Parameters Classifiers
#define DIM_FEATURE                     4
#define REG_CTE                         5.0
#define GAMMA_ML                        3.0
#define THRESHOLDFORCECONTACT           1.5  // 4.0 for black icub

class TouchEstimationThread: public RateThread
{
    private:

      //-- General thread variables
      int nGestures;                        // Number of gestures
      vector<string> gestureStrSet;         // Set of gestures names
      BufferedPort<iCub::skinDynLib::skinContactList> *port_skin_contacts;
      
      string partTouch;                     // Body part touched              
      int typeTouch;                        // Type of gesture
      int stateTouch ;                      // State of thread
      double startTime;                     // Starting time of touch
      double totalTime;                     // Touching time
      
      ofstream outputTouch;                 // File output "Touching.txt"
      ifstream inputTouch;                  // Gestures file input 
      string pathG;                         // Gestures source path 
     
      //-- Classifier variables
      vector<LSSVMLearner> classifiers;     // Ensamble of classifiers
      FixedRangeScaler scaler[4];           // Scaler
      
      //-- Temporal varables for features, per body part (Right, Left, Torso)
      Vector nContact,nContactL,nContactT;             // Number of contacts
      Vector meanForce,meanForceL,meanForceT;          // Mean contact force
      Vector meanSize,meanSizeL,meanSizeT;             // Mean contact size 
      Vector relPosition,relPositionL,relPositionT;    // Mean contact traslation
      Vector absPositionR, absPositionL, absPositionT; // Contact cartesian position 
      int counterTouchR,counterTouchL,counterTouchT;   // Number of valid contacts
      
      //-- Motor control variables per body part 
      IEncoders *encCtrlLeft, *encCtrlRight, *encCtrlTorso; // Encoders
      PolyDriver *polyHandRight, *polyHandLeft, *polyTorso; // Drivers
      iCubArmDyn *armTorsoDyn, *armTorsoDynR;               // iCub dyn arm for kinematics
      Property * optionEncs; 

      Vector liveTouchPosition;
      double liveTouchSize;

    public:
        TouchEstimationThread(string, string, string, vector<string >, int);
        bool threadInit();
        void run();
        void threadRelease();
        void getTactileInformation(iCub::skinDynLib::skinContactList, Vector& , Vector&, Vector&, Vector&, iCubArmDyn *, Vector &, int &, bool);
        int decideTouch(Vector, Vector, Vector, Vector);
        void bodyPartTouched(string &, int &, Vector &,double &);
        void trainClass();
        void computeFingerPosition();
};


