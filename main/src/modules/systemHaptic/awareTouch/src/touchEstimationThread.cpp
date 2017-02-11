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
//#include <iCub/iDyn/iDyn.h>
//#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContact.h>

#include <iostream>
#include <iomanip>
#include <string.h>
#include <sstream>
#include "touchEstimationThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace std;


TouchEstimationThread::TouchEstimationThread(string skinManagerPortName, string skinPortName, string pathGen, vector<string > gestureSet,  int period): RateThread (period)
 {
   cout<< "||  Creating Thread Touch estimation"<<endl;
   nGestures=gestureSet.size();
   gestureStrSet.resize(nGestures);
   
    // --------      Creates classifiers 
   cout<< "||  Creating classifiers ..."<<endl;
   classifiers.resize(nGestures);
   for (int iGesture=0; iGesture<nGestures;iGesture++){
        classifiers[iGesture] = LSSVMLearner(DIM_FEATURE,1,REG_CTE); 
        gestureStrSet[iGesture]=gestureSet[iGesture];
   }
   pathG=pathGen;
   scaler[0] = FixedRangeScaler(1, 10);
   scaler[1] = FixedRangeScaler(1, 200);
   scaler[2] = FixedRangeScaler(1, 100);
   scaler[3] = FixedRangeScaler(0, 1);
   trainClass();
    
    //----------    Open skin ports 
    cout<< "||  Opening ports ..."<<endl;
    port_skin_contacts = new BufferedPort<skinContactList>;
    port_skin_contacts->open(skinPortName.c_str());
    cout<< "||  Opened port in  Thread Touch estimat"<<endl;
    while(!Network::connect(skinManagerPortName.c_str(),skinPortName.c_str(),"tcp",false)) {
       cout<< "||  Trying to connect with skinManager"<<endl;
    }           
    cout<< "||  Connected with skinManager"<<endl;
    
 
   //----------    Configure encoder interfaces
   // Open driver for left arm kinematics
    optionEncs = new Property("(device remote_controlboard)");
    optionEncs->put("robot", "icub");
    optionEncs->put("remote", "/icub/left_arm");
    optionEncs->put("local", "/clientTouch/left_arm");

    polyHandLeft = new PolyDriver;
    if( !polyHandLeft->open(*optionEncs) )
        cout<<"No driver Left"<<endl;
    polyHandLeft->view(encCtrlLeft);
   
    cout<<"Driver created for reading"<<endl;

   // Openening driver for right arm kinematics
    optionEncs = new Property("(device remote_controlboard)");
    optionEncs->put("robot", "icub");
    optionEncs->put("remote", "/icub/right_arm");
    optionEncs->put("local", "/clientTouch/right_arm");

    polyHandRight = new PolyDriver;
    if( !polyHandRight->open(*optionEncs) )
        cout<<"No driver right"<<endl;
    polyHandRight->view(encCtrlRight);
   
    cout<<"Driver created for reading"<<endl;

    // Openening driver for torso kinematics
    delete optionEncs;
    optionEncs = new Property("(device remote_controlboard)");
    optionEncs->put("robot", "icub");
    optionEncs->put("remote", "/icub/torso");
    optionEncs->put("local", "/clientTouch/torso");

    polyTorso = new PolyDriver;
    if( !polyTorso->open(*optionEncs) )
        cout<<"No driver torso"<<endl;
    polyTorso->view(encCtrlTorso);
   
    cout<<"Driver created for reading"<<endl;


   // ---------  Configure iCub Arm oject 
   armTorsoDyn=new  iCubArmDyn("left");
   armTorsoDynR=new  iCubArmDyn("right");
   armTorsoDyn->releaseLink(0);
   armTorsoDyn->releaseLink(1);
   armTorsoDyn->releaseLink(2);

   armTorsoDynR->releaseLink(0);
   armTorsoDynR->releaseLink(1);
   armTorsoDynR->releaseLink(2);

}


void TouchEstimationThread::trainClass()
{
  Vector vecTrain;
  Vector vecOut;

  string stline;
  string featStr;
  string fileName;
  Vector vecTest;
  double dataFeat;

  // Feeding each gesture
  for (int iGesture=0;iGesture<nGestures;iGesture++) {
      // Opening file
      fileName=pathG+"/"+gestureStrSet[iGesture]+".txt";
      
      inputTouch.open(fileName.c_str());
      cout<<"opening file: "<< fileName<<endl;
      while ( inputTouch.good() )  
      {  // For each instance ...
         cout<<"Getting another gesture from file..."<<endl;
         vecTrain.clear();
        
         getline (inputTouch,stline);
         std::istringstream iss(stline);
         int iscaler=0;
         while(getline(iss, featStr, '\t'))    // extract each features
     {
        dataFeat=atof(featStr.c_str());
                dataFeat = scaler[iscaler].transform(dataFeat);  // scale the feature
                vecTrain.push_back(dataFeat);                    
                iscaler++;
                
     } // while (getline..
         if (vecTrain.size()==4) {   // if feature vector is valid, feed each classifier
            for (int jGesture=0;jGesture<nGestures;jGesture++){
                 vecOut.clear();
                if (jGesture==iGesture) {
                    vecOut.push_back(1);
                }else{
                    vecOut.push_back(-1);
                } //if-else
                classifiers[jGesture].feedSample(vecTrain,vecOut);
            }// for 
            
         }// if (vectrain ..


      }//while (inputTouch
      inputTouch.close();      
  }//for each gesture File

  // Actual training of classifiers
  
  for (int iGesture=0;iGesture<nGestures;iGesture++) {
   classifiers[iGesture].train();
 }
}//trainClass function
  




bool TouchEstimationThread::threadInit()
{
 // ---------  Initialize variables for gesture delivery
   liveTouchPosition.clear();
   liveTouchPosition.resize(3);
   liveTouchPosition.zero();
   liveTouchSize=0;
   typeTouch=-1;

 // ---------  Initialize variables for right arm
   nContact.clear();
   meanForce.clear();
   meanSize.clear();
   relPosition.clear();
   counterTouchR=0;

   absPositionR.resize(3);
   absPositionR.zero();
 // ---------  Initialize variables for left arm
   nContactL.clear();
   meanForceL.clear();
   meanSizeL.clear();
   relPositionL.clear();
   counterTouchL=0;

   absPositionL.resize(3);
   absPositionL.zero();

   // ---------  Initialize variables for torso
   nContactT.clear();
   meanForceT.clear();
   meanSizeT.clear();
   relPositionT.clear();
   counterTouchT=0; 

   absPositionT.resize(3);
   absPositionT.zero();

   // ------    General thread variables 
   stateTouch=THREAD_STATUS_UNTOUCHED;
   outputTouch.open((pathG+"/Touching.txt").c_str());
   
   return true;
}



void TouchEstimationThread::threadRelease()
{
  port_skin_contacts->close();
  outputTouch.close(); 
}





void TouchEstimationThread::run()
{

   Vector q(armTorsoDyn->getN()); q.zero();              // Encoder angles 
   Vector dq(q); dq.zero(); Vector ddq(q); ddq.zero();   // Encoder speed and accel are zeroed
   // ---  Configure arms with zero speed and accel
   armTorsoDyn->setDAng(dq);                             
   armTorsoDyn->setD2Ang(ddq);
   armTorsoDynR->setDAng(dq);                             
   armTorsoDynR->setD2Ang(ddq);

  // ---  Configure iCub left arm  with encodes from arms and torso
  Vector qTorso(3);
  Vector qArm(16);
  qTorso.zero();
  qArm.zero();
  Vector qArmTorso(10);
  encCtrlLeft->getEncoders(qArm.data());
  encCtrlTorso->getEncoders(qTorso.data());
  qArmTorso=cat(qTorso,qArm.subVector(0,6));
  armTorsoDyn->setAng(CTRL_DEG2RAD * qArmTorso); 

  // ---  Configure iCub right arm  with encodes from arms and torso
  encCtrlRight->getEncoders(qArm.data());
  qArmTorso=cat(qTorso,qArm.subVector(0,6));
  armTorsoDynR->setAng(CTRL_DEG2RAD * qArmTorso); 
 
  double currentTime=Time::now();
  skinContactList *scl = port_skin_contacts->read(false); // <------  Get skin contact list

  if(scl)   // there is contact with Skin
  {
    if (stateTouch==THREAD_STATUS_UNTOUCHED)
    {
            //   cout<<"|| I am untouched !!!!  ..."<<endl; 
           if(!scl->empty()) // There is contact
           { 
            
               stateTouch=THREAD_STATUS_FEELING;
               startTime = Time::now();
               // cout<<"|| iCub - Someone is touching me !!!!  ..."<<endl;
           }
        } // if untouched

    if (stateTouch==THREAD_STATUS_FEELING)
        {
           if(!scl->empty()) // There is contact
           {
               // Get information of touching per side
               //cout<<scl->toString().c_str()<<endl;
               map<BodyPart, skinContactList> contactsPerBp = scl->splitPerBodyPart();
               getTactileInformation(contactsPerBp[RIGHT_ARM],nContact,meanForce,meanSize,relPosition,armTorsoDynR,absPositionR,counterTouchR,false);
               getTactileInformation(contactsPerBp[LEFT_ARM],nContactL,meanForceL,meanSizeL,relPositionL,armTorsoDyn,absPositionL,counterTouchL,false);
               getTactileInformation(contactsPerBp[TORSO],nContactT,meanForceT,meanSizeT,relPositionT,armTorsoDyn,absPositionT,counterTouchT,true); 
               
           } else 
           {
               //cout<<"|| it is not ouched anymore ..."<<endl;  
               stateTouch=THREAD_STATUS_TOUCHED;
               totalTime=currentTime-startTime;
           }
        } // if feeling

    if (stateTouch==THREAD_STATUS_TOUCHED)
        {
           stateTouch=THREAD_STATUS_UNTOUCHED;
           // Make gesture classification
           int typeRight=decideTouch(nContact,meanForce,meanSize,relPosition);     
           int typeleft=decideTouch(nContactL,meanForceL,meanSizeL,relPositionL);
           int typeTorso=decideTouch(nContactT,meanForceT,meanSizeT,relPositionT);
           
           // Select the touch type and position based on the priority 
           typeTouch=typeleft;
           partTouch="left_arm";
           liveTouchPosition.zero();
           liveTouchSize=findMax(meanSizeL)-findMin(meanSizeL);
           if   (counterTouchL) {
            liveTouchPosition=absPositionL/counterTouchL;
           }
           
          cout<<"typeR:"<<typeRight<<endl;
          cout<<"typeL:"<<typeleft<<endl;
          cout<<"typeT:"<<typeTorso<<endl; 
          cout<<"LivePosition: "<<liveTouchPosition.toString()<<endl; 
         
           if (typeRight>typeTouch && typeRight>=0) {
              typeTouch=typeRight;
              partTouch="right_arm";
              liveTouchSize=findMax(meanSize)-findMin(meanSize);
              if (counterTouchR) {
              liveTouchPosition=absPositionR/counterTouchR;
              }
              
           }

           if (typeTorso>typeTouch && typeTorso>=0) {
              typeTouch=typeTorso;
              partTouch="torso";
              liveTouchSize=findMax(meanSizeT)-findMin(meanSizeT);
              if (counterTouchT) {
              liveTouchPosition=absPositionT/counterTouchT;
              }
           }

           // Clean temporal gesture variables
           nContact.clear();
           meanForce.clear();
           meanSize.clear();
           relPosition.clear();

           nContactL.clear();
           meanForceL.clear();
           meanSizeL.clear();
           relPositionL.clear();

           nContactT.clear();
           meanForceT.clear();
           meanSizeT.clear();
           relPositionT.clear();

       counterTouchR=0;
       counterTouchL=0;
           counterTouchT=0;

       absPositionR.resize(3);
       absPositionR.zero();

       absPositionL.resize(3);
       absPositionL.zero();
               
           absPositionT.resize(3);
       absPositionT.zero();
        }
   } // if scl
  
}  // function run


int TouchEstimationThread::decideTouch(Vector nContact1, Vector meanForce1, Vector meanSize1, Vector relPosition1)
{
    Vector vecInstance;
    vecInstance.clear();
   
    int typeT=-1; 
    if (findMax(nContact1)>0) {

    // Decide type of touching
        double sumPos=0;
        for (unsigned int i=0;i<nContact1.size();i++)
                sumPos=sumPos+relPosition1[i];
              //   cout<<"|| Robot has been just touched ..."<<endl;  
        sumPos=sumPos/nContact1.size();
        double meanDisp=0;
        for (unsigned int i=0;i<nContact1.size();i++)
            meanDisp=meanDisp+((sumPos-relPosition1[i])*(sumPos-relPosition1[i]));
              
       /*   cout<<"|| Robot has been just touched ..."<<endl;
        cout<<"|| min Contacts : "<< findMin(nContact1) <<endl;
        cout<<"|| max Contacts : "<< findMax(nContact1) <<endl;
        cout<<"|| min Force    : "<< findMin(meanForce1) <<endl;
        cout<<"|| max Force    : "<< findMax(meanForce1) <<endl;
        cout<<"|| min Size     : "<< findMin(meanSize1) <<endl;
        cout<<"|| max Size     : "<< findMax(meanSize1) <<endl;
        cout<<"|| position change : "<< findMax(relPosition1) - findMin(relPosition1) <<endl;
        cout<<"|| time touched :"<<totalTime<<endl;
        cout<<"|| Mean displ :"<<meanDisp<<endl;
         */     
        outputTouch << findMax(nContact1) << "\t"<< findMax(meanForce1) << "\t"<< findMax(meanSize1) << "\t"<< meanDisp<< endl; //<------Save to file this gesture
               // scale feature vector before classification
        vecInstance.push_back(scaler[0].transform(findMax(nContact1)) );
        vecInstance.push_back(scaler[1].transform(findMax(meanForce1)) );
        vecInstance.push_back(scaler[2].transform(findMax(meanSize1)) );
        vecInstance.push_back(scaler[3].transform(meanDisp));
               
        // Perform the ensamble of binary classification 
        double maxRes=-2.0;
        
        for (int iGesture=0;iGesture<nGestures;iGesture++){  
            Vector dataRes=classifiers[iGesture].predict (vecInstance).getPrediction();
                if (dataRes[0]>maxRes) {
                    maxRes=dataRes[0];
                    typeT=iGesture;
                }
         }
      }
   return typeT;
}


void TouchEstimationThread::getTactileInformation(skinContactList contactsPerBp,Vector& ncontac1, Vector& mforce1, Vector& msize1, Vector& rposicion1, iCubArmDyn * armTorsoDynX, Vector  & absPosition, int & counterTouch, bool flagTorso)
{

  // Get number of contacts
  int touchNumberL=contactsPerBp.size();
  int realTouchNumber=0;
  double contactsForce=0;
  double contactsSize=0;
  double contactGC=0; 
 
  Vector forceContactVec;  
  Vector sizeContactVec;
  // -----------  set angles to upper icub body 
  // Get mean information
  // cout<<"|| nTouch : "<< touchNumberL <<endl;
  if (touchNumberL>0) {   
      forceContactVec.clear();
      sizeContactVec.clear();
      for (int iLt=0;iLt<touchNumberL;iLt++){
         int nLinkL=contactsPerBp[iLt].getLinkNumber();
      //cout<<"|| Link : "<< nLinkL <<endl;
         if (nLinkL<=6) {
                // Force
                Vector forceVec=contactsPerBp[iLt].getForce();  // Get 3D vector of force in contact
                double force=norm(forceVec); // compute force of the contact
               // cout<<"|| Force : "<< force <<endl;
                if (force > THRESHOLDFORCECONTACT)
               {
                       //cout<<"|| force : "<< force <<endl;
                    // Number of contact
                        realTouchNumber++;
                    forceContactVec.push_back(force); // put the force in a Vector

                    // Size of contact
                    std::vector< unsigned int > taxesVec=contactsPerBp[iLt].getTaxelList();  // Get  vector of taxels in contact                
                    int nTaxel=taxesVec.size();
                        sizeContactVec.push_back(nTaxel);

            // Incremental Position of the contacts
                        Vector geoCenterVec=contactsPerBp[iLt].getGeoCenter();
                        //cout<<"Position of the contact:"<< geoCenterVec.toString().c_str()<<endl;
                        
                        //--------- Get cartesian position of touch from iCub arm object
                        Matrix matRT; 
                        if (flagTorso){
                            matRT=armTorsoDynX->getH(nLinkL);
                        } else {
                            matRT=armTorsoDynX->getH(nLinkL+3);
                        }
                        Vector augG=geoCenterVec;
                        augG.push_back(1.0);      
                        augG=matRT*augG;
                        Vector cartComtactPos=augG.subVector(0,2); //<- Cartesian position
                        absPosition=absPosition+cartComtactPos;  // summing all the touch position in current time
                        counterTouch++;                          // Increase the number of valid contacts
                        double gcContact=norm(geoCenterVec);
                        contactGC=contactGC+gcContact;
               } // If Force > than a threshold
         }  // If contact is not in hand
      }  // for each contact   

      if (forceContactVec.size()>0) {
    contactsForce=norm(forceContactVec);  // get mean Force of the contacts
        contactsSize=norm(sizeContactVec);  // get mean Force of the contacts 
     }
  } // if there is contact in this limb
  ncontac1.push_back(realTouchNumber);
  mforce1.push_back(contactsForce);
  msize1.push_back(contactsSize);
  rposicion1.push_back(contactGC);
}// funtion getInformation





void TouchEstimationThread::bodyPartTouched(string & partTouching, int & typeTouching, Vector & positionTouched, double & sizeTouched)
{

  partTouching=partTouch;
  typeTouching=typeTouch;
  positionTouched=liveTouchPosition;
  sizeTouched=liveTouchSize;

  liveTouchPosition.zero();
  liveTouchSize=0;
  typeTouch=-1;
}



