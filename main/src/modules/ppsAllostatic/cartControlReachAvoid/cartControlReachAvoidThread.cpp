/*
* Copyright (C) 2015 WYSIWYD
* Authors: Matej Hoffmann  and Ugo Pattacini
* email:   matej.hoffmann@iit.it, ugo.pattacini@itt.it
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/


#include "cartControlReachAvoidThread.h"


void cartControlReachAvoidThread::getTorsoOptions(Bottle &b, const char *type, const int i, Vector &sw, Matrix &lim)
{
        if (b.check(type))
        {
            Bottle &grp=b.findGroup(type);
            sw[i]=grp.get(1).asString()=="on"?1.0:0.0;

            if (grp.check("min","Getting minimum value"))
            {
                lim(i,0)=1.0;
                lim(i,1)=grp.find("min").asDouble();
            }

            if (grp.check("max","Getting maximum value"))
            {
                lim(i,2)=1.0;
                lim(i,3)=grp.find("max").asDouble();
            }
        }
}

void cartControlReachAvoidThread::getArmOptions(Bottle &b, Vector &reachOffs, Vector &orien, bool &impVelMode,
                       Vector &impStiff, Vector &impDamp)
    {
        if (b.check("reach_offset","Getting reaching offset"))
        {
            Bottle &grp=b.findGroup("reach_offset");
            int sz=grp.size()-1;
            int len=sz>3?3:sz;

            for (int i=0; i<len; i++)
                reachOffs[i]=grp.get(1+i).asDouble();
        }

        if (b.check("hand_orientation","Getting hand orientation"))
        {
            Bottle &grp=b.findGroup("hand_orientation");
            int sz=grp.size()-1;
            int len=sz>4?4:sz;

            for (int i=0; i<len; i++)
                orien[i]=grp.get(1+i).asDouble();
        }

        impVelMode=b.check("impedance_velocity_mode",Value("off"),"Getting arm impedance-velocity-mode").asString()=="on"?true:false;

        if (b.check("impedance_stiffness","Getting joints stiffness"))
        {
            Bottle &grp=b.findGroup("impedance_stiffness");
            size_t sz=grp.size()-1;
            size_t len=sz>impStiff.length()?impStiff.length():sz;

            for (size_t i=0; i<len; i++)
                impStiff[i]=grp.get(1+i).asDouble();
        }

        if (b.check("impedance_damping","Getting joints damping"))
        {
            Bottle &grp=b.findGroup("impedance_damping");
            size_t sz=grp.size()-1;
            size_t len=sz>impDamp.length()?impDamp.length():sz;

            for (size_t i=0; i<len; i++)
                impDamp[i]=grp.get(1+i).asDouble();
        }
    }

void cartControlReachAvoidThread::getHomeOptions(Bottle &b, Vector &poss, Vector &vels)
{
        if (b.check("poss","Getting home poss"))
        {
            Bottle &grp=b.findGroup("poss");
            int sz=grp.size()-1;
            int len=sz>7?7:sz;

            for (int i=0; i<len; i++)
                poss[i]=grp.get(1+i).asDouble();
        }

        if (b.check("vels","Getting home vels"))
        {
            Bottle &grp=b.findGroup("vels");
            int sz=grp.size()-1;
            int len=sz>7?7:sz;

            for (int i=0; i<len; i++)
                vels[i]=grp.get(1+i).asDouble();
        }
}

    
void cartControlReachAvoidThread::initCartesianCtrl(Vector &sw, Matrix &lim, const int sel)
{
        ICartesianControl *icart=cartArm;
        Vector dof;
        string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
            {
                drvCartLeftArm->view(icart);
                icart->storeContext(&startup_context_id_left);
                icart->restoreContext(0);
            }
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
            {
                drvCartRightArm->view(icart);
                icart->storeContext(&startup_context_id_right);
                icart->restoreContext(0);
            }
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Initializing %s controller ...\n",type.c_str());

        icart->setTrackingMode(false);
        icart->setTrajTime(trajTime);
        icart->setInTargetTol(reachTol);
        icart->getDOF(dof);

        for (size_t j=0; j<sw.length(); j++)
        {
            dof[j]=sw[j];

            if (sw[j] && (lim(j,0) || lim(j,2)))
            {
                double min, max;
                icart->getLimits(j,&min,&max);

                if (lim(j,0))
                    min=lim(j,1);

                if (lim(j,2))
                    max=lim(j,3);

                icart->setLimits(j,min,max);
                fprintf(stdout,"jnt #%d in [%g, %g] deg\n",(int)j,min,max);
            }
        }

        icart->setDOF(dof,dof);

        fprintf(stdout,"DOF's=( ");
        for (size_t i=0; i<dof.length(); i++)
            fprintf(stdout,"%s ",dof[i]>0.0?"on":"off");
        fprintf(stdout,")\n");
}
    
void cartControlReachAvoidThread::getSensorData()
{
         if (encTorso->getEncoders(torso.data())){
            R=(rotx(torso[1])) * (roty(-torso[2])) * (rotz(-torso[0]));
         }
}
    
bool cartControlReachAvoidThread::checkPosFromPortInput(Vector &target_pos)
{
        if (Bottle *targetPosNew=inportTargetCoordinates.read(false))
        {
            target_pos[0]=targetPosNew->get(0).asDouble();
            target_pos[1]=targetPosNew->get(1).asDouble();
            target_pos[2]=targetPosNew->get(2).asDouble();
            return true;
           
        }
        else{
            return false;   
        }
}
    

    
void cartControlReachAvoidThread::selectArm()
{
        if (useLeftArm && useRightArm)
        {
           if(targetPos[1]>0.0){
               armSel=RIGHTARM; 
               printf("Target in right space - selecting right arm.\n");
               drvRightArm->view(encArm);
               drvRightArm->view(posArm);
               drvRightArm->view(velArm);
               drvCartRightArm->view(cartArm);
               armReachOffs=&rightArmReachOffs;
               armHandOrien=&rightArmHandOrien;
           }
           else{
               armSel=LEFTARM; 
               printf("Target in left space - selecting left arm.\n");
               drvLeftArm->view(encArm);
               drvLeftArm->view(posArm);
               drvLeftArm->view(velArm);
               drvCartLeftArm->view(cartArm);
               armReachOffs=&leftArmReachOffs;
               armHandOrien=&leftArmHandOrien;
           }
         }
}
    
void cartControlReachAvoidThread::doReach()
    {
        if (useLeftArm || useRightArm)
        {
            if (state==STATE_REACH)
            {
                int cart_dof_nr = 0.0;
                //torso has global vars torsoAxes (~torso_joints_nr) and torso (~encodersTorso)
                
                Vector curDof;
                Vector xdhat, odhat, qdhat;
                Vector torsoAndArmJointDeltas; //!torso already in motor control order 
                // iKin / cartControl pitch, roll, yaw;   motorInterface - yaw, roll, pitch
                Vector qdotReach, qdotReachAndAvoid; //3 torso joint velocities (but in motor interface order! yaw, roll, pitch) + 7 arm joints
                Vector qdotArm, qdotTorso; //joint velocities in motor interface format
              
                /**** init ***************/
              
                qdotArm.resize(armAxes);
                qdotTorso.resize(torsoAxes,0.0);
                
                cartArm->getDOF(curDof);
                yDebug("cartArm DOFs [%s]\n",curDof.toString().c_str());  // [0 0 0 1 1 1 1 1 1 1] will be printed out if torso is off
                cart_dof_nr = curDof.length();
                torsoAndArmJointDeltas.resize(cart_dof_nr,0.0);
                qdotReach.resize(cart_dof_nr);
                qdotReachAndAvoid.resize(cart_dof_nr,0.0); 
                     
                
                /******* solve for optimal reaching configuration  **********************************/  
                //optionally set up weights - setRestWeights - optionally to give more value current joint pos (10 values, torso + arm))
                Vector x=R.transposed()*(targetPos+*armReachOffs);
                limitRange(x);
                x=R*x;
                yDebug("doReach(): reach target x %s.\n",x.toString().c_str());
                cartArm->askForPosition(x,xdhat,odhat,qdhat); //7+3, including torso, probably even if torso is off
                //  qdhat now contains target joint pos
                yDebug("xdhat from cart solver %s.\n",xdhat.toString().c_str());
                yDebug("qdhat from cart solver %s.\n",qdhat.toString().c_str());
           
                
                //*********** now the controller...  ***************************/
                //use minJerkVelCtrl - instance of minJerkVelCtrlForIdealPlant
                                      
                bool ret=encArm->getEncoders(arm.data()); 
                bool ret2 = encTorso->getEncoders(torso.data());
                     
                if ((!ret) && (!ret2))
                {
                    yError("Error reading encoders, check connectivity with the robot\n");
                }
                else
                {  /* use encoders */
                    yDebug("There are %d arm encoder values: %s\n",armAxes,arm.toString().c_str());
                    yDebug("There are %d torso encoder values: %s\n",torsoAxes,torso.toString().c_str());
                   
                    torsoAndArmJointDeltas[0] = qdhat[2]-torso[0];
                    torsoAndArmJointDeltas[1] = qdhat[1]-torso[1];
                    torsoAndArmJointDeltas[2] = qdhat[0]-torso[2];
                    torsoAndArmJointDeltas[3]=qdhat[3]-arm[0]; // shoulder pitch
                    yDebug("Shoulder pitch difference torsoAndArmJointDeltas[3]=qdhat[3]-encodersArm[0], %f = %f - % f\n", torsoAndArmJointDeltas[3],qdhat[3],arm[0]);
                    torsoAndArmJointDeltas[4]=qdhat[4]-arm[1]; // shoulder roll
                    torsoAndArmJointDeltas[5]=qdhat[5]-arm[2]; // shoulder yaw
                    torsoAndArmJointDeltas[6]=qdhat[6]-arm[3]; // elbow
                    torsoAndArmJointDeltas[7]=qdhat[7]-arm[4]; // wrist pronosupination
                    torsoAndArmJointDeltas[8]=qdhat[8]-arm[5]; // wrist pitch
                    torsoAndArmJointDeltas[9]=qdhat[9]-arm[6]; // wrist yaw
                    
                    yDebug("torsoAndArmJointDeltas: %s\n",torsoAndArmJointDeltas.toString().c_str());
                    qdotReach = minJerkVelCtrl->computeCmd(trajTime,torsoAndArmJointDeltas); //2 seconds to reach       
                    yDebug("qdotReach: %s\n",qdotReach.toString().c_str());
                    
                }
          
                //so far, qdot based only on reaching; now combine magically with avoidance  
         
                           
                //TODO
                /*
                for each avoidance vector
                    skinPart gives FoR, which is link after the point - move to first joint/link before the point
                    express point in last proximal joint; create extra link - rototranslation    
                
                instantiate new chain
                iKin:: iCubArm 
                
                blockLink() to block all the more distal joints after the joint
                (release all others)
                setHN applies the rototranslation to 
                then
                GeoJacobian to get J
                then from x_dot = J * q_dot
                q_dot_avoidance = pimv(J) * x_dot
                (maybe also here the weights)
                
                (optionally, if you want avoidance to act more locally, you can 0 some joints from q_dot_avoidance)
                
                combine q_dot_reach with q_dot_avoidance; such as weighted sum using gains from allostasis
                */
                qdotReachAndAvoid = qdotReach;
                
                
                /**** do the movement - send velocities ************************************/
                //need to pass from the cartArm format (3 for torso, 7 for arm) to torso (3 joints) + arm motor control (16 joints)
                
                qdotTorso[0]=qdotReachAndAvoid[0];
                qdotTorso[1]=qdotReachAndAvoid[1];
                qdotTorso[2]=qdotReachAndAvoid[2];
                
                qdotArm[0]=qdotReachAndAvoid[3];
                qdotArm[1]=qdotReachAndAvoid[4];
                qdotArm[2]=qdotReachAndAvoid[5];
                qdotArm[3]=qdotReachAndAvoid[6];
                qdotArm[4]=qdotReachAndAvoid[7];
                qdotArm[5]=qdotReachAndAvoid[8];
                qdotArm[6]=qdotReachAndAvoid[9];     
                for(int j=7;j<armAxes;j++){
                  qdotArm[j]=0.0; //no movement for the rest of the joints   
                }
                              
                yDebug("velocityMove(qdotArm.data()), qdotArm: %s\n", qdotArm.toString().c_str());
                velArm->velocityMove(qdotArm.data());
                yDebug("velocityMove(qdotTorso.data()), qdotTorso: %s\n", qdotTorso.toString().c_str());
                velTorso->velocityMove(qdotTorso.data());
              
            }
        }
    }
         
void cartControlReachAvoidThread::doIdle()
    {
    }
    
void cartControlReachAvoidThread::steerTorsoToHome()
    {
        Vector homeTorso(3);
        homeTorso.zero();

        Vector velTorso(3);
        velTorso=10.0;

        fprintf(stdout,"*** Homing torso\n");

        posTorso->setRefSpeeds(velTorso.data());
        posTorso->positionMove(homeTorso.data());
    }
    
void cartControlReachAvoidThread::checkTorsoHome(const double timeout)
    {
        fprintf(stdout,"*** Checking torso home position... ");

        bool done=false;
        double t0=Time::now();
        while (!done && (Time::now()-t0<timeout))
        {
            posTorso->checkMotionDone(&done);
            Time::delay(0.1);
        }

        fprintf(stdout,"*** done\n");
    }

    
void cartControlReachAvoidThread::stopArmJoints(const int sel)
    {
        IEncoders        *ienc=encArm;
        IPositionControl *ipos=posArm;
        string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
            {
                drvLeftArm->view(ienc);
                drvLeftArm->view(ipos);
            }
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
            {
                drvRightArm->view(ienc);
                drvRightArm->view(ipos);
            }
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Stopping %s joints\n",type.c_str());
        for (size_t j=0; j<homeVels.length(); j++)
        {
            double fb;

            ienc->getEncoder(j,&fb);
            ipos->positionMove(j,fb);
        }
    }
     
void cartControlReachAvoidThread::steerArmToHome(const int sel)
    {
        IPositionControl *ipos=posArm;
        string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
                drvLeftArm->view(ipos);
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
                drvRightArm->view(ipos);
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Homing %s\n",type.c_str());
        for (size_t j=0; j<homeVels.length(); j++)
        {
            ipos->setRefSpeed(j,homeVels[j]);
            ipos->positionMove(j,homePoss[j]);
        }

        //openHand(sel);
    }
    
void cartControlReachAvoidThread::checkArmHome(const int sel, const double timeout)
    {
        IPositionControl *ipos=posArm;
        string type;

        if (sel==LEFTARM)
        {
            if (useLeftArm)
                drvLeftArm->view(ipos);
            else
                return;

            type="left_arm";
        }
        else if (sel==RIGHTARM)
        {
            if (useRightArm)
                drvRightArm->view(ipos);
            else
                return;

            type="right_arm";
        }
        else if (armSel!=NOARM)
            type=armSel==LEFTARM?"left_arm":"right_arm";
        else
            return;

        fprintf(stdout,"*** Checking %s home position... ",type.c_str());

        bool done=false;
        double t0=Time::now();
        while (!done && (Time::now()-t0<timeout))
        {
            ipos->checkMotionDone(&done);
            Time::delay(0.1);
        }

        fprintf(stdout,"*** done\n");
    }
    
void cartControlReachAvoidThread::stopControl()
    {
        if (useLeftArm || useRightArm)
        {
            fprintf(stdout,"stopping control\n");
            cartArm->stopControl();
            Time::delay(0.1);
        }        
    }

void cartControlReachAvoidThread::limitRange(Vector &x)
    {               
        x[0]=x[0]>-0.1 ? -0.1 : x[0];       
    }
    
Matrix & cartControlReachAvoidThread::rotx(const double theta)
{
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Rx(1,1)=Rx(2,2)=c;
        Rx(1,2)=-s;
        Rx(2,1)=s;

        return Rx;
}

Matrix & cartControlReachAvoidThread::roty(const double theta)
{
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Ry(0,0)=Ry(2,2)=c;
        Ry(0,2)=s;
        Ry(2,0)=-s;

        return Ry;
}

Matrix & cartControlReachAvoidThread::rotz(const double theta)
{
        double t=CTRL_DEG2RAD*theta;
        double c=cos(t);
        double s=sin(t);

        Rz(0,0)=Rz(1,1)=c;
        Rz(0,1)=-s;
        Rz(1,0)=s;

        return Rz;
}
    
bool cartControlReachAvoidThread::reachableTarget(const Vector target_pos){
        if( (target_pos[0] < reachableSpace.minX) || (target_pos[0] > reachableSpace.maxX) ||
            (target_pos[1] < reachableSpace.minY) || (target_pos[1] > reachableSpace.maxY) ||
            (target_pos[2] < reachableSpace.minZ) || (target_pos[2] > reachableSpace.maxZ)){
            printf("Warning: target %f %f %f is outside of reachable area.\n",target_pos[0],target_pos[1],target_pos[2]);
            return false;          
        }
        else{
                return true;
        }
}

bool cartControlReachAvoidThread::targetCloseEnough(){
    
    Vector x,o;
    double distanceFromTarget = 0.0;
    //get current end-eff position 
    //cartArm is already pointing to the right arm
    cartArm->getPose(x,o);
    distanceFromTarget = sqrt( pow(targetPos[0]-x[0],2) + pow(targetPos[1]-x[1],2) + pow(targetPos[2]-x[2],2)); 
    yDebug("targetCloseEnough(): distance: %f, requested target: %s, current end-eff pos: %s\n",distanceFromTarget,targetPos.toString().c_str(),x.toString().c_str());
    if(distanceFromTarget<reachTol){
        yDebug("targetCloseEnough(): returning true: distanceFromTarget: %f < reachTol: %f\n",distanceFromTarget,reachTol);
        return true;
    }
    else{
        return false;
    }
    
}
    
cartControlReachAvoidThread::cartControlReachAvoidThread(int _rate, const string &_name, const string &_robot, int _v,ResourceFinder &_rf): 
RateThread(_rate),name(_name), robot(_robot), verbosity(_v),rf(_rf)
{
   
  ;
  
}

bool cartControlReachAvoidThread::threadInit()
{
       
        ts.update();
    
        inportTargetCoordinates.open(("/"+name+"/reachingTarget:i").c_str());
        inportAvoidanceVectors.open(("/"+name+"/avoidanceVectors:i").c_str());
        inportReachingGain.open(("/"+name+"/reachingGain:i").c_str());
        inportAvoidanceGain.open(("/"+name+"/avoidanceGain:i").c_str());
    
        //getting values from config file
        // general part
        Bottle &bGeneral=rf.findGroup("general");
        useLeftArm=bGeneral.check("left_arm",Value("on"),"Getting left arm use flag").asString()=="on"?true:false;
        useRightArm=bGeneral.check("right_arm",Value("on"),"Getting right arm use flag").asString()=="on"?true:false;
        trajTime=bGeneral.check("traj_time",Value(3.0),"Getting trajectory time").asDouble();
        reachTol=bGeneral.check("reach_tol",Value(0.05),"Getting reaching tolerance").asDouble();
        reachTmo=bGeneral.check("reach_tmo",Value(10.0),"Getting reach timeout").asDouble();
              
        // torso part
        Bottle &bTorso=rf.findGroup("torso");
        bTorso.setMonitor(rf.getMonitor());

        Vector torsoSwitch(3);   torsoSwitch.zero();
        Matrix torsoLimits(3,4); torsoLimits.zero();

        getTorsoOptions(bTorso,"pitch",0,torsoSwitch,torsoLimits);
        getTorsoOptions(bTorso,"roll",1,torsoSwitch,torsoLimits);
        getTorsoOptions(bTorso,"yaw",2,torsoSwitch,torsoLimits);    
               
        // arm parts
        Bottle &bLeftArm=rf.findGroup("left_arm");
        Bottle &bRightArm=rf.findGroup("right_arm");
        bLeftArm.setMonitor(rf.getMonitor());
        bRightArm.setMonitor(rf.getMonitor());
        
        // arm parts
        leftArmReachOffs.resize(3,0.0);
        leftArmHandOrien.resize(4,0.0);
        leftArmJointsStiffness.resize(5,0.0);
        leftArmJointsDamping.resize(5,0.0);
        rightArmReachOffs.resize(3,0.0);
        rightArmHandOrien.resize(4,0.0);
        rightArmJointsStiffness.resize(5,0.0);
        rightArmJointsDamping.resize(5,0.0);

        getArmOptions(bLeftArm,leftArmReachOffs,leftArmHandOrien,leftArmImpVelMode,
                      leftArmJointsStiffness,leftArmJointsDamping);
        getArmOptions(bRightArm,rightArmReachOffs,rightArmHandOrien,rightArmImpVelMode,
                      rightArmJointsStiffness,rightArmJointsDamping);
       
        // home part
        Bottle &bHome=rf.findGroup("home_arm");
        bHome.setMonitor(rf.getMonitor());
        homePoss.resize(7,0.0); homeVels.resize(7,0.0);
        getHomeOptions(bHome,homePoss,homeVels);
      
        targetPosFromPort.resize(3,0.0);
        targetPosFromRPC.resize(3,0.0);
        targetPos.resize(3,0.0);
       
        //TODO add to config file
        reachableSpace.minX=-0.4; reachableSpace.maxX=-0.2;
        reachableSpace.minY=-0.3; reachableSpace.maxY=0.3;
        reachableSpace.minZ=-0.1; reachableSpace.maxZ=0.1;
        
        string fwslash="/";

        // open remote_controlboard drivers
        Property optTorso("(device remote_controlboard)");
        Property optLeftArm("(device remote_controlboard)");
        Property optRightArm("(device remote_controlboard)");

        optTorso.put("remote",(fwslash+robot+"/torso").c_str());
        optTorso.put("local",(fwslash+name+"/torso").c_str());

        optLeftArm.put("remote",(fwslash+robot+"/left_arm").c_str());
        optLeftArm.put("local",(fwslash+name+"/left_arm").c_str());

        optRightArm.put("remote",(fwslash+robot+"/right_arm").c_str());
        optRightArm.put("local",(fwslash+name+"/right_arm").c_str());

        drvTorso=new PolyDriver;
        if (!drvTorso->open(optTorso))
        {
            threadRelease();
            return false;
        }

        if (useLeftArm)
        {
            drvLeftArm=new PolyDriver;
            if (!drvLeftArm->open(optLeftArm))
            {
                threadRelease();
                return false;
            }
        }

        if (useRightArm)
        {
            drvRightArm=new PolyDriver;
            if (!drvRightArm->open(optRightArm))
            {
                threadRelease();
                return false;
            }
        }

        // open cartesiancontrollerclient drivers
        Property optCartLeftArm("(device cartesiancontrollerclient)");
        Property optCartRightArm("(device cartesiancontrollerclient)");
       
        optCartLeftArm.put("remote",(fwslash+robot+"/cartesianController/left_arm").c_str());
        optCartLeftArm.put("local",(fwslash+name+"/left_arm/cartesian").c_str());
    
        optCartRightArm.put("remote",(fwslash+robot+"/cartesianController/right_arm").c_str());
        optCartRightArm.put("local",(fwslash+name+"/right_arm/cartesian").c_str());

      
        if (useLeftArm)
        {
            drvCartLeftArm=new PolyDriver;
            if (!drvCartLeftArm->open(optCartLeftArm))
            {
                threadRelease();
                return false;
            }
            if (leftArmImpVelMode)
            {
                IControlMode      *imode;
                IImpedanceControl *iimp;

                drvLeftArm->view(imode);
                drvLeftArm->view(iimp);

                int len=leftArmJointsStiffness.length()<leftArmJointsDamping.length()?
                        leftArmJointsStiffness.length():leftArmJointsDamping.length();

                for (int j=0; j<len; j++)
                {
                    imode->setImpedanceVelocityMode(j);
                    iimp->setImpedance(j,leftArmJointsStiffness[j],leftArmJointsDamping[j]);
                }
            }
        }

        if (useRightArm)
        {
            drvCartRightArm=new PolyDriver;
            if (!drvCartRightArm->open(optCartRightArm))
            {
                threadRelease();
                return false;
            }

            if (rightArmImpVelMode)
            {
                IControlMode      *imode;
                IImpedanceControl *iimp;

                drvRightArm->view(imode);
                drvRightArm->view(iimp);

                int len=rightArmJointsStiffness.length()<rightArmJointsDamping.length()?
                        rightArmJointsStiffness.length():rightArmJointsDamping.length();

                for (int j=0; j<len; j++)
                {
                    imode->setImpedanceVelocityMode(j);
                    iimp->setImpedance(j,rightArmJointsStiffness[j],rightArmJointsDamping[j]);
                }
            }
        }

        // open views
      
        drvTorso->view(encTorso);
        drvTorso->view(posTorso);
        drvTorso->view(velTorso);
              
        if (useLeftArm)
        {
            drvLeftArm->view(encArm);
            drvLeftArm->view(posArm);
            drvLeftArm->view(velArm);
            drvCartLeftArm->view(cartArm);
            armReachOffs=&leftArmReachOffs;
            armHandOrien=&leftArmHandOrien;
            armSel=LEFTARM;
        }
        else if (useRightArm)
        {
            drvRightArm->view(encArm);
            drvRightArm->view(posArm);
            drvRightArm->view(velArm);
            drvCartRightArm->view(cartArm);
            armReachOffs=&rightArmReachOffs;
            armHandOrien=&rightArmHandOrien;
            armSel=RIGHTARM;
        }
        else
        {
            encArm=NULL;
            posArm=NULL;
            velArm=NULL;
            cartArm=NULL;
            armReachOffs=NULL;
            armHandOrien=NULL;
            armSel=NOARM;
        }

        encArm->getAxes(&armAxes);
        arm.resize(armAxes,0.0);
        encTorso->getAxes(&torsoAxes);
        torso.resize(torsoAxes,0.0);

        /* we need to set reference accelerations for the velocityMove to be used later */
        /* profile, 50 degrees/sec^2 */
        int k;
        Vector tmp_acc_arm;
        tmp_acc_arm.resize(armAxes,0.0);
        for (k = 0; k < armAxes; k++) {
            tmp_acc_arm[k] = REF_ACC;
        }
        velArm->setRefAccelerations(tmp_acc_arm.data());
                 
        int l;
        Vector tmp_acc_torso;
        tmp_acc_torso.resize(torsoAxes,0.0);
        for (l = 0; l < torsoAxes; l++) {
            tmp_acc_torso[l] = REF_ACC;
        }
        velTorso->setRefAccelerations(tmp_acc_torso.data());
        
        targetPos.resize(3,0.0);
        R=Rx=Ry=Rz=eye(3,3);

        initCartesianCtrl(torsoSwitch,torsoLimits,LEFTARM);
        initCartesianCtrl(torsoSwitch,torsoLimits,RIGHTARM);

        // steer the robot to the initial configuration
        stopControl();
        steerTorsoToHome();
        steerArmToHome(LEFTARM);
        steerArmToHome(RIGHTARM);
     
        wentHome=false;
        state=STATE_IDLE;

        minJerkVelCtrl = new  minJerkVelCtrlForIdealPlant(threadPeriod,10); //3 torso + 7 arm
        
        return true;
      
  
}

void cartControlReachAvoidThread::cartControlReachAvoidThread::run()
{
     ts.update();
    
     bool newTarget = false;
       
     getSensorData();
        
     newTargetFromPort =  checkPosFromPortInput(targetPosFromPort);
     if(newTargetFromPort || newTargetFromRPC){ //target from RPC is set asynchronously
            newTarget = true;
            if (newTargetFromRPC){ //RPC has priority
                printf("run: setting new target from rpc: %f %f %f \n",targetPos[0],targetPos[1],targetPos[2]);
                newTargetFromRPC = false;
            }
            else{
                targetPos = targetPosFromPort;
                printf("run: setting new target from port: %f %f %f \n",targetPos[0],targetPos[1],targetPos[2]);
                newTargetFromPort = false;
            }
     }
        
    if (state==STATE_IDLE){
        yDebug("run(): STATE_IDLE\n");
        if (newTarget){
            if (reachableTarget(targetPos)){
                printf("--- Got new target => REACHING\n"); 
                state = STATE_REACH;
                reachTimer = Time::now();
            }
            else{
                printf("Target (%f %f %f) is outside of reachable space - x:<%f %f>, y:<%f %f>, z:<%f %f>\n",
                    targetPos[0],targetPos[1],targetPos[2],reachableSpace.minX,reachableSpace.maxX,reachableSpace.minY,
                    reachableSpace.maxY,reachableSpace.minZ,reachableSpace.maxZ);
                }
            }
            //else - remain idle
        }
    else if(state==STATE_REACH){
        yDebug("run(): STATE_REACH\n");
        selectArm();
        doReach();
        
    }
          
    /*else if(state == STATE_CHECKMOTIONDONE)
    {
        yDebug("run(): STATE_CHECKMOTIONDONE\n");
        bool done = false;
        done = targetCloseEnough();
        //cartArm->checkMotionDone(&done); //cannot be used here - we are controlling manually with velocity
            if (!done) 
            {
                if( ((Time::now()-reachTimer)>reachTmo)){
                    fprintf(stdout,"--- Reach timeout => go home\n");
                    state = STATE_GO_HOME;
                }
            }
            else{ //done
                fprintf(stdout,"--- Reach done. => go IDLE\n");
                state = STATE_IDLE;
            }
    }*/
     
    else if(state==STATE_GO_HOME){
        yDebug("run(): STATE_GO_HOME\n");
        stopControl();
        steerTorsoToHome();
        steerArmToHome(LEFTARM);
        steerArmToHome(RIGHTARM);
        yDebug("--- I'm home => go idle\n");
        state=STATE_IDLE;
    }
}
 

int cartControlReachAvoidThread::printMessage(const int l, const char *f, ...)
{
    if (verbosity>=l)
    {
        fprintf(stdout,"[%s] ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);
        return ret;
    }
    else
        return -1;
}

void cartControlReachAvoidThread::threadRelease()
{
     delete drvTorso;
     delete drvLeftArm;
     delete drvRightArm;
     delete drvCartLeftArm;
     delete drvCartRightArm;
     delete minJerkVelCtrl;
        
      
    printMessage(0,"Closing ports..\n");
    inportTargetCoordinates.interrupt();
    inportTargetCoordinates.close();
    printMessage(1,"inportTargetCoordinates successfully closed\n");
    inportAvoidanceVectors.interrupt();
    inportAvoidanceVectors .close();
    printMessage(1,"inportAvoidanceVectors successfully closed\n");
    inportReachingGain.interrupt();
    inportReachingGain .close();
    printMessage(1,"inportReachingGain successfully closed\n");
    inportAvoidanceGain.interrupt();
    inportAvoidanceGain.close();
    printMessage(1,"inportAvoidanceGain successfully closed\n");
    
}

bool cartControlReachAvoidThread::setTargetFromRPC(const Vector target_pos)
{
        for(int i=0;i<target_pos.size();i++)
        {
            targetPos[i]=target_pos[i];
        }
        //printf("reachingThread::setTargetFromRPC: Setting targetPos from RPC to %f %f %f.\n",targetPos[0],targetPos[1],targetPos[2]);
        newTargetFromRPC = true;
        return true;
}
    
bool cartControlReachAvoidThread::setHomeFromRPC()
{
    // steer the robot to the initial configuration
        state = STATE_GO_HOME;
        return true;
}