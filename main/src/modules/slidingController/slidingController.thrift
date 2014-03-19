# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# slidingController.thrift

/**
* slidingController_IDLServer
*
* IDL Interface to \ref slidingController services.
*/
service slidingController_IDLServer
{
   /**
   * Yield an immediate stop of any ongoing movements
   * except exploration.
   * @return true/false on success/failure.
   */
   bool stop();

   /**
   * Yield a fingers recalibration.
   * @return true/false on success/failure.
   */
   bool calibrate();

   /**
   * Enable/disable impedance mode.
   * @param sw is "on"|"off".
   * @return true/false on success/failure.
   */
   bool impedance(1:string sw);

   /**
   * Start off table exploration.
   * @return true/false on success/failure.
   */
   bool explore();

   /**
   * Execute an hand posture.
   * @param key the tag of the hand posture.
   * @param wait if true wait until posture is reached.
   * @return true/false on success/failure.
   */
   bool hand(1:string key, 2:bool wait=false);

   /**
   * Quit the module.
   * @return true/false on success/failure.
   */
   bool quit();  
}

