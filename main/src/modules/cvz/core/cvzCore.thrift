# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# cvzCore.thrift

/**
* cvzCore_IDL
*
* IDL Interface to \ref cvzCore services.
*/
service cvzCore_IDL
{
   /**
   * Start the computation of predictions trying to cope with the period.
   */
   void start();

   /**
   * Pause the computation of predictions
   */
   void pause();

   /**
   * Quit the module.
   * @return true/false on success/failure.
   */
   bool quit();  
}

