# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# cvz.thrift

/**
* cvz_IDL
*
* IDL Interface to \ref cvz services.
*/
service cvz_IDL
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