# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# interpersonalDistanceRegulator.thrift

/**
* interpersonalDistanceRegulator_IDL
*
* IDL Interface to \ref interpersonalDistanceRegulator services.
*/
service interpersonalDistanceRegulator_IDL
{
   /**
   * Pause the automatic distance regulation.
   * @return true/false on success/failure.
   */
   bool pause();

   /**
   * Resume the automatic distance regulation.
   * @return true/false on success/failure.
   */
   bool resume();


   /**
   * Quit the module.
   * @return true/false on success/failure.
   */
   bool quit();  
}

