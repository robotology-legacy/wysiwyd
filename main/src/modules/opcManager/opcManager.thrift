# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Grégoire Pointeau
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# opcManager.thrift

/**
* opcManager_IDL
*
* IDL Interface to \ref opcManager services.
*/


service opcManager_IDL
{
   /**
   * Connect the module to an OPC.
   * @param nameOPC name of the OPC to connect (real or mental OPC).
   * @return Bottle with state of the connection.
   */
   bool connect(1:string nameOPC);

   /**
   * Update the beliefs of the agents present in an OPC.
   * @param nameOPC name of the OPC to update (real or mental OPC).
   * @return true/false on success/failure.
   */
   bool updateBeliefs(1:string nameOPC);
   
   /**
   * Synchronise the content of the mentalOPC on the content of the realOPC
   * @return Bottle with state of the synchronisation.
   */
   bool synchronise();
   
   /**
   * Simulate the execution of an activity in the mental OPC.
   * input format is the same as for abmReasoning:
   * Bottle ("executeActivity"       actionName      argument        object      agent)
   * @param name of the action to execute
   * @param argument of the action to execute
   * @param object of the action to execute
   * @param agent of the action to execute
   * @return Bottle with state of the execution.
   */
   bool executeActivity(1:string actionName, 2:string Argument, 3:string object, 4:string agent);
   
   /**
   * Return the differences between the 2 OPCs
   * @return Bottle of the differences
   */
   bool diffOPC();
   
   /**
   * Quit the module.
   * @return true/false on success/failure.
   */
   bool quit();  
}

