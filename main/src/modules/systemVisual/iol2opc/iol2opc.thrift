# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Ugo Pattacini, Tobias Fischer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# iol2opc.thrift

/**
* iol2opc_IDL
*
* IDL Interface to \ref iol2opc service.
*/
service iol2opc_IDL
{
   /**
   * Add a new object to the object-recognition database
   * based on the selected blob. If the object is already
   * existing, its recognition is improved.
   * @param name is the object name
   * @return true/false on success/failure.
   */
   bool train_object(1:string name);

   /**
   * Remove object with a given name from the object-recognition
   * database.
   * @param name is the object name
   * @return true/false on success/failure.
   */
   bool remove_object(1:string name);

   /**
   * Remove all objects from the object-recognition
   * database.
   * @return true/false on success/failure.
   */
   bool remove_all();

   /**
   * Change the name of an object
   * @param old_name is the object which name is to be changed
   * @param new_name is the new object name
   * @return true/false on success/failure.
   */
   bool change_name(1:string old_name, 2:string new_name);

   /**
   * Pause module
   */
   void pause();

   /**
   * Resume module
   */
   void resume();
}
