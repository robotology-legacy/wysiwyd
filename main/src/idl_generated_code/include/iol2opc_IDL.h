/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_iol2opc_IDL
#define YARP_THRIFT_GENERATOR_iol2opc_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class iol2opc_IDL;


/**
 * iol2opc_IDL
 * IDL Interface to \ref iol2opc service.
 */
class iol2opc_IDL : public yarp::os::Wire {
public:
  iol2opc_IDL();
  /**
   * Add a new object to the object-recognition database
   * based on the selected blob. If the object is already
   * existing, its recognition is improved.
   * @param name is the object name
   * @return true/false on success/failure.
   */
  virtual bool train_object(const std::string& name);
  /**
   * Remove object with a given name from the object-recognition
   * database.
   * @param name is the object name
   * @return true/false on success/failure.
   */
  virtual bool remove_object(const std::string& name);
  /**
   * Remove all objects from the object-recognition
   * database.
   * @return true/false on success/failure.
   */
  virtual bool remove_all();
  /**
   * Change the name of an object
   * @param old_name is the object which name is to be changed
   * @param new_name is the new object name
   * @return true/false on success/failure.
   */
  virtual bool change_name(const std::string& old_name, const std::string& new_name);
  /**
   * Enable/disable object persistence.
   * @param sw can be "on"|"off".
   * @return true/false on success/failure.
   */
  virtual bool set_object_persistence(const std::string& sw);
  /**
   * Return current status of object persistence.
   * @return "on"|"off".
   */
  virtual std::string get_object_persistence();
  /**
   * Pause module
   */
  virtual void pause();
  /**
   * Resume module
   */
  virtual void resume();
  virtual bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
