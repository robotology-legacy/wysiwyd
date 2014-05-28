// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_opcManager_IDL
#define YARP_THRIFT_GENERATOR_opcManager_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class opcManager_IDL;


/**
 * opcManager_IDL
 * IDL Interface to \ref opcManager services.
 */
class opcManager_IDL : public yarp::os::Wire {
public:
  opcManager_IDL() { yarp().setOwner(*this); }
/**
 * Connect the module to an OPC.
 * @param nameOPC name of the OPC to connect (real or mental OPC).
 * @return Bottle with state of the connection.
 */
  virtual bool connect(const std::string& nameOPC);
/**
 * Update the beliefs of the agents present in an OPC.
 * @param nameOPC name of the OPC to update (real or mental OPC).
 * @return true/false on success/failure.
 */
  virtual bool updateBeliefs(const std::string& nameOPC);
/**
 * Synchronise the content of the mentalOPC on the content of the realOPC
 * @return Bottle with state of the synchronisation.
 */
  virtual bool synchronise();
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
  virtual bool executeActivity(const std::string& actionName, const std::string& Argument, const std::string& object, const std::string& agent);
/**
 * Return the differences between the 2 OPCs
 * @return Bottle of the differences
 */
  virtual bool diffOPC();
/**
 * Quit the module.
 * @return true/false on success/failure.
 */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

