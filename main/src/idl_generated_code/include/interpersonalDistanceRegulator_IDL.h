// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_interpersonalDistanceRegulator_IDL
#define YARP_THRIFT_GENERATOR_interpersonalDistanceRegulator_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class interpersonalDistanceRegulator_IDL;


/**
 * interpersonalDistanceRegulator_IDL
 * IDL Interface to \ref interpersonalDistanceRegulator services.
 */
class interpersonalDistanceRegulator_IDL : public yarp::os::Wire {
public:
  interpersonalDistanceRegulator_IDL() { yarp().setOwner(*this); }
  /**
   * Pause the automatic distance regulation.
   * @return true/false on success/failure.
   */
  virtual bool pause();
  /**
   * Resume the automatic distance regulation.
   * @return true/false on success/failure.
   */
  virtual bool resume();
  /**
   * Quit the module.
   * @return true/false on success/failure.
   */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

