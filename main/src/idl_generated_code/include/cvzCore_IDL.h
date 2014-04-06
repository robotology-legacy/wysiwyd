// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_cvzCore_IDL
#define YARP_THRIFT_GENERATOR_cvzCore_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class cvzCore_IDL;


/**
 * cvzCore_IDL
 * IDL Interface to \ref cvzCore services.
 */
class cvzCore_IDL : public yarp::os::Wire {
public:
  cvzCore_IDL() { yarp().setOwner(*this); }
/**
 * Start the computation of predictions trying to cope with the period.
 */
  virtual void start();
/**
 * Pause the computation of predictions
 */
  virtual void pause();
/**
 * Quit the module.
 * @return true/false on success/failure.
 */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

