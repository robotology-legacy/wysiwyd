// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_slidingController_IDLServer
#define YARP_THRIFT_GENERATOR_slidingController_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class slidingController_IDLServer;


/**
 * slidingController_IDLServer
 * IDL Interface to \ref slidingController services.
 */
class slidingController_IDLServer : public yarp::os::Wire {
public:
  slidingController_IDLServer() { yarp().setOwner(*this); }
/**
 * Yield an asynchronous stop.
 * @return true/false on success/failure.
 */
  virtual bool stop();
/**
 * Yield a fingers recalibration.
 * @return true/false on success/failure.
 */
  virtual bool calibrate();
/**
 * Enable/disable impedance mode.
 * @param sw is "on"|"off".
 * @return true/false on success/failure.
 */
  virtual bool impedance(const std::string& sw);
/**
 * Start off table exploration.
 * @return true/false on success/failure.
 */
  virtual bool explore();
/**
 * Execute an hand posture.
 * @param key the tag of the hand posture.
 * @param wait if true wait until posture is reached.
 * @return true/false on success/failure.
 */
  virtual bool hand(const std::string& key, const bool wait = 0);
/**
 * Quit the module.
 * @return true/false on success/failure.
 */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

