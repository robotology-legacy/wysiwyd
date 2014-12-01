// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_slidingController_IDL
#define YARP_THRIFT_GENERATOR_slidingController_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class slidingController_IDL;


/**
 * slidingController_IDL
 * IDL Interface to \ref slidingController services.
 */
class slidingController_IDL : public yarp::os::Wire {
public:
  slidingController_IDL();
  /**
   * Yield an immediate stop of any ongoing movements
   * except exploration.
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
   * Move the arm towards the specified location.
   * @param x the x target coordinate.
   * @param y the y target coordinate.
   * @param z the z target coordinate.
   * @param interpolate if true generate waypoints
   *        to smooth out the trajectory.
   * @return true/false on success/failure.
   */
  virtual bool goTo(const double x, const double y, const double z, const bool interpolate = 1);
  /**
   * Quit the module.
   * @return true/false on success/failure.
   */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

