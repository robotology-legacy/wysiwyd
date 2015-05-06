// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_perspectiveTaking_IDL
#define YARP_THRIFT_GENERATOR_perspectiveTaking_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class perspectiveTaking_IDL;


/**
 * perspectiveTaking_IDL
 * IDL Interface to \ref perspectiveTaking service.
 */
class perspectiveTaking_IDL : public yarp::os::Wire {
public:
  perspectiveTaking_IDL();
  /**
   * Set the interval how often the
   * third person perspective
   * camera view should be updated
   * @param interval is the interval in ms
   * @return true/false on success/failure.
   */
  virtual bool setUpdateTimer(const int32_t interval);
  /**
   * Set the decimation (quality) of the most
   * recent camera frame. The lower, the better
   * @param decimation is a power of 2 (1, 2, 4, 8, ...)
   * @return true/false on success/failure.
   */
  virtual bool setDecimationOdometry(const int32_t decimation);
  /**
   * Set the decimation (quality) of for the
   * frames in the memory. The lower, the better
   * @param decimation is a power of 2 (1, 2, 4, 8, ...)
   * @return true/false on success/failure.
   */
  virtual bool setDecimationStatistics(const int32_t decimation);
  /**
   * Decide whether or not new frames should
   * be added to the memory or not
   * @param enable sets whether the current frame
   * is added to the memory or not
   * @return true/false on success/failure.
   */
  virtual bool processStats(const bool enable);
  virtual bool kinectStereoCalibrate();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

