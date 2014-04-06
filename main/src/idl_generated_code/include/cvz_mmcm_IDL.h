// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_cvz_mmcm_IDL
#define YARP_THRIFT_GENERATOR_cvz_mmcm_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class cvz_mmcm_IDL;


/**
 * cvz_mmcm_IDL
 * IDL Interface to \ref cvz - mmcm services.
 */
class cvz_mmcm_IDL : public yarp::os::Wire {
public:
  cvz_mmcm_IDL() { yarp().setOwner(*this); }
/**
 * Set the learning rate.
 */
  virtual void setLearningRate(const double l);
/**
 * Get the current learning rate.
 */
  virtual double getLearningRate();
/**
 * Set the neighborhood radius (sigma).
 */
  virtual void setSigma(const double s);
/**
 * Get the current neighborhood radius (sigma).
 */
  virtual double getSigma();
/**
 * Get the activity of a neuron.
 * @param x the horizontal coordinate of the neuron.
 * @param y the vertical coordinate of the neuron.
 * @param z the layer to which the neuron belongs.
 * @return Current activity of the neuron.
 */
  virtual double getActivity(const int32_t x, const int32_t y, const int32_t z);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

