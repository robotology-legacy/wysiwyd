# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# cvzMmcm.thrift

/**
* cvzMmcm_IDL
*
* IDL Interface to \ref cvz - mmcm services.
*/
service cvzMmcm_IDL
{
   /**
   * Set the learning rate.
   */
   void setLearningRate(1:double l);
   
   /**
   * Get the current learning rate.
   */
   double getLearningRate();

   /**
   * Set the neighborhood radius (sigma).
   */
   void setSigma(1:double s);
  
   /**
   * Get the current neighborhood radius (sigma).
   */
   double getSigma();

   /**
   * Get the activity of a neuron.
   * @param x the horizontal coordinate of the neuron.
   * @param y the vertical coordinate of the neuron.
   * @param z the layer to which the neuron belongs.
   * @return Current activity of the neuron.
   */
   double getActivity(1:i32 x, 2:i32 y, 3:i32 z);
}

