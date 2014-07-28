# Copyright: (C) 2014 WYSIWYD Consortium
# Authors: Stéphane Lallée
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# cvzMmcm.thrift

/**
* cvzMmcm_IDL
*
* IDL Interface to \ref cvz - mmcm services (cvzCore).
*/
service cvzMmcm_IDL
{
   /**
   * Start the computation of predictions trying to cope with the period.
   */
   void start();

   /**
   * Pause the computation of predictions
   */
   void pause();

   /**
   * Quit the module.
   * @return true/false on success/failure.
   */
   bool quit(); 

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

   /**
   * Save the current weights of the map for latter reloading.
   * @param path The path to where you want to store the weights.
   * @return true/false in case of success/failure.
   */
   bool saveWeightsToFile(1:string path);

   /**
   * Load the weights of the map from a file. 
   * The file should have been saved from a map using the same config file (modaility names & size, map size, etc.).
   * @param path Path to the file containing the weights.
   * @return true/false in case of success/failure.
   */
   bool loadWeightsFromFile(1:string path);
}

