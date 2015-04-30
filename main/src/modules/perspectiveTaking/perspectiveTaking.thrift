# Copyright: (C) 2015 WYSIWYD Consortium
# Authors: Tobias Fischer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# perspectiveTaking.thrift

/**
* perspectiveTaking_IDL
*
* IDL Interface to \ref perspectiveTaking service.
*/
service perspectiveTaking_IDL
{
   /**
   * Set the interval how often the
   * third person perspective
   * camera view should be updated
   * @param interval is the interval in ms
   * @return true/false on success/failure.
   */
   bool setUpdateTimer(1:i32 interval);

   /**
   * Set the decimation (quality) of the most
   * recent camera frame. The lower, the better
   * @param decimation is a power of 2 (1, 2, 4, 8, ...)
   * @return true/false on success/failure.
   */
   bool setDecimationOdometry(1:i32 decimation);

   /**
   * Set the decimation (quality) of for the
   * frames in the memory. The lower, the better
   * @param decimation is a power of 2 (1, 2, 4, 8, ...)
   * @return true/false on success/failure.
   */
   bool setDecimationStatistics(1:i32 decimation);

   /**
   * Decide whether or not new frames should
   * be added to the memory or not
   * @param enable sets whether the current frame
   * is added to the memory or not
   * @return true/false on success/failure.
   */
   bool processStats(1:bool enable);
}
