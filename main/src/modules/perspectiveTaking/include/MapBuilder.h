/*
 *
 * Copyright (C) 2014 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Tobias Fischer
 * email:   t.fischer@imperial.ac.uk
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * wysiwyd/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef MAPBUILDER_H_
#define MAPBUILDER_H_

#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/RtabmapEvent.h>

#include <rtabmap/utilite/UEventsHandler.h>

#include "VisualizerWrapper.h"

using namespace rtabmap;

// This class receives RtabmapEvent and construct/update a 3D Map
class MapBuilder : public UEventsHandler
{
public:
    MapBuilder(unsigned int decOdo=2, unsigned int decVis=2);
    virtual ~MapBuilder();

    void spinOnce(int time = 1, bool force_redraw = false);
    bool wasStopped();

    pcl::visualization::PCLVisualizer& getVisualizer() {
        return _vWrapper->getVisualizer();
    }

private:
    void processOdometry(const rtabmap::SensorData & data);
    void processStatistics(const rtabmap::Statistics & stats);

protected:
    virtual void handleEvent(UEvent * event);

private:
    VisualizerWrapper* _vWrapper;
    Transform lastOdomPose_;
    unsigned int decimationOdometry_;
    unsigned int decimationVisualization_;
    bool _processingStatistics;
    bool _processingOdometry;
    bool _lastPoseNull;
    boost::mutex vis_mutex;
};

#endif /* MAPBUILDER_H_ */
