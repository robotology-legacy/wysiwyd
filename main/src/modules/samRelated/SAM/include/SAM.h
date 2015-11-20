// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
* Authors: Uriel Martinez, Daniel Camilleri, Andreas Damianou, Luke Boorman
* email:   d.camilleri@sheffield.ac.uk
* website: http://wysiwyd.upf.edu/
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* $WYSIWYD_ROOT/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

/*
* @defgroup SAM
* @ingroup wysiwyd_modules
*
* A Synthetic Autobiographical Memory for the iCub robot. This module contains the core, which implements the high-level functionality, and the drivers which act as middleware between the perceptual system and the core.
*
* This module is using: 
* - C++ code which is included in this repo
* - Python code which is included in this repo
* And requires:
* - GPy dependency from https://github.com/SheffieldML/GPy. The best way to include this is:
*    1) clone the GPy repository https://github.com/SheffieldML/GPy.git
*    2) switch to branch "devel"
*    3) Include the directory where you cloned GPy in your PYTHONPATH.
*    4) Move to the GPy directory and run: "python setup.py install" (try "python setup.py build_ext --inplace" if you want to do the installation in the same directory).
* - OpenCV 2.4.11 compiled from source to enable cuda.
*
* STRUCTURE OF THE CODE:
* ---------------------
*
* samrelated contains SAM_Core, SAM_Demos and SAM_Drivers. The CMakeLists.txt takes care of including these directories, but the samrelated can also be run as a Python module in standalone mode, by doing "import SAM" in Python. This will import the SAM python module, excluding anything that has to do with yarp and the robot (ie the core and certain demos will still work and you can run them with toy data, useful for debugging without a robot).
*
* SAM_Core: Implements the core functionality of SAM. This module is the memory system where the already transformed (by SAM_Drivers) perceived signals are compressed and stored in a coherent way. Coherent meaning that audio, visual etc signals are treated in analogous manner. This model is built upon the deep Gaussian process model using GPy, so that high-dimensional, noisy data can be compressed, chunked and "cleaned" automatically in a probabilistic way (ie the model is trying to keep the "relevant" variance in the data and eliminate redundancies in the representation by encoding memories as "clean" and non-redundant signals).
*
* SAM_Drivers: This module contains the implementation for transforming the raw perceptual signal into preprocessed signal. For example, in the biological brain, the visual cortex is hierarchically processing the visual stimuli before the signal is reaching the brain. In the current implementation, the drivers are doing this work, and currently we have implemented:
*   - VisionDriver: For visual stimuli. Currently it works to recognize and track faces (more info locally in the driver file).
*   - SpeechInteraction: For speech recognition and interaction. More info locally in the driver file.
*   - abmActions: For interacting with ABM. WARNING: This is still under development.
*
* SAM_Demos: Some of the most important demos demonstrating SAM. Currently:
*   - demo_interaction.py: This is for recognizing faces and interacting by speech with the iCub (a preselected vocabulary has to be used for the speech interaction)
*   - demo_faces.py: A subset of the above. This demo tracks faces and recognizes (without interaction). This demo can be run without a robot if you provide test data to the python module.
*   - demo_actions.py: A demo for recognizing actions and depends on vision driver and speech interaction driver. Currently actions recognized are push object left, push right, lift up, put down, waving, and these are recognized with both arms.
*
*
* RELATION TO ABM:
* ----------------
*
* WYSIWYD also contains the ABM module which is very related to the SAM. Currently ABM is used for long term memory (in practice it's used for storage) and SAM is used for a more cognitive, high-level (and short-term memory), in the sense that it implements bio-inspired concepts, such as compression, chunking, pattern completion etc. These two modules are complementary. In practice, the output from the SAM (e.g. a trained model) can be stored in ABM. Furthermore, in the next level (episodic memory), the temporal aspect of the memory can be implemented somewhere in between, eg using SAM to organize in a temporal manner events stored in ABM.
*
* 
* \author Uriel Martinez
* \author Daniel Camilleri
* \author Andreas Damianou
* \author Luke Boorman
* Copyright (C) 2015 WYSIWYD Consortium\n
* CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
*
*/

