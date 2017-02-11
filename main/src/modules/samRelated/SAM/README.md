# SAM_Sheffield

Authors: Uriel Martinez, Daniel Camilleri, Andreas Damianou, Luke Boorman
email:   d.camilleri@sheffield.ac.uk

A Synthetic Autobiographical Memory for the iCub robot. This module contains the core, which implements the high-level functionality, and the drivers which act as middleware between the perceptual system and the core.

Pre-trained Action Recognition model and data can be found at the following link:
https://drive.google.com/open?id=0B6fkkRLTYjNLbjFFSEZtUmtHNk0

##Features

samSupervisor at this point can manage multiple models with great flexibility in training and interacting with models. 

trainSAMModel allows for the training of multiple, single or temporal models by simply changing the value in the config.ini of the particular model and automates testing of the different types of models.

interactionSAMModel also allows for great flexibility in the method of data collection. 3 Methods are currently supported.

-`continuous` : 	Data is continuously classified by the driver with a buffer of classifications being kept. Each time a classification is queried, the classification buffer decreases in size until it reaches 0 and returns None

-`buffered` :		In this case, a buffer of the latest n data points are kept and when the classification command is sent, these data points are sent to the driver for classification which is returned immediately. 

-`future_buffered` : 	In this case, when a classification request is sent, the future n data points are collected, classified and a classification returned.

Setting a model to any one of these data collection methods and also specifying the buffer lengths is done within sensory_level_config.ini

##PreRequisites

This module is using:
- Python code which is included in this repo
And requires:
- GPy dependency from https://github.com/SheffieldML/GPy. The best way to include this is:
    1) clone the GPy repository https://github.com/SheffieldML/GPy.git
    2) switch to branch "devel"
    3) Include the directory where you cloned GPy in your PYTHONPATH.
    4) Move to the GPy directory and run: "python setup.py install" (try "python setup.py build_ext --inplace" if you want to do the installation in the same directory).
- GPyOpt dependency from https://github.com/SheffieldML/GPy. The best way to include this is:
    1) sudo apt-get install python-pip
    2) pip install gpyopt

If this does not work:
    1) clone the GPyOpt repository https://github.com/SheffieldML/GPy.git
    2) Include the directory where you cloned GPyOpt in your PYTHONPATH.
    4) Move to the GPyOpt directory and run: "python setup.py develop"

##How to use:

Everything is accessed via **samSupervisor** which is installed to **WYSIWYD_DIR**.

Connect with samSupervisor via `/sam/rpc:i` and issue help for a list of all possible commands

-`check_all`: 		Check data and config.ini availabilities for that model, as well as current status (Training or Loaded) and check if model is curently up to date

-`check <modelName>`: 		Check the same points as above but for a single model

-`close <modelName>`: 		Terminates a currently loaded interaction process for the particular model

-`delete <modelName>`: 		Physically deletes the trained model for `<modelName>`

-`help`: 			Provides a list of available commands

-`load <modelName>`: 		Launches an interaction process for the corresponding model. If model already present, reloads model from disk

-`optimise <modelName>`: 	Optimises the parameters of the model via Bayesian Optimisation to achieve the best result. Require GPyOpt to be installed

-`quit`: 			Terminates samSupervisor

-`train` <modelName>:		Launches a training process for the corresponding model

-`list_callSigns`:		Compiles a list of currently active callSigns. Where callSigns are the messages that loaded models respond to

When a model is loaded, to retrieve a classification of a generation from the model, issue the respective command in the list_callSigns list. There is a timeout of 10 seconds for the response so as not to block the operation of samSupervisor


##Prerequisites to use samSupervisor:

A folder, `<FolderName>` which has the following folder structure:

`<FolderName>`:
  - `Data:`
    - `<model1Name>`
    - `<model2Name>`
    - `<model3Name>`
      - `<model3SubModelName1>`
      - `<model3SubModelName2>`
      - `<model3SubModelNameN>`
    - `<modelMName>`
  - `Models:`

Where `<model1Name>` and `<model2Name>` will be modelled as a single model while `<model3Name>` will have multiple models depending on the number of subfolders in `<model3Name>`. 

##Important Notes:

1. Each `<modelXname>` folder must contain data as well as a `config.ini` which specifies the parameters for training the data found in that folder.

2. To carry out training, **samSupervisor** requires at least one `<modelXName>` folder.

3. samSupervisor on startup compiles a list of models that are available for training. The models are referenced according their `<modelXName>`

4. `<FolderName>/Models` is left empty and is used by samSupervisor to store trained models. 

5. You can get an example `<FolderName>` from ______ which contains data, config.ini and pre-trained models for ______

6. Finally, modify `default.ini` in **samSupervisor** context to point to your `<FolderName>` and comment out models within `sensory_level_conf.ini` also found in the **samSupervisor** context which you do not require to run automatically when launching samSupervisor 

7. `default.ini` has 3 options. 
persisitence: which defines if windows opened should remain open or close auomatically upo termination. Set to true this is useful to debug training or interaction algorithms.
windowed:     which defines if training and interaction functions should spawn a window or not
verbose:      defines the level of verbosity of samSupervisor

8. `sensory_level_conf.ini` specifies which models should be loaded as sections, the names of their respective input/output ports as well as the callsigns which will trigger recall or recognition of that particular model

9. All **PORTS**, **RPCBASES**, and **CALLSIGNS** must be unique

##Breakdown of contents in SAM folder:

####SAM_Core: 
- Includes the base classes for SAM_Core and SAM_Driver
- Includes samSupervisor.py, trainSAMModel.py, interactionSAMModel.py classes which are installed into WYSIWYD_DIR
-Implements the core functionality of SAM. This module is the memory system where the already transformed (by SAM_Drivers) perceived signals are compressed and stored in a coherent way. Coherent meaning that audio, visual etc signals are treated in analogous manner. This model is built upon the deep Gaussian process model using GPy, so that high-dimensional, noisy data can be compressed, chunked and "cleaned" automatically in a probabilistic way (ie the model is trying to keep the "relevant" variance in the data and eliminate redundancies in the representation by encoding memories as "clean" and non-redundant signals).

####SAM_Drivers:
- This folder contains all developed drivers. 
- These drivers are accessed via the generic trainModel and interactionModel classes which are called from samSupervisor
- This module contains the implementation for transforming the raw perceptual signal into preprocessed signal. For example, in the biological brain, the visual cortex is hierarchically processing the visual stimuli before the signal is reaching the brain. In the current implementation, the drivers are doing this work, and currently we have implemented: Faces, Actions

##License
Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
website: http://wysiwyd.upf.edu/
Permission is granted to copy, distribute, and/or modify this program under the terms of the GNU General Public License, version 2 or any later version published by the Free Software Foundation.

A copy of the license can be found at $WYSIWYD_ROOT/license/gpl.txt

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details
