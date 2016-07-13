/*
 * Copyright (C) 2015 WYSIWYD Consortium, European Commission FP7 Project ICT-612139
 * Authors: Grégoire Pointeau
 * email:   gregoire.pointeau@inserm.fr
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

/**

[O]: O "O"

  \defgroup wysiwyd_narrativeGraph narrativeGraph
  @ingroup wysiwyd_modules

[sActionEvt]: @ref storygraph::sActionEvt "sActionEvt"

[sRelation]: @ref storygraph::sRelation "sRelation"

[SituationModel]: @ref storygraph::SituationModel "SituationModel"

 Module to understand narration through a intermediate structure: the [SituationModel].
  @sa narrativeHandler
  @author Solène Mirliaz, Grégoire Pointeau
  @copyright Released under the terms of the GNU GPL v2.0

 \section intro_sec Description
 Module to build and modify a [SituationModel]. The [SituationModel] is an intermediate structure between
 the human narratives (interpreted thanks to the LRH) and the ABM.
 It is a graph representing the events that occur, organized temporally and hierarchically.

 The module is organized around three main feature:
 - Automatic creation of the [SituationModel] from the ABM
 - Creation of a LRH train using an ABM story automatically converted in a [SituationModel] on one side and a narrative on the other.
 - Use of a narrative and a trained LRH to improve or create a [SituationModel]

 \section parameters_sec Parameters
 Same as narrativeHandler.

 \section conf_file_sec Configuration Files
 - \c LRH \c lrh.ini \n
   used to train the LRH (used the \c closed_class_words parameter)
 - \c narrativeGraph \c narrativeGraph.ini \n
   specified the files used to train the LRH (\c [__narrative__] section) and the rendering parameters (\c [__rendering__] section)

   In the narrativeGraph.ini default values are:

   To train the LRH:
~~~
[narrative]
trainOutput         "Corpus/trainFromSM.txt" // Where the train produce will be append
narrativeFolder     "Corpus"                 // The folder where narratives must be searched
narrativeFile       "narrative.txt"          // The default narrative loaded
synonyms            "vocabSynonyms.txt"      // The file defining synonyms classes
pronouns			"he,it,him"              // The pronouns for Agent, Object and Recipient, in order
~~~
   And to define the parameters of rendering the [SituationModel] in SVG:
~~~
[rendering]
svgFolder  "situationModels" // The folder where the SVG representation are saved
svgFile    "default.svg"     // The default name of the SVG file for the representation
wEvtBox  140 // Width of an Event box
hEvtBox   80 // Height of an Event box
hOffset   40 // Horizontal offset between IGARF boxes
vOffset   70 // Vertical offset between IGARF boxes
~~~

 \section portsi_sec Input Ports
 - /narrativeGraph/rpc: rpc port of the module \n

 \section cmds_sec Commands

 The commands are (/narrativeGraph/rpc):

 From narrativeHandler:
  - setNarrator + name
  - askNarrate + instanceStory = default_value
  - narrate + instanceStory = default_value
  - displayStories + n-back = default_all
  - listenStory
  - cleanMental

 To create and modify the [SituationModel][] :

  -
~~~
createFromStory
createFromStory INSTANCE
~~~
Create the [SituationModel] from a story. By default it is the last story. (See storygraph::SituationModel::createFromStory)

  - helpSM
    -
~~~
listActionEvts
~~~
    -
~~~
listRels
~~~
    -
~~~
listIGARF
~~~
    -
~~~
createEvt PREDICATE AGENT
createEvt PREDICATE AGENT OBJECT
createEvt PREDICATE AGENT OBJECT RECIPIENT
~~~
Creates a new [sActionEvt][] (but don't adds it in any IGARF) and returns its instance number
    -
~~~
getRel SUBJECT VERB OBJECT
~~~
Creates a new [sRelation][] (but don't adds it in any IGARF) or find it if it already exists, and returns its instance number
    -
~~~
createIGARF
~~~
Creates a new IGARF (but none of its sub-events or sub-relations are defined) and returns its instance number
    -
~~~
showIGARF INSTANCE_NUMBER
~~~
Displays a textual representation of the IGARF in the `cout` stream of the module
    -
~~~
changeEvtIGARF INSTANCE_IGARF PART INSTANCE_EVENT
~~~
Defines the instance number for an [sActionEvt][] (INSTANCE_EVENT) either for the Action or Result part (PART= A or R) of an IGARF
    -
~~~
changeCntIGARF INSTANCE_IGARF PART INSTANCE_SUB_IGARF
~~~
Defines the instance number for an IGARF Event (INSTANCE_SUB_IGARF) either for the Action or Result part (PART= A or R) of an IGARF
    -
~~~
removeCntIGARF INSTANCE_IGARF PART
~~~
Sets to undefined either the Action or Result part (PART= A or R) of an IGARF
    -
~~~
addRelIGARF INSTANCE_IGARF PART INSTANCE_RELATION
~~~
Adds a [sRelation][] (INSTANCE_RELATION) to either the InitState, Goal or FinalState part (PART= I, G or F) of an IGARF
    -
~~~
remRelIGARF INSTANCE_IGARF PART INSTANCE_RELATION
~~~

 To train the LRH using narrative and an existing [SituationModel][]:
  -
~~~
train SENTENCE
~~~
From a sentence, searches the corresponding [sActionEvt] or [sRelation] in the [SituationModel].
Then, using this element, extracts meaning and focus. Displays both in the `cout` stream of the module
and appends them to the trainFile (@ref conf_file_sec)
  -
~~~
trainFromFile
trainFromFile FILENAME
~~~
Like the `train` command but with a whole file where each line is one sentence.
File is search in `narrativeFolder` and the default file is `narrativeFile` (@ref conf_file_sec)

 To add Narrative Links in the [SituationModel][]:

  -
~~~
learnDFW WORD
~~~
Learn a new Discourse Function Word so that it then correctly remove from OCW when processing a sentence or meaning

  - createLink + (instanceFromIGARF cPart instanceRel) + word + (instanceRoIGARF cPart instanceRel)
  - linkFromMeaning + meaning
  - createFromMeaning + meaning
  - period
  - produceBasicMeaning

 To have a visual representation of the [SituationModel][]:

  -
~~~
createSVG
createSVG FILENAME
createSVG FILENAME INSTANCE
~~~
Creates a SVG representation of the given INSTANCE of IGARF (default is last) in FILENAME.
Default Filename and Filefolder is given by the configuration file, [rendering] section.
Warning: Extention is not provide, it must be specified.

 \section tested_os_sec Tested OS
 Linux

 Contact : solene.mirliaz@ens-rennes.fr , gregoire.pointeau@inserm.fr

*/

#include "narrativeHandler.h"

int main(int argc, char * argv[])
{
    yarp::os::Network::init();
    narrativeHandler mod;
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("narrativeGraph");
    rf.setDefaultConfigFile("narrativeGraph.ini");
    rf.configure(argc, argv);
    mod.runModule(rf);
    return 0;
}
