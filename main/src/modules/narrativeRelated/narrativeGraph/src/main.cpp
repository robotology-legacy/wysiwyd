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
# Where the train produce will be append (!Be sure it exist !)
trainOutput         "Corpus/trainFromSM.txt"
# The folder where narratives must be searched
narrativeFolder     "Corpus"
# The default narrative loaded
narrativeFile       "narrative.txt"
# The file defining synonyms classes
synonyms            "vocabSynonyms.txt"
# The pronouns for Agent, Object and Recipient, in order
pronouns			"he,it,him"
~~~
   And to define the parameters of rendering the [SituationModel] in SVG:
~~~
[rendering]
#The folder where the SVG representations are saved
svgFolder  "situationModels"
# The default name of the SVG file for the representation
svgFile    "default.svg"
# Width of an Event box
wEvtBox  140
# Height of an Event box
hEvtBox   80
# Horizontal offset between IGARF boxes
hOffset   40
# Vertical offset between IGARF boxes
vOffset   70
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
cleanSM
~~~
Removes all the events, relations, IGARF and links to start with a blank [SituationModel].

  -
~~~
ABMtoSM
ABMtoSM INSTANCE
~~~
Create the [SituationModel] from an ABM story. By default it is the last story. (See storygraph::SituationModel::createFromABM)

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
Defines the instance number for an IGARF Event (INSTANCE_SUB_IGARF) either for the Action, Result or Next part (PART= A, R or N) of an IGARF
    -
~~~
removeCntIGARF INSTANCE_IGARF PART
~~~
Sets to undefined either the Action, Result or Next part (PART= A, R or N) of an IGARF
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
SMtoTrain SENTENCE
~~~
From a sentence, searches the corresponding [sActionEvt] or [sRelation] in the [SituationModel].
Then, using this element, extracts meaning and focus. Displays both in the `cout` stream of the module
and appends them to the trainFile (@ref conf_file_sec)
  -
~~~
SMandNarrativeToTrain
SMandNarrativeToTrain FILENAME
~~~
Like the `SMtoTrain` command but with a whole file where each line is one sentence.
File is search in `narrativeFolder` and the default file is `narrativeFile` (@ref conf_file_sec)

 To add Narrative Links or Events in the [SituationModel][]:

  -
~~~
learnDFW WORD
~~~
Learn a new Discourse Function Word so that it then correctly remove from OCW when processing a sentence or meaning.
It is added for this session only (not saved at the end of the program).

  -
~~~
createLink (INSTANCE_IGARF CPART INDEX_REL) WORD (INSTANCE_IGARF CPART INDEX_REL)
~~~
Create a link with the dfw WORD, from the first event or relation pointed at to the second.
INSTANCE_IGARF is the instance number of the IGARF where the event or relation is (use command showIGARF)
CPART is the part of the IGARF it must be one of the following character: I, G, A, R or F
INDEX_REL is the index of the relation in the State (CPART = I, G or F). Set it to -1 if CPART = A or R.

  -
~~~
LRHtoSM MEANING
~~~
Improve an existing SM by adding links between known events. MEANING should have format
`dfw1 dfw2... , predicate agent object recipient` or just `predicate agent object recipient`.
The link is made between the event that has focus last and the event describe by the meaning.
This event then get the focus.

  -
~~~
LRHFiletoSM FILENAME
~~~
Same as `LRHtoSM` but all meanings are taken from the file FILENAME (in the folder narrativeFolder, must be changed in future versions)

  -
~~~
LRHtoBlankSM MEANING
~~~
Like the `LRHtoSM` command except that if the event isn't known it is added, either in Initial State
or Action cell of an IGARF. If Action cell is empty in the IGARF that has focus last, then the events
are added in that IGARF, else they are added in a new IGARF.

  -
~~~
period
~~~
Lost last focus (re-initialize the focus on the Void event (IGARF = -1)), so that next link generated by `LRHtoSM`
or `LRHtoBlankSM` is made between Void and a new event.

  -
~~~
autoStructSM
~~~
 Use the same algorithm than `ABMtoSM` to construct story arcs in the [SituationModel].
You should use this command after `LRHtoBlankSM` to build your [SituationModel].

  -
~~~
cleanLinks
~~~
Removes all links. Recommended before using `autoLink`.

 To produce a Narrative from the [SituationModel] (links must be provided):

  -
~~~
autoLink
autoLink INSTANCE_IGARF
~~~
Create basic links in the [SituationModel] so that it can be told. Start at INSTANCE_IGARF or at the last one
It first links the states in the InitState then it links them to the Action. When done it goes to Next IGARF.

  -
~~~
SMtoLRH
SMtoLRH LANG
~~~
Create Meanings and Focus for the LRH to produce sentences (a discourse).
They are produced using the links stored, in the order they were given.
Focus is naïve and created according to LANG grammar (= en by default).

 To have a visual representation of the [SituationModel][]:

  -
~~~
SMtoSVG
SMtoSVG FILENAME
SMtoSVG FILENAME INSTANCE
~~~
Creates a SVG representation of the given INSTANCE of IGARF (default is last) in FILENAME.
Default Filename and Filefolder is given by the configuration file, [rendering] section.
Warning: Extention is not provide, it must be specified.

 \section ex_sec Examples
 \subsection abmToSM ABM to SM
 To create a [SituationModel] from the ABM you must first choose a story.
 The number of stories available is given at the beginning of the module:
~~~
[INFO] BEGIN SEARCH NARRATION
[INFO] END SEARCH NARRATION
[INFO] END FINDSTORIES
[INFO] Found:  11  stories.
~~~

To have more information about the stories you can enter the command `displayStories`.

Once you have choose the story you can create the [SituationModel] with command:
~~~
ABMtoSM INSTANCE
~~~
Where INSTANCE is the number of your story (Warning: index starts at 0). If you don't precise it,
the last story will be taken.

Once you have enter the command, the system will display the textual representation of the [SituationModel] produced.
If you want a graphical representation, enter command `SMtoSVG` (@ref cmds_sec).

(Note: the command ABMtoSM clean the [SituationModel] before creating it from ABM.)

Example with command `ABMtoSM 0` (in the narrativeGraph log)
~~~{.log}
[INFO] rpc command received:  ABMtoSM 0
[INFO] create the situation model from ABM
[6]
-------------------------------------------------------------------------------
+-INIT:  [iCub want croco]  [Interlocutor has croco]  [it is out-of-reach]
+-GOAL:  [icub have croco]
+-ACTION: [0]
| [0]
| -----------------------------------------------------------------------------
| +-INIT:  [iCub want croco]  [Interlocutor has croco]  [it is out-of-reach]
| +-GOAL:  [icub have croco]
| +-ACTION: [0] iCub take croco
| +-RESULT: [1] iCub fail take
| +-FINAL:  [it is out-of-reach]  [iCub want croco]  [Interlocutor has croco]
| +-NEXT:
| -----------------------------------------------------------------------------
+-RESULT: [5]
| [5]
| -----------------------------------------------------------------------------
| +-INIT:  [iCub want croco]  [Interlocutor has croco]
| +-GOAL:  [icub have croco]
| +-ACTION: [1]
| | [1]
| | ---------------------------------------------------------------------------
| | +-INIT:  [iCub want croco]  [Interlocutor has croco]
| | +-GOAL:  [icub have croco]
| | +-ACTION: [2] iCub reason
| | +-RESULT: [3] iCub reason
| | +-FINAL:  [iCub want croco]  [Interlocutor has croco]
| | +-NEXT: [2]
| | ---------------------------------------------------------------------------
| | [2]
| | ---------------------------------------------------------------------------
| | +-INIT:  [iCub want croco]  [Interlocutor has croco]
| | +-GOAL:  [icub have croco]
| | +-ACTION: [4] iCub reason
| | +-RESULT: [5] iCub reason
| | +-FINAL:  [iCub want croco]  [Interlocutor has croco]
| | +-NEXT: [3]
| | ---------------------------------------------------------------------------
| | [3]
| | ---------------------------------------------------------------------------
| | +-INIT:  [iCub want croco]  [Interlocutor has croco]
| | +-GOAL:  [icub have croco]
| | +-ACTION: [6] iCub say Give_me_the_croco_please Interlocutor
| | +-RESULT:
| | +-FINAL:  [iCub want croco]  [Interlocutor has croco]
| | +-NEXT: [4]
| | ---------------------------------------------------------------------------
| | [4]
| | ---------------------------------------------------------------------------
| | +-INIT:  [iCub want croco]  [Interlocutor has croco]
| | +-GOAL:  [icub have croco]
| | +-ACTION: [7] Interlocutor give croco iCub
| | +-RESULT: [8] Interlocutor give croco iCub
| | +-FINAL:  [iCub has croco]
| | +-NEXT:
| | ---------------------------------------------------------------------------
| +-RESULT: [8] Interlocutor give croco iCub
| +-FINAL:  [iCub has croco]
| +-NEXT:
| -----------------------------------------------------------------------------
+-FINAL:  [iCub has croco]
+-NEXT:
-------------------------------------------------------------------------------
[INFO] import and creation sucessful
~~~

\subsection smToLrhTrain SM to LRH Train
Then you can create a train for the LRH.
For example you have this narrative (yarp/contexts/narrativeGraph/Corpus/narrative.txt) for the [SituationModel] above:
~~~
first iCub wants the croco
but when he grasps it
he fails to take
because it was out-of-reach
iCub reason
and he says Give_me_the_croco_please to Sam
Sam thus gave the croco to iCub
now iCub has the croco
~~~
Then you can run the command `SMandNarrativetoTrain` (or `SMandNarrativetoTrain otherFile.txt` if you don't want to use default)and the result should be the following in the log:
~~~{.log}
[INFO] rpc command received:  SMandNarrativetoTrain
[INFO] create a train from a narrative
[INFO] Opening  /home/rococo/.local/share/yarp/contexts/narrativeGraph/Corpus/narrative.txt
[INFO] first , wants iCub croco <o> [P-_-_-_-_-_-_-_][_-A-P-O-_-_-_-_] <o>; first iCub wants the croco
[INFO] but when , grasps he it <o> [P-A-_-_-_-_-_-_][_-_-A-P-O-_-_-_] <o>; but when he grasps it
[INFO] fails he take <o> [A-P-O-_-_-_-_-_] <o>; he fails to take
[INFO] because , was it out-of-reach <o> [P-_-_-_-_-_-_-_][_-A-P-O-_-_-_-_] <o>; because it was out-of-reach
[INFO] reason iCub <o> [A-P-_-_-_-_-_-_] <o>; iCub reason
[INFO] and , says he Give_me_the_croco_please Sam <o> [P-_-_-_-_-_-_-_][_-A-P-O-R-_-_-_] <o>; and he says Give_me_the_croco_please to Sam
[INFO] thus , gave Sam croco iCub <o> [_-P-_-_-_-_-_-_][A-_-P-O-R-_-_-_] <o>; Sam thus gave the croco to iCub
[INFO] now , has iCub croco <o> [P-_-_-_-_-_-_-_][_-A-P-O-_-_-_-_] <o>; now iCub has the croco
[INFO] creation successful
~~~
This result is append in the default trainOuput. The LRH can use it to be trained.
~~~
first , wants iCub croco <o> [P-_-_-_-_-_-_-_][_-A-P-O-_-_-_-_] <o>; first iCub wants the croco
but when , grasps he it <o> [P-A-_-_-_-_-_-_][_-_-A-P-O-_-_-_] <o>; but when he grasps it
fails he take <o> [A-P-O-_-_-_-_-_] <o>; he fails to take
because , was it out-of-reach <o> [P-_-_-_-_-_-_-_][_-A-P-O-_-_-_-_] <o>; because it was out-of-reach
reason iCub <o> [A-P-_-_-_-_-_-_] <o>; iCub reason
and , says he Give_me_the_croco_please Sam <o> [P-_-_-_-_-_-_-_][_-A-P-O-R-_-_-_] <o>; and he says Give_me_the_croco_please to Sam
thus , gave Sam croco iCub <o> [_-P-_-_-_-_-_-_][A-_-P-O-R-_-_-_] <o>; Sam thus gave the croco to iCub
now , has iCub croco <o> [P-_-_-_-_-_-_-_][_-A-P-O-_-_-_-_] <o>; now iCub has the croco
~~~
To know where files are taken, see @ref conf_file_sec. If there is an error check that all files exist.

Note: If you want to have the meaning for one sentence only, you should use `SMtoTrain sentence`
Warning: Sometime, the content isn't written in the output file because it is open and edited with another program.

\subsection lrhToSM LRH to SM
You can improve a SM by adding link. You first give a sentence to the LRH to obtain its meaning.
Then you give this meaning to the [SituationModel] so that it adds links between the events it knows.
For example you have a new narrative (yarp/contexts/narrativeGraph/Corpus/narrative2.txt) for the same [SituationModel]:
~~~
iCub said Give_me_the_croco_please to Sam
because he failed to take it
Sam gave the toy to iCub
because iCub wanted it
~~~
You send it to the LRH which reply:
~~~
said iCub Give_me_the_croco_please Sam
because,failed he take it
gave Sam toy iCub
because,wanted iCub it
~~~
If it is in the file yarp/contexts/narrativeGraph/Corpus/meanings.txt
Then you can run the command `LRHFiletoSM meanings.txt` and the result should be the following in the log:
~~~{.log}
command received: LRHFiletoSM meanings.txt
[INFO] rpc command received:  LRHFiletoSM meanings.txt
[INFO] Opening  /home/rococo/.local/share/yarp/contexts/narrativeGraph/Corpus/meanings.txt
[INFO]: From A of IGARF -1 to A of IGARF 3
[INFO]because: From A of IGARF 3 to A of IGARF -1
[INFO]: From A of IGARF -1 to A of IGARF 4
[INFO]because: From A of IGARF 4 to I of IGARF 0
[INFO] creation successful
~~~
Four links where created: two without a dfw (lines 4 and 6) and two other with "because" (lines 5 and 7).
Use the command `listLinks` if you want to see all the links created.

\subsection lrhToBlankSM LRH to Blank SM
You can also create a new [SituationModel] only from a narrative.
__It is higly recommended to clean your [SituationModel] (command `cleanSM`) before starting using this command.__
Use the command `LRHtoBlankSM MEANING` (see @ref cmds_sec).
~~~{.unparse}
cleanSM
LRHtoBlankSM "wanted I moon"
LRHtoBlankSM "is it far"
LRHtoBlankSM "take I ship"
LRHtoBlankSM "is ship broken"
LRHtoBlankSM "failed I fly"
LRHtoBlankSM "am I sad"
autoStructSM
~~~
The result of the last command in the log:
~~~{.log}
[INFO] rpc command received:  autoStructSM
[INFO] auto structuration of the situation model
[0]
-------------------------------------------------------------------------------
+-INIT:  [I wanted moon]  [moon is far]
+-GOAL:
+-ACTION: [0] I take ship
+-RESULT:
+-FINAL:
+-NEXT: [1]
-------------------------------------------------------------------------------
[1]
-------------------------------------------------------------------------------
+-INIT:  [ship is broken]
+-GOAL:
+-ACTION: [1] I failed fly
+-RESULT:
+-FINAL:
+-NEXT: [2]
-------------------------------------------------------------------------------
[2]
-------------------------------------------------------------------------------
+-INIT:  [I am sad]
+-GOAL:
+-ACTION:
+-RESULT:
+-FINAL:
+-NEXT:
-------------------------------------------------------------------------------
[INFO] autostruct sucessful
~~~

\subsection smToLrh SM to LRH
If the [SituationModel] already have discourse links, it is possible to tell the story. Else, it is possible to create basic links with command `autoLink INSTANCE_IGARF`.
__It is highly recommended to clean all links existing (command `cleanLinks`) before launching that command.__

The following commands are send to the [SituationModel] above:
~~~
cleanLinks
autoLink 0
~~~
The log of last command should be:
~~~{.log}
[INFO] rpc command received:  listLinks
: From A of IGARF -1 to I of IGARF 0
and: From I of IGARF 0 to I of IGARF 0
so: From I of IGARF 0 to A of IGARF 0
: From A of IGARF -1 to I of IGARF 1
so: From I of IGARF 1 to A of IGARF 1
: From A of IGARF -1 to I of IGARF 2
[INFO] listing sucessful
~~~

Finally, you can ask for the narration. It will follow the links in the order they were given to tell the story.
It returns meanings that the LRH can then process into sentences.
Command is:
~~~
SMtoLRH
~~~
Result in the log is:
~~~{.log}
wanted I moon <o> [A-P-O-_-_-_-_-_] <o>;
and , is moon far <o> [P-_-_-_-_-_-_-_][_-A-P-O-_-_-_-_] <o>;
so , take I ship <o> [P-_-_-_-_-_-_-_][_-A-P-O-_-_-_-_] <o>;
is ship broken <o> [A-P-O-_-_-_-_-_] <o>;
so , failed I fly <o> [P-_-_-_-_-_-_-_][_-A-P-O-_-_-_-_] <o>;
am I sad <o> [A-P-O-_-_-_-_-_] <o>;
~~~

If the LRH is well trained, the result after giving him these sentences (command `production MEANING`) should be:
~~~
I wanted the moon
and the moon is far
so I take a ship
the ship is broken
so I failed to fly
I am sad
~~~

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
