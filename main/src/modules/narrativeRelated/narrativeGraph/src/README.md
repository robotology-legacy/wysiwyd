# narrativeGraph

Authors: Grégoire Pointeau, Solène Mirliaz
email:   gregoire.pointeau@inserm.fr

A narrative system to discuss with the ABM and reason about stories.


### Initialisation of the system and main parameters
The aim of the system is to create story under the format of a graph of IGARF (for Initial, Goal, Action, Result, Final).
To do so, the system must be connected to the `autobiographicaMemory`
narrativeGraph will then search for story in the ABM
This research is conditionned by parameters in the narrativeGraph.ini file:

- dThresholdDiffStory			default: 10.0       delay in second where the end of an action, and the beggining of the following are considered as part of the same story
- iThresholdSizeStory			default: 2          minimum size of event to create a story
- researchWindows               default: 1          boolean. Define the type of research of the story in the ABM. If 1 (true) the system will search for instance included in the interval "instanceStart - instanceStop", if 0 (false) the system will take the n-lask instance (define at nback parameter)
- instanceStart		            in case of research by window, starts the research in the ABM at this instance
- instanceStop		            in case of research by window, stops the research in the ABM at this instance
- nBackSize    10000            in case of research by n-last, define the number of last instance taken into account.



### Other parameters
- storyToNarrate              default: 9824           If the robot is ask to use a prerecorded narration, it will be set on the story starting at this instance
- scenarioToRecall			  0                       In the case of the HRI interaction triggered, select the story (not instance) to refer. 0 is the oldest. Can be negative (ie: -1 is the newest, -2 the second newest ...)
- initialize				  default: 1              Boolean to decide if the system should train the NarrativeFunctionWords capacities
- forwardABM                  default: 0              Boolean to decide is the narration of the robot should populate the ABM (risk of loops)
- loadNaives                  default: 0              Boolean to decide if the data comming from naive subjects should be tested
- listScenario                                        list of text files with narration scenario corresponding to precise story (obsolete)
- listMeaning                                         list of text files with train data for the NFW system
- listNaives                                          list of text files with data from the naive subjects

- GrammarNarration                                    grammar file for the speech recognizer for narrating a full story to the robot
- GrammarYesNo                                        Yes/no grammar for easy confirmation with the robot
- GrammarQuestionDFW                                  grammar file used for the HRI

- iThresholdScoreIGARFPAOR    default: 13             minimum score to detect if a given PAOR is part of an IGARF

- svgFolder       default: "situationModels"          folder to export the situation model
- svgFile         default: "situationModel.svg"       name of the file to export the situation model


### Useful Command available: 
- HRI                                                   ;       main external command: trigger the Human-Robot Interaction
- displayKnownNarrations                                ;       display all the available narration of the robot
- displayKnownStories                                   ;       display the list of the story created
- displayStories + n-back = default_all                 ;       display the easy narration of the n-last stories
- exportDFW                                             ;       export the analysis of the DFW into a file in the context folder that can be analysed with a R script
- useDFW                                                ;       rpc command to test the DFW training
- quit \n";


### Vocal command through HRI:
Questionning: What happened:
-	What happened + DFW (first/then/finally)
-	What happened + DFW (because/after/before) + agent + predicate (+ object + recipient)
-	Why did + agent + predicate (+ object + recipient)
-	Why is that / Why did this happened
-	What else

-	Do you remember when (confirmed by robot saying: "hum hum"   must be followed by one of the possibilities:
    -	DFW (first/then/finally) + agent + predicate (+ object + recipient)       (example: first Greg have the brain)
    -	agent + predicate (+ object + recipient) + DFW (because/before/but/and …) + agent + predicate (+ object + recipient)    (example: Allan gave the mouse to you but you wanted the croco)



### lrh
Each input/ouput is sent to the module `lrh` for analysis.
to be compiled and run, `lrh` requires: python, and the librairy Oger installed. Information about how to install Oger can be found at: http://reservoir-computing.org/installing_oger
lrh can be tested with the following command:
yarp rpc /lrh/rpc:
>  <UNDEFINED> meaning "first I have the brain" ; response: "first, have I brain, P1 P2 A2 O2""
>  <UNDEFINED> production "first, have I brain <o>[P-_-_-_][_-A-P-O]<o>" ; response: "first I have the brain"


### Train the NFW system
In order to know how to use NFW the system needs to be trained. To do so, we create stories in the ABM using `opcPopulater` that will be linked to the files in the conf folder containing the sentences related to the story.

Populating the ABM:
launch `opcPopulater`
>  <UNDEFINED>  `yarp rpc /opcPopulater/rpc`
>  <UNDEFINED>  populateScenario 1          ;       response: "populating scenario 1 done !"
>  <UNDEFINED>  populateScenario 2          ;       response: "populating scenario 2 done !"
>  <UNDEFINED>  populateScenario 3          ;       response: "populating scenario 3 done !"
>  <UNDEFINED>  populateScenario 4          ;       response: "populating scenario 4 done !"
>  <UNDEFINED>  populateScenario 5          ;       response: "populating scenario 5 done !"
>  <UNDEFINED>  populateScenario 6          ;       response: "populating scenario 6 done !"

get the first instance of the scenario 1

launch narrativeGraph with options: `initialize 1` and be set the research windows to `1`, instanceStart at the first instance of the scenario 1, and be sure that all instances of the 6 scenarios are included in the window (with the parameter instanceStop)
narrativeGraph should display a warning message it the training is not complete: "n sentences losts"

if not the system should display: 
> [INFO] 0 sentences lost.  
> <INFO>   5  stories 
> <INFO> 
> <UNDEFINED>  
> <UNDEFINED>  ---------------------------------------------- 
> <UNDEFINED>  
> <UNDEFINED>  narrativeGraph  ready ! 



