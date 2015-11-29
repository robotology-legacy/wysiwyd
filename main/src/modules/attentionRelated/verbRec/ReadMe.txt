ACTION RECOGNIZER (verbRec): Determines the subset of the actions/verbs GIVE, GRASP, HAVE, LIFT, MOVE, POINT, PULL, PUSH, PUT, TAKE and WAVE
                  that are currently performed by an external agent. In the case of the verb HAVE the module determines both the set of objects
                  held by the external agent and the set of objects held by the robot itself. The module detects which actions/verbs that are
                  ongoing at each time step by deciding on whether specific combinations of conditions, such as, for example, object movements,
                  joint movement and distances, are fulfilled or altered in specific ways for the preprocessed input obtained from other modules
                  in the robot's cognitive architecture. An action performed by the agent will therefore yield output about the action during the
                  sequence of time steps that the action is ongoing.
REFERENCES:
INPUT: The joint positions of the agent, the position of the robot and the positions and identities of the present objects.
PROCESSING: Detects the set of actions/verbs currently performed by an external agent and what objects they are applied to if any.
OUPUT: The detected actions (verbs) and the objects they are applied to.

'''''''''''''''''''''''''''''

* In a first console: Launch a yarpserver.

* In a second console: Launch yarpdataplayer and open dumpHuman wave

* In a third console: yarp read /output_data

* In a fourth console: verbRec

* Run yarpdataplayer
