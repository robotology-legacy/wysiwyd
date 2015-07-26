



The config file is app/reactiveLayer/conf/default.ini

The involved parameters are in the section [ALLOSTATIC]

The parameter "drives" is a list of drive names. You can add those you want here. Drives which require modules that can't be loaded should not be a problem, they are just nor taken into account. At the current time (end of first week at VVV2015), only the "tagging" drive (for proactive tagging) plays a role (triggering behavior through the ProactiveTagging module, using rpc commands). However it is in principle feasible to add more, see below (and the # Warning notes at the end, unfortunately).

Each drive "name" (to be replaced by the actual drive name) has to define three parameters:
name-homeostasisMin, name-homeostasisMax and name-decay, corresponding to the comfort zone (CZ) lower bound, the CZ upper bound, and the decay rate (linear decay), respectively.

Optionnally you can set a priority to a drive using name-priority. Default value is 1. The drive priorities will be normalized during the module configuration to sum up to 1 and used as probabilities to be select a particular drive (only among those outside of their CZ). For example, if there are 3 drives d1, d2 and d3, and the priority of d2 is set to 3 (using the parameter d2-prioriy), the probabiliies will be 1/5 for d1, 3/5 for d2 and 1/5 for d3 (i.e. 1-3-1 normalized to sum up to 1). A priority of 0 will prevent the drive to trigger behavior. However, if only d2 and d3 are out of their CZ, the priority of d1 will be temporarily set to 0, and those for d2 and d3 renormalized accordingly (i.e P(d2)=3/4 and P(d3)=1/4). 
You can therefore play with the drive priorities in the config file to have a better control on the final robot behavior. 

Whenever a drive is out of its CZ, it can trigger behavior according to some parameters and methods. To simply trigger speech, use name-over-sentences  and name-under-sentences (and if you don't want trigger speech, just don't specify those). To trigger behavior from another module using rpc, specify the port in name-under-behavior-port and name-over-behavior-port. This port will be open and connected during the interactionEngine module configuration. In the case of using rpc, define a new method in the ReactiveLayer class to implement the message construction, by taking example of the ReactiveLayer::handleTagging method. Finally, call this new method in ReactiveLayer::updateModule(). Yes, module, file and class names are not coherent at all, we'll fix this. If you don't want to use rpc you can also directly code your behavior in a ReactiveLayer::handleSomething method (don't forget to call it in ReactiveLayer::updateModule() as in the rpc case).

To run the allostatic controller with the proactive tagging drive, launch modules in that order:

homeostasis
proactiveTagging
reactiveLayer

# Warning notes
To be honest, the code is a mess like I've rarely seen before. It relies on the original reactiveLayer from Stephane's code (which was of good quality but rather specific speech-emotional responses) and has then be adapted in an attempt to be more general by managing the triggered behaviors in other modules through rpc (as in the proactive case), instead of directly in the current module. However this has been only tried during VVV ans is still in development state (by development, understand messy). It uses Jordi's new homeostasis module, which was done to separate the generic drive implementation from the Yarp-specific module, due to the fact that we also xneed this code in other contexts than icub/wysiwyd ones. In summary, all this part has to be recoded properly. I have now sufficient knowledge to do it, thanks VVV! We just have to find a few days to do it with Jordi	.
