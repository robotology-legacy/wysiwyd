This is the refactored version of the reactive layer as done by UPF for the London integration meeting (December 2015).

# How

To launch the proactive demo (assuming that the standard modules are running -- OPC etc...), you have to launch these modules:

    ears
    proactiveTagging
    homeostasis
    sensationManager
    allostaticController
    behaviorManager

We'll provide soon a proper xml file to make it easier. For the time being it will launch the proactive demo as it was after BCBT. We are currently adding other drives and behaviors (looking at partner, naming known objects ...).


# What

The reactive layer resides for the time being in the `modules/interactionEngine` directory. We'll move it soon to a `reactiveLayer` one (which for the time being contains the EFAA version of it, that we will rename `reactiveLayerEFAA`).

Anyway, it contains three subfolders:

- `allostaticController`: a module dealing with the creation of drives and their connections to the modules below. Drives are configured in `app/allostaticController/conf`. It defines the dynamics of each drive (ranges, decay, etc...), which sensation modulates the dynamics (see `sensationManager` below) as well as which behavior to trigger when the drive is under or over threshold (see `behaviorManager` below). Internally, the `allostaticController` module relies on the `homeostasis` one, which deals only with the drive dynamic and allows the drives to be continuously updated even when the `allostaticController` is waiting for a behavior to terminate (through RPC).
- `behaviorManager`: a module containing a set of behaviors. It can receive RPC commands of the form `behavior_name` which trigger the corresponding behavior (usually sent by the `allostaticController`). Each behavior is (actually, will be) responsible of the possible connections to other modules in order to gather the extra information it might need (e.g. an object name for pointing). 
- `sensationManager`: a module containing a set of sensations, in the aim of modulating the  dynamics in `allostaticController`. For example, in the context of the proactive demo, there is a sensation which is active when there are unknown objects in the scene and which is connected to the `tagging` drive.


# Why

The former `reactiveLayer` module has become an unmanageable mess during the last year for a number of reasons. When Stéphane left SPECS, it let a clean reactive layer module. However it was limited to associate drives with speech sentences or face expressions. In the following integration meetings (Sheffield, VVV, BCBT), we needed something more general: associating any kind of behavior to a drive, and modulating those drives in a more complex way according to input sensations. This was done directly by adding more and more specific code to the `reactiveLayer` module file. Hence the mess ;)

The new version separates the reactive layer in three modules, `sensationManager`, `allostaticController` and `behaviorManager`, allowing a better management of the code and an easy way to add new sensations, drives and behaviors using a common interface, and to configure them via config files . Moreover it fit with the original DAC pipeline for the reactive layer.