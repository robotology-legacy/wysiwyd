# Copyright (c) 2015, Andreas Damianou
<<<<<<< 3657c0e778f3e2021d8d8caee04072555fb57629
<<<<<<< 854f4746d5eb91a2612f255559418e136171cc13
<<<<<<< 1f58a55701ad7ae6acb758ebedcece9da3cc1ce2
<<<<<<< 0fc049c41ac91b2dab4d5c1a55bcbea931f3c422
# from .SAMDriver_speech import *
from .SAMDriver_AR import *
from .SAMDriver_temporalActions import *
from .SAMDriver_temporal import *
=======
=======
=======
<<<<<<< 08e1b9b6b4f32ccd2b7b29bc5b21dee094de17b4
>>>>>>> updated version of samSupervisor and generic train and interaction implementations
<<<<<<< b809f2aecd078104d91d3f1153eadd625e488a28
>>>>>>> updated SAM folder
from .SAMDriver_faces import *
#from .SAMDriver_actions import *
#try:
#    import yarp
#    from .SAMDriver_interaction import *
#except ImportError:
#    print 'WARNING! SAMDriver_interaction not imported because yarp is missing!'
>>>>>>> New version of samSupervisor with rpc interface and README
from .SAMDriver_interaction import *
=======

import matplotlib as mp
# Use this backend for when the server updates plots through the X 
#mp.use('TkAgg')
import numpy as np
import pylab as pb
from . import SAM_Core
from . import SAM_Drivers
#from . import SAM_Demos

pb.ion()

default_seed = 123344
>>>>>>> updated SAM folder
=======
=======
>>>>>>> Updated fix/samRelated with generic train and interactions files. Added optimiser to samSupervisor and fixed some bugs
# from .SAMDriver_speech import *
from .SAMDriver_AR import *
from .SAMDriver_temporalActions import *
from .SAMDriver_temporal import *
from .SAMDriver_interaction import *
