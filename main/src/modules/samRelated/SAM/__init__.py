# Copyright (c) 2015, Andreas Damianou

import matplotlib as mp
# Use this backend for when the server updates plots through the X 
#mp.use('TkAgg')
import numpy as np
import pylab as pb
from . import SAM_Core
from . import SAM_Drivers
from . import SAM_Demos

pb.ion()

default_seed = 123344