# Copyright (c) 2015, Andreas Damianou

"""
This is a demo for unsupervised learning, ie the robot
just perceives the world. In this case, we only have observables
(and no inputs).
"""


""" 
Import necessary libraries
"""
import matplotlib as mp
# Use this backend for when the server updates plots through the X 
mp.use('TkAgg')
import numpy as np
import pylab as pb
import GPy
# To display figures once they're called
pb.ion()
default_seed = 123344
import pods
try:
    from SAM import SAM
except ImportError:
    import SAM

"""
Prepare some data. This is NOT needed in the final demo,
since the data will come from the iCub through the
drivers. So, the next section is to just test the code
in standalone mode.
"""

Ntr = 100
Nts = 50

#-- Uncomment for oil data
data = pods.datasets.oil()
Y = data['X'] # Data
L = data['Y']
L_list = data['Y'].argmax(axis=1) # Labels 

perm = np.random.permutation(Y.shape[0])
indTs = perm[0:Nts]
indTs.sort()
indTr = perm[Nts:Nts+Ntr]
indTr.sort()
Ytest = Y[indTs]
Ltest = L[indTs]
L_list_test = L_list[indTs]
Y = Y[indTr]
L = L[indTr]
L_list = L_list[indTr]

"""
This needs to go to the code, since it's what happens after data collection
"""

#--- Observables (outputs) - no inputs in this demo
# Normalise data (zero mean, std=1)
Ymean = Y.mean()
Yn = Y - Ymean
Ystd = Yn.std()
Yn /= Ystd
# Normalise test data similarly to training data
Ytest-= Ymean
Ytest /= Ystd

# Build a dictionary. Every element in the dictionary is one modality
# (one view of the data).
Data = {'Y':Yn,'L':L}


# The dimensionality of the compressed space (how many features does
# each original point is compressed to)
Q = 4

# Instantiate object
a=SAM.LFM()

# Store the events Y.
# ARG: Y: A N x D matrix, where N is the number of points and D the number
# of features needed to describe each point.
# ARG: Q: See above 
# ARG: kernel: can be left as None for default.
# ARG: num_inducing: says how many inducing points to use. Inducing points are
# a fixed number of variables through which all memory is filtered, to achieve
# full compression. E.g. it can correspond to the number of neurons.
# Of course, this is not absolutely fixed, but it also doesn't grow necessarily
# proportionally to the data, since synapses can make more complicated combinations
# of the existing neurons. The GP is here playing the role of "synapses", by learning
# non-linear and rich combinations of the inducing points.
a.store(observed=Data, inputs=None, Q=Q, kernel=None, num_inducing=40)

# In this problem each of the observables (each row of Y) also has a label.
# This can be added through the next line via L, where L is a N x K matrix,
# where K is the total number of labels. 
a.add_labels(L_list)

# Learn from the data, (analogous to forming synapses)
a.learn(optimizer='bfgs',max_iters=2000, verbose=True)

# This is an important function: It visualises the internal state/representation
#  of the memory.
ret = a.visualise()

# Only for images
#ret2= a.visualise_interactive(dimensions=(20,28))

# Pattern completion. In this case, we give a new set of test observables 
# (i.e. events not experienced before) and we want to infer the internal/compressed
# representation of those. We can then perform inference in this compressed representation.
# pred_mean is the point estimates of the inernal representation adn pred_Var is the variance
# (cenrtainty) with which they were predicted (low variance = high certainty)
pred_mean, pred_var = a.pattern_completion(Ytest)

# Visualise the predictive point estimates for the test data
pb.plot(pred_mean[:,0],pred_mean[:,1],'bx')