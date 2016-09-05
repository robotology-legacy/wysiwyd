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

np.random.seed(default_seed)

"""
Prepare some data. This is NOT needed in the final demo,
since the data will come from the iCub through the
drivers. So, the next section is to just test the code
in standalone mode.
"""

Ntr = 200
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
L += np.random.randn(*L.shape)*0.001 # Add a bit of noise to training label data for numerical stability
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
a=SAM.SAM_Core.LFM()

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
a.learn(optimizer='bfgs',max_iters=1000, verbose=True)

# This is an important function: It visualises the internal state/representation
#  of the memory.
ret = a.visualise()
ret.set_title('Memory space visualisation (Training)'); pb.draw()

ret1 = a.visualise(which_indices=SAM.SAM_Core.most_significant_input_dimensions(a.model.bgplvms[-1],None),plot_scales=False)
ret1.set_title('Memory space for the label-specific significant dimensions'); pb.draw()

# Train reverse model for uncertainty prediction
a.familiarity_reverse(Ytest=None, source_view=0)

# Only for images
#ret_in= a.visualise_interactive(dimensions=(20,28))

# Pattern completion. In this case, we give a new set of test observables 
# (i.e. events not experienced before) and we want to infer the internal/compressed
# representation of those. We can then perform inference in this compressed representation.
# pred_mean is the point estimates of the inernal representation adn pred_Var is the variance
# (cenrtainty) with which they were predicted (low variance = high certainty)
predictions = a.pattern_completion(Ytest)
pred_mean = predictions[0]
pred_var = predictions[1]

# Visualise the predictive point estimates for the test data; plot them on top of the memory visualization
ret2 = a.visualise(plot_scales=False)
ret2.plot(pred_mean[:,0],pred_mean[:,1],'xm')
ret2.set_title('Mapping test data onto the memory space as magenda x''s'); pb.draw()

pb.show()

# Example of pattern completion inference:
index_to_test = 10 # try different ones
pred_label = a.pattern_completion_inference(Ytest[index_to_test,:][None,:]).values
rev_pred_uncert = a.familiarity_reverse(Ytest=Ytest[index_to_test,:][None,:])[1]
print('')
print("# I predict label " + str(np.round(pred_label).argmax()) + " with uncertainty " + str(rev_pred_uncert) + " and correct label is: " + str(Ltest[index_to_test,:].argmax()))
