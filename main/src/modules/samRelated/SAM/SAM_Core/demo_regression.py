# Copyright (c) 2015, Andreas Damianou

""" 
This demo implements a supervised learnin scenario, where observations
are accompanied by inputs. So, we have pairs of inputs-observations,
eg. note C -> frequency 98 etc. 
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
data=dict()
N=80
Nts=20
X = np.array(range(N))*0.14
X=X[None,:].T
Y = np.sin(X) + np.random.randn(*X.shape) * 0.05
perm = np.random.permutation(X.shape[0])
indTs = perm[0:Nts]
indTs.sort()
indTr = perm[Nts:]
indTr.sort()
Xtest = X[indTs]
Ytest = Y[indTs]
X = X[indTr]
Y = Y[indTr]

#data = pods.datasets.olympic_marathon_men()
# Y = data['Y']
# X = data['X']


"""
This needs to go to the code, since it's what happens after data collection
"""
# Normalise data (zero mean, std=1)
Ymean = Y.mean(0)
Y = Y - Ymean
Ystd = Y.std(0)
Y /= Ystd

"""
For the following, see demoOneView.py.
The only difference here is that we also give inputs X to the store functionp
and that we don't have labels associated with our data.
"""
Ydict = {'Y':Y}
a=SAM.LFM()
a.store(observed=Ydict, inputs=X, Q=None, kernel=None, num_inducing=20)
a.learn()
ret = a.visualise()

"""
In the supervised setting, pattern completion consists of giving new inputs
and producing new outputs
"""
pred_mean, pred_var = a.pattern_completion(Xtest) # a.model.predict(Xtest)
pb.figure()
pb.plot(Xtest, Ytest, 'r-x')
pb.plot(Xtest, pred_mean, 'b-x')
pb.axis('equal')
pb.title('Pattern Completion given Novel Inputs')
pb.legend(('True Location', 'Predicted Location'))

sse = ((Ytest - pred_mean)**2).sum()
print('Sum of squares error on test data: ' + str(sse))