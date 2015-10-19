import GPy
import numpy as np
import matplotlib.cm as cm
import itertools
import pylab as pb
from GPy.plotting.matplot_dep import dim_reduction_plots as dredplots
import cPickle as pickle

# try:
#     from mpi4py import MPI
# except:
#     print "mpi not found"

# from SAM import SAM
# d={'first':1,'second':2}
# a=SAM.pSAM(d)


"""
SAM based on Latent Feature Models
"""
class LFM(object):
    def __init__(self):
        self.type = []
        self.model = []
        self.observed = None
        self.inputs=None
        self.__num_views=None
        self.Q=None
        self.N=None
        self.num_inducing=None
        self.namesList=None

    def store(self,observed, inputs=None, Q=None, kernel=None, num_inducing=None, init_X='PCA'):
        """
        Store events.
        ARG: obserbved: A N x D matrix, where N is the number of points and D the number
        of features needed to describe each point.
        ARG: inputs: A N x Q matrix, where Q is the number of features per input. 
        Leave None for unsupervised learning.
        ARG: Q: Leave None for supervised learning (Q will then be the dimensionality of
        inputs). Otherwise, specify with Q the dimensionality (number of features) for the
        compressed space that acts as "latent" inputs.
        ARG: kernel: for the GP. can be left as None for default.
        ARG: num_inducing: says how many inducing points to use. Inducing points are
        a fixed number of variables through which all memory is filtered, to achieve
        full compression. E.g. it can correspond to the number of neurons.
        Of course, this is not absolutely fixed, but it also doesn't grow necessarily
        proportionally to the data, since synapses can make more complicated combinations
        of the existing neurons. The GP is here playing the role of "synapses", by learning
        non-linear and rich combinations of the inducing points.
        """
        assert(isinstance(observed,dict))
        self.observed = observed
        self.__num_views = len(self.observed.keys())
        self.Q = Q
        #self.D = observed.shape[1]
        self.N = observed[observed.keys()[0]].shape[0]
        self.num_inducing = num_inducing
        if num_inducing is None:
            self.num_inducing = self.N
        if inputs is None:
            if self.Q is None:
                self.Q = 2#self.D
            if self.__num_views == 1:
                assert(self.type == [] or self.type == 'bgplvm')
                self.type = 'bgplvm'
            else:
                assert(self.type == [] or self.type == 'mrd')
                self.type = 'mrd'
        else:
            assert(self.type == [] or self.type == 'gp')
            assert(self.__num_views == 1)
            self.Q = inputs.shape[1]
            self.type = 'gp'
            self.inputs = inputs

        if kernel is None:
            kernel = GPy.kern.RBF(self.Q, ARD=True) + GPy.kern.Bias(self.Q) + GPy.kern.White(self.Q)

        if self.type == 'bgplvm':
            Ytmp = self.observed[self.observed.keys()[0]]
            pcaFailed = False
            if init_X == 'PCA':
                try:
                    self.model = GPy.models.BayesianGPLVM(Ytmp, self.Q, kernel=kernel, num_inducing=self.num_inducing)
                except ValueError:
                    pcaFailed = True
                    print "Initialisation with PCA failed. Initialising with PPCA..."
            elif init_X == 'PPCA' or pcaFailed:
                print "Initialising with PPCA..."
                Xr = GPy.util.linalg.ppca(Ytmp, self.Q, 2000)[0]
                Xr -= Xr.mean(0)
                Xr /= Xr.std(0)
                self.model = GPy.models.BayesianGPLVM(Ytmp, self.Q, kernel=kernel, num_inducing=self.num_inducing, X=Xr)
            self.model['.*noise']=Ytmp.var() / 100.
        elif self.type == 'mrd':
            # Create a list of observation spaces (aka views)
            self.Ylist = []
            self.namesList = []
            for k in self.observed.keys():
                self.Ylist = [self.Ylist, self.observed[k]]
                self.namesList = [self.namesList, k]
            self.Ylist[0]=self.Ylist[0][1]
            self.namesList[0]=self.namesList[0][1]
            pcaFailed=False
            if init_X == 'PCA':
                try:
                    self.model = GPy.models.MRD(self.Ylist, input_dim=self.Q, num_inducing=self.num_inducing, kernel=kernel, initx="PCA_single", initz='permute')
                except ValueError:
                    pcaFailed = True
                    print "Initialisation with PCA failed. Initialising with PPCA..."
            elif init_X == 'PPCA' or pcaFailed:
                print "Initialising with PPCA..."
                from GPy.util.initialization import initialize_latent
                Xr = np.zeros((self.Ylist[0].shape[0], self.Q))
                for qs, Y in zip(np.array_split(np.arange(self.Q), len(self.Ylist)), self.Ylist):
                    try:
                        x,frcs = initialize_latent('PCA', len(qs), Y)
                    except ValueError:
                        x = GPy.util.linalg.ppca(Y, len(qs), 2000)[0]
                    Xr[:, qs] = x
                Xr -= Xr.mean()
                Xr /= Xr.std()
                self.model = GPy.models.MRD(self.Ylist, input_dim=self.Q, num_inducing=self.num_inducing, kernel=kernel, initx="PCA_single", initz='permute', X=Xr)
            self.model['.*noise']=[yy.var() / 100. for yy in self.model.Ylist]
        elif self.type == 'gp':
            self.model = GPy.models.SparseGPRegression(self.inputs, self.observed[self.observed.keys()[0]], kernel=kernel, num_inducing=self.num_inducing)
        
        self.model.data_labels = None

    #def _init_latents():
    #    from GPy.util.initialization import initialize_latent
    #    X, fracs = initialize_latent(init, input_dim, Y)

    def add_labels(self, labels):
        """
        If observables are associated with labels, they can be added here.
        labels has to be a matrix of size N x K, where K is the total number
        of different labels. If e.g. the i-th row of L is [1 0 0] (or [1 -1 -1])
        then this means that there are K=3 different classes and the i-th row
        of the observables belongs to the first class.
        """
        if len(labels.shape) == 1 or labels.shape[1] == 1:
            self.model.data_labels = labels
        else:
            print "Warning: labels assumed to be in 1-of-K encoding!"
            self.model.data_labels = np.argmax(labels,1)[:,None]

    def learn(self, optimizer='bfgs',max_iters=1000, init_iters=300, verbose=True):
        """
        Learn the model (analogous to "forming synapses" after perveiving data).
        """
        if self.type == 'bgplvm' or self.type == 'mrd':
            self.model['.*noise'].fix()
            self.model.optimize(optimizer, messages=verbose, max_iters=init_iters)
            self.model['.*noise'].unfix()
            self.model['.*noise'].constrain_positive()
        
        self.model.optimize(optimizer, messages=verbose, max_iters=max_iters)

    def visualise(self):
        """
        Show the internal representation of the memory
        """
        if self.type == 'bgplvm' or self.type == 'mrd':
            if self.model.data_labels is not None:
                ret = self.model.plot_latent(labels=self.model.data_labels)
            else:
                ret = self.model.plot_latent()
        elif self.type == 'gp':
            ret = self.model.plot()
        if self.type == 'mrd':
            ret2 = self.model.plot_scales()

        #if self.type == 'mrd':
        #    ret1 = self.model.X.plot("Latent Space 1D")
        #    ret2 = self.model.plot_scales("MRD Scales")
        
        return ret

    def visualise_interactive(self, dimensions=(20,28), transpose=True, order='F', invert=False, scale=False, colorgray=True, view=0, which_indices=(0, 1)):
        """
        Show the internal representation of the memory and allow the user to
        interact with it to map samples/points from the compressed space to the
        original output space
        """
        if self.type == 'bgplvm':
            ax = self.model.plot_latent(which_indices)
            y = self.model.Y[0, :]
            # dirty code here
            if colorgray:
                data_show = GPy.plotting.matplot_dep.visualize.image_show(y[None, :], dimensions=dimensions, transpose=transpose, order=order, invert=invert, scale=scale, cmap = cm.Greys_r)
            else:
                data_show = GPy.plotting.matplot_dep.visualize.image_show(y[None, :], dimensions=dimensions, transpose=transpose, order=order, invert=invert, scale=scale)
            lvm = GPy.plotting.matplot_dep.visualize.lvm(self.model.X.mean[0, :].copy(), self.model, data_show, ax)
            raw_input('Press enter to finish')
        elif self.type == 'mrd':
            """
            NOT TESTED!!!
            """
            ax = self.model.bgplvms[view].plot_latent(which_indices)
            y = self.model.bgplvms[view].Y[0, :]
            # dirty code here
            if colorgray:
                data_show = GPy.plotting.matplot_dep.visualize.image_show(y[None, :], dimensions=dimensions, transpose=transpose, order=order, invert=invert, scale=scale, cmap = cm.Greys_r)
            else:
                data_show = GPy.plotting.matplot_dep.visualize.image_show(y[None, :], dimensions=dimensions, transpose=transpose, order=order, invert=invert, scale=scale)
            lvm = GPy.plotting.matplot_dep.visualize.lvm(self.model.bgplvms[view].X.mean[0, :].copy(), self.model.bgplvms[view], data_show, ax)
            raw_input('Press enter to finish')

    def recall(self, locations):
        """
        Recall stored events. This is closely related to performing pattern pattern_completion
        given "training" data.
        Input is the index of the stored event (TODO: make it more generic)
        """
        if locations == -1:
            locations = range(self.N)
        if self.type == 'bgplvm' or self.type == 'gp':
            return self.model.Y[locations,:].values
        elif self.type == 'mrd':
            return self.model.bgplvms[0].Y[locations,:].values

    def pattern_completion(self, test_data, view=0, verbose=False, visualiseInfo=None):
        """
        In the case of supervised learning, pattern completion means that we 
        give new inputs and infer their correspondin outputs. In the case of
        unsupervised leaerning, pattern completion means that we give new
        outputs and we infer their corresponding "latent" inputs, ie the internal
        compressed representation of the new outputs in terms of the already
        formed "synapses".
        """
        if self.type == 'bgplvm':
            #tmp = self.model.infer_newX(test_data)[0]
            #pred_mean = tmp.mean
            #pred_variance = tmp.variance #np.zeros(pred_mean.shape)
            tmp=self.model.infer_newX(test_data,optimize=False)[1]
            tmp.optimize(max_iters=2000, messages=verbose)
            pred_mean = tmp.X.mean
            pred_variance = tmp.X.variance
        elif self.type == 'mrd':
            tmp = self.model.bgplvms[view].infer_newX(test_data, optimize=False)[1]
            tmp.optimize(max_iters=2000, messages=verbose)
            pred_mean = tmp.X.mean
            pred_variance = tmp.X.variance
        elif self.type == 'gp':
            pred_mean, pred_variance = self.model.predict(test_data)
        if (self.type == 'mrd' or self.type == 'bgplvm') and visualiseInfo is not None:
            ax = visualiseInfo['ax']
            inds0, inds1=dredplots.most_significant_input_dimensions(self.model, None)
            pp=ax.plot(pred_mean[:,inds0], pred_mean[:,inds1], 'om', markersize=11, mew=11)
            pb.draw()
        else:
            pp=None
	    
        return pred_mean, pred_variance, pp, tmp

    def _get_inducing(self):
        # TODO
        pass

    def _get_latent(self):
        # TODO
        pass



"""
Helper functions for saving and loading the SAMObject.
For the moment, these are quite naive functions which dump the whole object as a
serialized sequence of bytes, into a txt file.
For the future, we can make them "smarter" by not having to save information e.g. about
the data or about the object functions, but just save the parameters of the trained model.
"""
def save_model(mm, fileName='m_serialized.txt'):
    #mPruned = mm.getstate() # TODO (store less stuff)
    output = open(fileName, 'wb')
    #pickle.dump(mPruned, output)
    pickle.dump(mm, output)
    output.close()

def load_model(fileName='m_serialized.txt'):
    mm = pickle.load(open(fileName,'r'))
    return mm

def save_pruned_model(mm, fileName='m_pruned'):
    SAMObjPruned=dict()
    SAMObjPruned['type'] = mm.type
    # SAMObjPruned['observed'] = mm.observed # REMOVE
    # SAMObjPruned['inputs'] = mm.inputs
#    SAMObjPruned['__num_views'] = mm.__num_views
    SAMObjPruned['__num_views'] = None
    SAMObjPruned['Q'] = mm.Q
    SAMObjPruned['N'] = mm.N
    SAMObjPruned['num_inducing'] = mm.num_inducing
    SAMObjPruned['namesList'] = mm.namesList
    SAMObjPruned['modelPath'] = fileName + '_model.pickle'
    mm.model.pickle(SAMObjPruned['modelPath'])
    output = open(fileName+'.pickle', 'wb')
    pickle.dump(SAMObjPruned, output)
    output.close()

def load_pruned_model(fileName='m_pruned'):
    SAMObjPruned = pickle.load(open(fileName + '.pickle','rb'))
    SAMObject=LFM()
    with open(SAMObjPruned['modelPath'], 'rb') as f:
        print "Loading file: " + str(f)
        SAMObject.model = pickle.load(f)
    # TODO: The following is supposed to update the model, but maybe not. Change...
    # LB get error here using MRD
    #SAMObject.model.update_toggle()
    #SAMObject.model.update_toggle()
    
    SAMObject.type = SAMObjPruned['type'] 
    # SAMObject.observed = SAMObjPruned['observed'] 
    # SAMObject.inputs = SAMObjPruned['inputs'] 
    SAMObject.__num_views = SAMObjPruned['__num_views'] 
    SAMObject.Q = SAMObjPruned['Q'] 
    SAMObject.N = SAMObjPruned['N'] 
    SAMObject.num_inducing = SAMObjPruned['num_inducing'] 
    SAMObject.namesList = SAMObjPruned['namesList'] 

    return SAMObject


