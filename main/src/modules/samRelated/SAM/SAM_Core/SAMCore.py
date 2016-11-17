# """"""""""""""""""""""""""""""""""""""""""""""
# The University of Sheffield
# WYSIWYD Project
#
# The core of the Synthetic Autobiographical Memory (SAM) system.
# The core is built upon Latent Feature Models (LFMs) and implements
# cognitive memory properties, such as recall, pattern completion, compression etc.
#
# The Core is accompanied by peripherals, currently implemented as Drivers.
# Drivers facilitate the communication of sensory modalities and the Core.
# See Driver.py for this.
#
# Created:  2015
#
# @authors: Andreas Damianou
#
# """"""""""""""""""""""""""""""""""""""""""""""


import GPy
import numpy as np
import matplotlib.cm as cm
import itertools
import pylab as pb
import cPickle as pickle
from scipy.spatial import distance
import operator
import os


"""
SAM based on Latent Feature Models
"""


class LFM(object):
    def __init__(self):
        self.type = []
        self.model = []
        self.observed = None
        self.inputs = None
        self.__num_views = None
        self.Q = None
        self.N = None
        self.num_inducing = None
        self.namesList = None
        self.Ylist = None

    def store(self, observed, inputs=None, Q=None, kernel=None, num_inducing=None, init_X='PCA'):
        """
        Store events.
        ARG: observed: A N x D matrix, where N is the number of points and D the number
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
        assert(isinstance(observed, dict))
        self.observed = observed
        self.__num_views = len(self.observed.keys())
        self.Q = Q
        # self.D = observed.shape[1]
        self.N = observed[observed.keys()[0]].shape[0]
        self.num_inducing = num_inducing
        if num_inducing is None:
            self.num_inducing = self.N
        if inputs is None:
            if self.Q is None:
                self.Q = 2  # self.D
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
            self.model['.*noise'] = Ytmp.var() / 100.
        elif self.type == 'mrd':
            # Create a list of observation spaces (aka views)
            self.Ylist = []
            self.namesList = []
            for k in self.observed.keys():
                self.Ylist = [self.Ylist, self.observed[k]]
                self.namesList = [self.namesList, k]
            self.Ylist[0] = self.Ylist[0][1]
            self.namesList[0] = self.namesList[0][1]
            pcaFailed = False
            if init_X == 'PCA':
                try:
                    self.model = GPy.models.MRD(self.Ylist, input_dim=self.Q, num_inducing=self.num_inducing,
                                                kernel=kernel, initx="PCA_single", initz='permute')
                except ValueError:
                    pcaFailed = True
                    print "Initialisation with PCA failed. Initialising with PPCA..."
            elif init_X == 'PPCA' or pcaFailed:
                print "Initialising with PPCA..."
                from GPy.util.initialization import initialize_latent
                Xr = np.zeros((self.Ylist[0].shape[0], self.Q))
                for qs, Y in zip(np.array_split(np.arange(self.Q), len(self.Ylist)), self.Ylist):
                    try:
                        x, frcs = initialize_latent('PCA', len(qs), Y)
                    except ValueError:
                        x = GPy.util.linalg.ppca(Y, len(qs), 2000)[0]
                    Xr[:, qs] = x
                Xr -= Xr.mean()
                Xr /= Xr.std()
                self.model = GPy.models.MRD(self.Ylist, input_dim=self.Q, num_inducing=self.num_inducing, kernel=kernel, initx="PCA_single", initz='permute', X=Xr)
            self.model['.*noise'] = [yy.var() / 100. for yy in self.model.Ylist]
        elif self.type == 'gp':
            self.model = GPy.models.SparseGPRegression(self.inputs, self.observed[self.observed.keys()[0]], kernel=kernel, num_inducing=self.num_inducing)
        
        self.model.data_labels = None
        self.model.textLabelPts = dict()

    # def _init_latents():
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
            self.model.data_labels = np.argmax(labels, 1)[:, None]

    def learn(self, optimizer='bfgs', max_iters=1000, init_iters=300, verbose=True):
        """
        Learn the model (analogous to "forming synapses" after perveiving data).
        """
        if self.type == 'bgplvm' or self.type == 'mrd':
            self.model['.*noise'].fix()
            self.model.optimize(optimizer, messages=verbose, max_iters=init_iters)
            self.model['.*noise'].unfix()
            self.model['.*noise'].constrain_positive()
        
        self.model.optimize(optimizer, messages=verbose, max_iters=max_iters)
        self.check_snr()

        for j in list(np.unique(self.model.data_labels)):
            self.model.textLabelPts[j] = [l for l, k in enumerate(self.model.data_labels) if k[0] == j]

    def check_snr(self, warning=True, messages=True):
        if self.type == 'bgplvm':
            snr = self.model.Y.var()/self.model.Gaussian_noise.variance.values[0]

            if messages:
                print('# SNR: ' + str(snr))
            if warning and snr < 8:
                print(' WARNING! SNR is small!')
        elif self.type == 'mrd':
            snr = []
            for i in range(len(self.model.bgplvms)):
                snr.append(self.model.bgplvms[i].Y.var()/self.model.bgplvms[i].Gaussian_noise.variance.values[0])
                if messages:
                    print('# SNR view ' + str(i) + ': ' + str(snr[-1]))
                if warning and snr[-1] < 8:
                    print(' WARNING! SNR for view ' + str(i) + ' is small!!')
        else:
            snr = None
        return snr
        
    def visualise(self, which_indices=None, plot_scales=True):
        """
        Show the internal representation of the memory
        """
        # if self.type == 'bgplvm' and which_indices is None:
        #    which_indices = most_significant_input_dimensions(self.model,None)
        # if self.type == 'mrd' and which_indices is None:
        #    # Assume that labels modality is always the last one!!
        #    which_indices = most_significant_input_dimensions(self.model.bgplvms[-1],None)
        if self.type == 'bgplvm' or self.type == 'mrd':
            if self.model.data_labels is not None:
                ret = self.model.plot_latent(labels=self.model.data_labels, which_indices=which_indices)
            else:
                ret = self.model.plot_latent(which_indices=which_indices)
        elif self.type == 'gp':
            ret = self.model.plot()
        if self.type == 'mrd' and plot_scales:
            ret2 = self.model.plot_scales()

        # if self.type == 'mrd':
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
            return self.model.Y[locations, :].values
        elif self.type == 'mrd':
            return self.model.bgplvms[0].Y[locations, :].values

    def pattern_completion(self, test_data, view=0, verbose=False, visualiseInfo=None, optimise=100):
        """
        In the case of supervised learning, pattern completion means that we 
        give new inputs and infer their corresponding outputs. In the case of
        unsupervised learning, pattern completion means that we give new
        outputs and we infer their corresponding "latent" inputs, ie the internal
        compressed representation of the new outputs in terms of the already
        formed "synapses".
        """
        if self.type == 'bgplvm':
            # tmp = self.model.infer_newX(test_data)[0]
            # pred_mean = tmp.mean
            # pred_variance = tmp.variance #np.zeros(pred_mean.shape)
            tmp = self.model.infer_newX(test_data, optimize=False)[1]
            if optimise != 0:
                tmp.optimize(max_iters=optimise, messages=verbose)
            pred_mean = tmp.X.mean
            pred_variance = tmp.X.variance
        elif self.type == 'mrd':
            tmp = self.model.bgplvms[view].infer_newX(test_data, optimize=False)[1]
            if optimise != 0:
                tmp.optimize(max_iters=optimise, messages=verbose)
            pred_mean = tmp.X.mean
            pred_variance = tmp.X.variance
        elif self.type == 'gp':
            tmp = []
            pred_mean, pred_variance = self.model.predict(test_data)

        if (self.type == 'mrd' or self.type == 'bgplvm') and visualiseInfo is not None:
            ax = visualiseInfo['ax']
            inds0, inds1 = most_significant_input_dimensions(self.model, None)
            pp = ax.plot(pred_mean[:, inds0], pred_mean[:, inds1], 'om', markersize=11, mew=11)
            pb.draw()
        else:
            pp = None
        
        return pred_mean, pred_variance, pp, tmp

    def pattern_completion_inference(self, y, target_modality=-1):
        """
        This is a wrapper around pattern completion where:
        1) First, we do normal pattern completion, where given an output y, out map to the memory space to get a test memory x*.
        2) Now the test memory x* is compared with stored memories. This allows us to infer the label of x*. If the labels
           are given in another modality (by default in the last one), then we return the label from that modality (careful, 
            the encoding might be 1-of-K, e.g. -1 1 -1 -> 2 and also noise might exist).
           Instead, if the labels are not given in another modality (completely unsupervised learning), then we just return
           the index to the most similar training memory.
        """
        # Returns the predictive mean, the predictive variance and the axis (pp) of the latent space backwards mapping.            
        ret = self.pattern_completion(y)
        mm = ret[0]
        post = ret[3]        
        # find nearest neighbour of mm and model.X
        dists = np.zeros((self.model.X.shape[0], 1))

        for j in range(dists.shape[0]):
            dists[j, :] = distance.euclidean(self.model.X.mean[j, :], mm[0].values)
        nn, min_value = min(enumerate(dists), key=operator.itemgetter(1))
        if self.type == 'mrd':
            ret = self.model.bgplvms[target_modality].Y[nn, :]
        elif self.type == 'bgplvm':
            ret = nn  # self.model.data_labels[nn]
        return ret

    def fantasy_memory(self, X, view=0):
        """
        The opposite of pattern completion. Instead of finding a memory from an output, here we find an output from a
        (possibly fantasy) memory. Here, fantasy memory is a memory not existing in the training set, found by interpolating
        or sampling in the memory space.
        """
        if self.type == 'mrd':
            pred_mean, pred_variance = self.model.bgplvms[view].predict(X)
        elif self.type == 'bgplvm':
            pred_mean, pred_variance = self.model.predict(X)
        elif self.type == 'gp':
            pred_mean, pred_variance = self.model.predict(X)
        return pred_mean, pred_variance

    def familiarity(self, Ytest, ytrmean=None, ytrstd=None, optimise=100):
        assert(self.type == 'bgplvm')

        N = Ytest.shape[0]
        if ytrmean is not None:
            Ytest -= ytrmean
            Ytest /= ytrstd

        from SAM.SAM_Core.svi_ratio import SVI_Ratio
        s = SVI_Ratio()
        _, _, _, qX = self.pattern_completion(Ytest, verbose=False, optimise=optimise)
        qX = qX.X

        ll = 0
        for i in range(N):
            ll += s.inference(self.model.kern, qX[i, :][None, :], self.model.Z, self.model.likelihood,
                              Ytest[i, :][None, :], self.model.posterior)[0]
        ll /= N
        return ll
        
##############################  TMP   ##############################################################
    def familiarity_reverse(self, Ytest, ytrmean=None, ytrstd=None, source_view=0, max_iters=1000, num_inducing=15):
        if Ytest is None:
            if self.type == 'bgplvm':
                tmpX = self.model.Y.values.copy()
            elif self.type == 'mrd':
                tmpX = self.model.bgplvms[source_view].Y.values.copy()
            kernel = GPy.kern.RBF(tmpX.shape[1], ARD=False)+GPy.kern.Bias(tmpX.shape[1])
            tmpY = self.model.X.mean.values.copy()
            self.back_GP = GPy.models.SparseGPRegression(tmpX, tmpY, kernel=kernel, num_inducing=num_inducing)
            self.back_GP.optimize(optimizer='bfgs', max_iters=max_iters, messages=1)
            return (None,None)
        else:
            if ytrmean is not None:
                Ytest -= ytrmean
                Ytest /= ytrstd
            return self.back_GP.predict(Ytest)

    def familiarity3(self, Ytest, ytrmean=None, ytrstd=None):
        assert(self.type == 'bgplvm')

        import numpy as np
        N = Ytest.shape[0]
        if ytrmean is not None:
            Ytest -= ytrmean
            Ytest /= ytrstd

        qx, mm = self.model.infer_newX(Ytest)
        # Optional for more iters---
        mm.optimize(max_iters=800)
        # qx = mm.X

        return mm._log_marginal_likelihood - self.model._log_marginal_likelihood

    def familiarity2(self, Ytest, ytrmean = None, ytrstd=None, sigma2=None, use_uncert=True):
        #def my_logpdf(y, ymean, yvar):
        #    import numpy as np
        #    N = y.shape[0]
        #    ln_det_cov = N * np.log(yvar)
        #    return -0.5 * (np.sum((y - ymean) ** 2 / yvar) + ln_det_cov + N * np.log(2. * np.pi))
        #from scipy.stats import multivariate_normal
        #var = multivariate_normal(mean=[0,0], cov=[[1,0],[0,1]])
        #var.pdf([1,0])
        from scipy.stats import lognorm

        assert(self.type == 'bgplvm')

        import numpy as np
        N = Ytest.shape[0]
        if ytrmean is not None:
            Ytest -= ytrmean
            Ytest /= ytrstd

        qx, mm = self.model.infer_newX(Ytest)
        # Optional for more iters---
        mm.optimize(max_iters=400)
        qx = mm.X
        #----
        
        #ymean, yvar = model._raw_predict(qx)
        # This causes the code to hang!!! Replace qx with qx.mean.values...!!!!
        if use_uncert:
            ymean, yvar = self.model.predict(qx)
        else:
            ymean,yvar = self.model.predict(qx.mean.values)
        ll = np.zeros(N)
        for j in range(N):
            #ll[j] = my_logpdf(Ytest[j], ymean[j], yvar[j])
            #ll[j] = multivariate_normal(mean=ymean[j], cov=np.diag(yvar[j])).pdf(Ytest[j])
            ll[j] = lognorm.pdf(Ytest[j], s=1, loc=ymean[j], scale=yvar[j]).mean()
        loglike = ll.mean()

        return loglike
##############################  TMP   ##############################################################





    def _get_inducing(self):
        # TODO
        pass

    def _get_latent(self):
        if self.type == 'bgplvm':
            return self.model.X.mean
        elif self.type == 'mrd':
            return self.model.bgplvms[0].X.mean
        else:
            print('No latent space for this type of model.')
            return None



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


def save_pruned_model(mm, fileName='m_pruned', economy=False, extraDict=dict()):
    """
    Save a trained model after prunning things that are not needed to be stored.
    Economy set to True will trigger a (currently BETA) storing which creates much smaller files.
    See the load_pruned_model discussion on what this means in terms of restrictions.
    """
    SAMObjPruned=dict()
    SAMObjPruned['type'] = mm.type
    if mm.model:
        SAMObjPruned['textLabelPts'] = mm.model.textLabelPts
    # SAMObjPruned['observed'] = mm.observed # REMOVE
    # SAMObjPruned['inputs'] = mm.inputs
#    SAMObjPruned['__num_views'] = mm.__num_views
    SAMObjPruned['__num_views'] = None
    SAMObjPruned['Q'] = mm.Q
    SAMObjPruned['N'] = mm.N
    SAMObjPruned['num_inducing'] = mm.num_inducing
    SAMObjPruned['namesList'] = mm.namesList
    SAMObjPruned['kernelString'] = mm.kernelString
    SAMObjPruned.update(extraDict)

    # if economy:
    #     SAMObjPruned['modelPath'] = fileName + '_model.h5'
    #     #if file exists delete
    #     if(os.path.isfile(SAMObjPruned['modelPath'])):
    #         os.remove(SAMObjPruned['modelPath'])
    #     mm.model.save(SAMObjPruned['modelPath'])
    # else:
    #     SAMObjPruned['modelPath'] = fileName + '_model.pickle'
    #     mm.model.pickle(SAMObjPruned['modelPath'])
    folderPath = os.path.join('/', *fileName.split('/')[:-1])
    fileName = fileName.split('/')[-1]

    if economy:
        SAMObjPruned['modelPath'] = fileName + '_model.h5'
        # if file exists delete
        if os.path.isfile(os.path.join(folderPath, SAMObjPruned['modelPath'])):
            os.remove(os.path.join(folderPath, SAMObjPruned['modelPath']))
        if mm.model:
            mm.model.save(os.path.join(folderPath, SAMObjPruned['modelPath']))
    else:
        SAMObjPruned['modelPath'] = fileName + '_model.pickle'
        mm.model.pickle(os.path.join(folderPath, SAMObjPruned['modelPath']))

    output = open(os.path.join(folderPath, fileName) + '.pickle', 'wb')
    pickle.dump(SAMObjPruned, output)
    output.close()


def load_pruned_model(fileName='m_pruned', economy=False, m=None):
    """
    Load a trained model. If economy is set to True, then a not-None initial model m is needed.
    This model needs to be created exactly as the one that was saved (so, it is demo specific!) and
    in this case calling the present function will set its parameters (meaning that you still need to
    create a model but don't need to optimize it.)
    """
    folderPath = os.path.join('/', *fileName.split('/')[:-1])
    SAMObjPruned = pickle.load(open(fileName + '.pickle', 'rb'))
    SAMObject = LFM()
    if economy:
        assert m is not None
        import tables
        f = tables.open_file(os.path.join(folderPath,SAMObjPruned['modelPath']), 'r')
        m.param_array[:] = f.root.param_array[:]
        f.close()
        m._trigger_params_changed()
        SAMObject.model = m
    else:
        with open(SAMObjPruned['modelPath'], 'rb') as f:
            print "Loading file: " + str(f)
            SAMObject.model = pickle.load(f)
        # TODO: The following is supposed to update the model, but maybe not. Change...
    # LB get error here using MRD
    # SAMObject.model.update_toggle()
    # SAMObject.model.update_toggle()
    
    SAMObject.type = SAMObjPruned['type'] 
    # SAMObject.observed = SAMObjPruned['observed'] 
    # SAMObject.inputs = SAMObjPruned['inputs']
    SAMObject.model.textLabelPts = SAMObjPruned['textLabelPts']
    SAMObject.__num_views = SAMObjPruned['__num_views'] 
    SAMObject.Q = SAMObjPruned['Q'] 
    SAMObject.N = SAMObjPruned['N'] 
    SAMObject.num_inducing = SAMObjPruned['num_inducing'] 
    SAMObject.namesList = SAMObjPruned['namesList']

    return SAMObject


# Copied from GPy
def most_significant_input_dimensions(model, which_indices):
    """
    Determine which dimensions should be plotted based on the relevance weights.
    """
    if which_indices is None:
        if model.input_dim == 1:
            input_1 = 0
            input_2 = None
        if model.input_dim == 2:
            input_1, input_2 = 0, 1
        else:
            try:
                input_1, input_2 = np.argsort(model.input_sensitivity())[::-1][:2]
            except:
                raise ValueError("cannot automatically determine which dimensions to plot, please pass 'which_indices'")
    else:
        input_1, input_2 = which_indices
    return input_1, input_2


def latent_cluster_estimate(SAMObject, n_components=10, X=None, plot=True, alpha=10, covariance_type='diag',which_indices=(0,1)):
    """
    Use Dirichlet Process GMMs to cluster the latent space by automatically estimating an effective number of clusters.
    ARG SAMObject: The SAMObject to operate on.
    ARG n_components: The number of DPGMM commponents to use (ie max number of clusters). Some components will switch off.
    ARG X: If None, we'll use the SAMObject's latent space, otherwise the provided one.
    ARG plot: Whether to plot the result or not.
    ARG alpha: The parameter for the stick-breaking process. In theory, large alpha encourages more clusters, although in practice I haven't seen such behaviour.
    ARG covariance_type: See DPGMM from scikit-learn.
    ARG which_indices: If plotting, which indices to plot.
    RETURN Y_: The cluster assignments for each component in the latent space. This is not (0,1,...,n_clusters), but instead it is
               (0,1,...,n_components), so that switched off components will not appear in Y_.
    """
    from sklearn import mixture

    if X is None:
        X = SAMObject._get_latent()
    # Fit a Dirichlet process mixture of Gaussians using five components
    dpgmm = mixture.DPGMM(n_components=n_components, covariance_type=covariance_type, n_iter=5000,alpha=alpha)
    dpgmm.fit(X)
    Y_ = dpgmm.predict(X)

    if plot:
        from scipy import linalg
        import matplotlib as mpl
        import itertools

        color_iter = colors = cm.rainbow(np.linspace(0, 1, 20))
        myperm = np.random.permutation(color_iter.shape[0])
        color_iter = color_iter[myperm, :]
        marker_iter = itertools.cycle((',', '+', '.', 'o', '*','v','x','>')) 
        splot = pb.subplot(1, 1, 1)

        for i, (mean, covar, color,marker) in enumerate(zip(dpgmm.means_, dpgmm._get_covars(), color_iter,marker_iter)):
            # as the method will not use every component it has access to
            # unless it needs it, we shouldn't plot the redundant components.
            # if not np.any(Y_ == i):
            #    continue
            pb.scatter(X[Y_ == i, which_indices[0]], X[Y_ == i, which_indices[1]], s=40, color=color,marker=marker)

        pb.legend(np.unique(Y_))
        pb.show()
        pb.draw()
        pb.show()
    return Y_


def latent_cluster(SAMObject, n_clusters=10, X=None, plot=True, which_indices=(0,1)):
    """
    Use Anglomerative clustering to cluster the latent space by having a given number of clusters.
    ARG SAMObject: The SAMObject to operate on.
    ARG n_clusters: The number of clusters to find.
    ARG X: If None, we'll use the SAMObject's latent space, otherwise the provided one.
    ARG plot: Whether to plot the result or not.
    ARG which_indices: If plotting, which indices to plot.
    RETURN Y_: The cluster assignments for each component in the latent space. 
    """
    from sklearn.cluster import AgglomerativeClustering

    if X is None:
        X = SAMObject._get_latent()

    # Define the structure A of the data. Here a 10 nearest neighbors
    from sklearn.neighbors import kneighbors_graph
    connectivity = kneighbors_graph(X, n_neighbors=10, include_self=False)

    # Compute clustering
    print("Compute structured hierarchical clustering...")
    ward = AgglomerativeClustering(n_clusters=n_clusters, connectivity=connectivity, linkage='ward',
                                   compute_full_tree=True).fit(X)
    # ward = AgglomerativeClustering(n_clusters=8,linkage='ward',compute_full_tree=True).fit(X)
    Y_ = ward.labels_

    if plot:
        color_iter = colors = cm.rainbow(np.linspace(0, 1, 20))

        # ---- a silly way to get maximal separation in colors for the n_cluster first elements...
        # move to separate function
        index_all = np.linspace(0, 19, 20).astype(int)
        space = np.floor(color_iter.shape[0]/float(n_clusters)).astype(int)
        index_first = index_all[::space][:n_clusters]
        index_rest = np.array(list(set(index_all)-set(index_first)))
        myperm = np.random.permutation(index_rest.shape[0])
        index_rest = index_rest[myperm]
        inds = np.hstack((index_first, index_rest))

        color_iter = color_iter[inds,:]

        marker_iter = itertools.cycle((',', '+', '.', 'o', '*','v','x','>')) 
        splot = pb.subplot(1, 1, 1)

        for i, (color, marker) in enumerate(zip(color_iter, marker_iter)):
            # as the method will not use every component it has access to unless it needs it,
            # we shouldn't plot the redundant components.
            # if not np.any(Y_ == i):
            #    continue
            # ##### tmp
            # cc = ['b','g','r']
            # mm = ['<','^','>']
            # pb.scatter(X[Y_ == i, which_indices[0]], X[Y_ == i, which_indices[1]], s=40, color=cc[i],marker=mm[i])
            # ######
            pb.scatter(X[Y_ == i, which_indices[0]], X[Y_ == i, which_indices[1]], s=40, color=color,marker=marker) #UNCOMMENT
            if i >= n_clusters:
                break

        pb.legend(np.unique(Y_))
        pb.show()
        pb.draw()
        pb.show()
    return Y_


def util_plot_cov_ellipse(pos, cov, volume=.5, ax=None, fc='none', ec=[0,0,0], a=1, lw=2, which_indices=(0,1)):
    """
    SEE: http://www.nhsilbert.net/source/2014/06/bivariate-normal-ellipse-plotting-in-python/
    #
    Plots an ellipse enclosing *volume* based on the specified covariance
    matrix (*cov*) and location (*pos*). Additional keyword arguments are passed on to the 
    ellipse patch artist.

    Parameters
    ----------
        cov : The 2x2 covariance matrix to base the ellipse on
        pos : The location of the center of the ellipse. Expects a 2-element
            sequence of [x0, y0].
        volume : The volume inside the ellipse; defaults to 0.5
        ax : The axis that the ellipse will be plotted on. Defaults to the 
            current axis.
    """

    import numpy as np
    from scipy.stats import chi2
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse

    if ax is None:
        ax = plt.gca()

    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    vals = vals[order]
    vecs = vecs[:, order]
    angle = np.degrees(np.arctan2(*vecs[:, 0][::-1]))

    kwrg = {'facecolor': fc, 'edgecolor': ec, 'alpha': a, 'linewidth': lw}

    # Width and height are "full" widths, not radius
    width, height = 2 * np.sqrt(chi2.ppf(volume,2)) * np.sqrt(vals)
    posOrder = [which_indices[0], which_indices[1]]
    ellip = Ellipse(xy=pos[posOrder], width=width, height=height, angle=angle, **kwrg)
    ax.add_artist(ellip)

    plt.draw()
    plt.show()
    plt.draw()
    plt.plot(pos[which_indices[0]], pos[which_indices[1]], 'k+', markersize=15, mew=2)


#def util_plot_cov_ellipse(mean,covar,ax=None):
#     from matplotlib.patches import Ellipse
#     import matplotlib as mpl
#     import matplotlib.pyplot as plt

#     import numpy as np

#     v, w = np.linalg.eigh(covar)
#     #order = v.argsort()[::-1]
#     #v=v[order]
#     #w=w[:,order]
#     u = w[0] / np.linalg.norm(w[0])

#     if ax is None:
#         ax = plt.gca()

#     # Plot an ellipse to show the Gaussian component
#     angle = np.arctan(u[1] / u[0])
#     angle = 180 * angle / np.pi  # convert to degrees
#     ell = mpl.patches.Ellipse(mean, v[0], v[1], 180 + angle, color=color)
#     ell.set_clip_box(splot.bbox)
#     ell.set_alpha(0.5)
#     ax.add_artist(ell)
#     plt.draw()
#     plt.show()
#     plt.draw()

#     ---
#     theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

#     kwrg = {'facecolor':fc, 'edgecolor':ec, 'alpha':a, 'linewidth':lw}

#     # Width and height are "full" widths, not radius
#     width, height = 2 * np.sqrt(chi2.ppf(volume,2)) * np.sqrt(vals)
#     ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwrg)


def latent_cluster_centers(SAMObject, X=None, labels=None, center='gaussian', plot=True, which_indices=(0,1), randSeed=None, ax=None):
    """
    Find centers for the clusters identified in param. labels for the latent space. Centers can be a gaussian density (so, mean and covar.)
    or (not implemented yet) mean and median, as controlled by the param. center.
    """
    from sklearn import mixture

    assert(labels is not None)

    if X is None:
        X = SAMObject._get_latent()

    cluster_labels = np.unique(labels)
    K = len(cluster_labels)
    Q = X.shape[1]

    cntr = np.zeros((K, Q))*np.nan
    if center == 'gaussian':
        covars = np.zeros((K, Q, Q))*np.nan
    else:
        covars = None

    for i in range(K):
        if center == 'gaussian':
            g = mixture.GMM(covariance_type='full', init_params='wmc', min_covar=0.001,
                            n_components=1, n_init=1, n_iter=300, params='wmc',
                            random_state=randSeed, thresh=None, tol=0.001, verbose=0)
            g.fit(X[labels == cluster_labels[i], :])
            cntr[i, :] = g.means_
            covars[i] = g.covars_
        elif center == 'median':
            raise NotImplementedError("This is not implemented yet")
        elif center == 'mean':
            raise NotImplementedError("This is not implemented yet")
        else:
            print('Not known center type')
            raise

    if plot:
        color_iter = colors = cm.rainbow(np.linspace(0, 1, 20))
        myperm = np.random.permutation(color_iter.shape[0])
        color_iter = color_iter[myperm, :]
        marker_iter = itertools.cycle((',', '+', '.', 'o', '*', 'v', 'x', '>'))
        splot = pb.subplot(1, 1, 1)

        for i, (color, marker) in enumerate(zip(color_iter,marker_iter)):
            pb.scatter(X[labels == cluster_labels[i], which_indices[0]],
                       X[labels == cluster_labels[i], which_indices[1]], s=40, color=color, marker=marker)

            if i == K-1:
                break
        if ax is None:
            ax = pb.gca()
        for i in range(K):
            util_plot_cov_ellipse(cntr[i, :], covars[i], ax=ax, which_indices=which_indices)
    return cntr, covars
