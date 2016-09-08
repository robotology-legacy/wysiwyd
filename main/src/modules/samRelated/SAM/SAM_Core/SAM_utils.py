# """"""""""""""""""""""""""""""""""""""""""""""
# The University of Sheffield
# WYSIWYD Project
#
# SAMpy class for various methods related to processing data
#
# Created on 26 May 2015
#
# @authors: Andreas Damianou, Daniel Camilleri
#
# """"""""""""""""""""""""""""""""""""""""""""""
import numpy as np
from ConfigParser import SafeConfigParser
import pickle
from SAM.SAM_Core import samOptimiser
from os import listdir
from os.path import join, isdir
import threading


def initialiseModels(argv, update, initMode='training'):
    # argv[1] = dataPath
    # argv[2] = modelPath
    # argv[3] = driverName
    # update = 'update' or 'new'

    from SAM.SAM_Core import SAMDriver as Driver
    dataPath = argv[0]
    modelPath = argv[1]
    driverName = argv[2]

    print argv
    stringCommand = 'from SAM.SAM_Drivers import ' + driverName + ' as Driver'
    print stringCommand
    exec stringCommand

    mySAMpy = Driver()
    mode = update
    trainName = dataPath.split('/')[-1]

    # participantList is extracted from number of subdirectories of dataPath
    participantList = [f for f in listdir(dataPath) if isdir(join(dataPath, f))]

    off = 17
    print '-------------------'
    print 'Training Settings:'
    print
    print 'Init mode: '.ljust(off), initMode
    print 'Data Path: '.ljust(off), dataPath
    print 'Model Path: '.ljust(off), modelPath
    print 'Participants: '.ljust(off), participantList
    print 'Model Root Name: '.ljust(off), trainName
    print 'Training Mode:'.ljust(off), mode
    print 'Driver:'.ljust(off), driverName
    print '-------------------'
    print 'Loading Parameters...'
    print
    temporalFlag = False
    modeConfig = ''
    found = ''
    try:
        parser = SafeConfigParser()
        found = parser.read(dataPath + "/config.ini")

        if parser.has_option(trainName, 'update_mode'):
            modeConfig = parser.get(trainName, 'update_mode')
        else:
            modeConfig = 'update'
        print modeConfig
    except IOError:
        pass

    defaultParamsList = ['experiment_number', 'model_type', 'model_num_inducing',
                         'model_num_iterations', 'model_init_iterations', 'verbose',
                         'Quser', 'kernelString', 'ratioData', 'update_mode', 'model_mode', 'windowSize']

    mySAMpy.experiment_number = None
    mySAMpy.model_type = None
    mySAMpy.kernelString = None
    mySAMpy.fname = None
    mySAMpy.ratioData = None

    if initMode == 'training' and (mode == 'new' or modeConfig == 'new' or 'exp' not in modelPath):
        print 'Loading training parameters from: \n ', '\t' + dataPath + "/config.ini"
        try:
            default = False
            parser = SafeConfigParser()
            parser.optionxform = str
            found = parser.read(dataPath + "/config.ini")

            # load parameters from config file
            if parser.has_option(trainName, 'experiment_number'):
                mySAMpy.experiment_number = int(parser.get(trainName, 'experiment_number'))
            elif '.pickle' in modelPath:
                mySAMpy.experiment_number = int(modelPath.split('__')[-2].replace('exp', '')) + 1
            else:
                fail = True
                print 'No experiment_number found'

            if parser.has_option(trainName, 'model_type'):
                mySAMpy.model_type = parser.get(trainName, 'model_type')
            else:
                default = True
                mySAMpy.model_type = 'mrd'

            if parser.has_option(trainName, 'model_num_inducing'):
                mySAMpy.model_num_inducing = int(parser.get(trainName, 'model_num_inducing'))
            else:
                default = True
                mySAMpy.model_num_inducing = 30

            if parser.has_option(trainName, 'model_num_iterations'):
                mySAMpy.model_num_iterations = int(parser.get(trainName, 'model_num_iterations'))
            else:
                default = True
                mySAMpy.model_num_iterations = 700

            if parser.has_option(trainName, 'model_init_iterations'):
                mySAMpy.model_init_iterations = int(parser.get(trainName, 'model_init_iterations'))
            else:
                default = True
                mySAMpy.model_init_iterations = 2000

            if parser.has_option(trainName, 'verbose'):
                mySAMpy.verbose = parser.get(trainName, 'verbose') == 'True'
            else:
                default = True
                mySAMpy.verbose = False

            if parser.has_option(trainName, 'model_mode'):
                mySAMpy.model_mode = parser.get(trainName, 'model_mode')
                if mySAMpy.model_mode == 'temporal' and parser.has_option(trainName, 'windowSize'):
                        mySAMpy.windowSize = int(parser.get(trainName, 'windowSize'))
                else:
                    temporalFlag = True
            else:
                default = True
                mySAMpy.model_mode = 'single'

            if parser.has_option(trainName, 'Quser'):
                mySAMpy.Quser = int(parser.get(trainName, 'Quser'))
            else:
                default = True
                mySAMpy.Quser = 2

            if parser.has_option(trainName, 'kernelString'):
                mySAMpy.kernelString = parser.get(trainName, 'kernelString')
            else:
                default = True
                mySAMpy.kernelString = "GPy.kern.RBF(Q, ARD=False) + GPy.kern.Bias(Q) + GPy.kern.White(Q)"

            if parser.has_option(trainName, 'ratioData'):
                mySAMpy.ratioData = int(parser.get(trainName, 'ratioData'))
            else:
                default = True
                mySAMpy.ratioData = 50

            if default:
                print 'Default settings applied'

            mySAMpy.paramsDict = dict()
            mySAMpy.loadParameters(parser, trainName)

        except IOError:
            print 'IO Exception reading ', found
            pass
    else:
        print 'Loading parameters from: \n ', '\t' + modelPath
        try:
            parser = SafeConfigParser()
            parser.optionxform = str
            found = parser.read(dataPath + "/config.ini")

            # load parameters from config file
            if parser.has_option(trainName, 'experiment_number'):
                mySAMpy.experiment_number = int(parser.get(trainName, 'experiment_number'))
            else:
                mySAMpy.experiment_number = int(modelPath.split('__')[-2].replace('exp', ''))

            modelPickle = pickle.load(open(modelPath+'.pickle', 'rb'))
            mySAMpy.paramsDict = dict()
            for j in parser.options(trainName):
                if j not in defaultParamsList:
                    print j
                    mySAMpy.paramsDict[j] = modelPickle[j]

            mySAMpy.ratioData = modelPickle['ratioData']
            mySAMpy.model_type = modelPickle['model_type']
            mySAMpy.model_mode = modelPickle['model_mode']
            if mySAMpy.model_mode == 'temporal':
                mySAMpy.windowSize = modelPickle['windowSize']
                mySAMpy.model_type = 'mrd'
            mySAMpy.model_num_inducing = modelPickle['model_num_inducing']
            mySAMpy.model_num_iterations = modelPickle['model_num_iterations']
            mySAMpy.model_init_iterations = modelPickle['model_init_iterations']
            mySAMpy.verbose = modelPickle['verbose']
            mySAMpy.Quser = modelPickle['Quser']
            mySAMpy.kernelString = modelPickle['kernelString']
            try:
                mySAMpy.listOfModels = modelPickle['listOfModels']
                mySAMpy.classifiers = modelPickle['classifiers']
                mySAMpy.classif_thresh = modelPickle['classif_thresh']
            except:
                pass

        except IOError:
            print 'IO Exception reading ', found
            pass

    if 'exp' in modelPath:
        fnameProto = '/'.join(modelPath.split('/')[:-1]) + '/' + dataPath.split('/')[-1] + '__' + driverName + \
                             '__' + mySAMpy.model_type + '__exp' + str(mySAMpy.experiment_number)
    else:
        fnameProto = modelPath + dataPath.split('/')[-1] + '__' + driverName + '__' + mySAMpy.model_type + \
                             '__exp' + str(mySAMpy.experiment_number)

    print 'Full model name: \n', '\t' + fnameProto
    print '-------------------'
    print

    mySAMpy.save_model = False
    mySAMpy.economy_save = True
    mySAMpy.visualise_output = False
    # test_mode = True

    mySAMpy.readData(dataPath, participantList)
    # at this point, all the data that will be eventually used for training is contained in mySAMpy.Y
    # and mySAMpy.L contains all labels if any (depending on mrd model or bgplvm model)
    # mySAMpy.L is a list of labels while mySAMpy.Y is a numpy array of data
    # mySAMpy.Y should have 2 dimensions, length of dimension 0 = number of instances
    # length of dimension 1 = length of feature vector

    if mySAMpy.model_mode != 'temporal':
        # get list of labels
        mySAMpy.textLabels = list(set(mySAMpy.L))

        # convert L from list of strings to array of indices
        mySAMpy.L = np.asarray([mySAMpy.textLabels.index(i) for i in mySAMpy.L])[:, None]
        mySAMpy.textLabels = mySAMpy.textLabels
    else:
        mySAMpy.X, mySAMpy.Y = transformTimeSeriesToSeq(mySAMpy.Y1, mySAMpy.windowSize)
        mySAMpy.L, mySAMpy.tmp = transformTimeSeriesToSeq(mySAMpy.U1, mySAMpy.windowSize)

    mm = [mySAMpy]
    # mm.append(mySAMpy)
    # mm[0] contains root model
    # this is the only model in the case of a single model
    # or contains all info for the rest of the models in case of multiple models
    #

    if mySAMpy.model_mode == 'single' or mySAMpy.model_mode == 'temporal':
        mm[0].participantList = ['all']
    else:
        mm[0].participantList = ['root'] + mySAMpy.textLabels

    for k in range(len(mm[0].participantList)):
        if mm[0].participantList[k] == 'all':
            minData = len(mm[k].L)
            mm[0].fname = fnameProto
            mm[0].model_type = mySAMpy.model_type
            Ntr = int(mySAMpy.ratioData * minData / 100)
        else:
            if k > 0:
                mm.append(Driver())
                # extract subset of data corresponding to this model
                inds = [i for i in range(len(mm[0].Y['L'])) if mm[0].Y['L'][i] == k - 1]
                mm[k].Y = mm[0].Y['Y'][inds]
                mm[k].L = mm[0].Y['L'][inds]
                mm[k].Quser = mm[0].Quser
                mm[k].verbose = mm[0].verbose
                print 'Object class: ', mm[0].participantList[k]
                minData = len(inds)
                mm[k].fname = fnameProto + '__L' + str(k - 1)
                mm[0].listOfModels.append(mm[k].fname)
                mm[k].model_type = 'bgplvm'
                Ntr = int(mySAMpy.ratioData * minData / 100)
            else:
                mm[0].listOfModels = []
                mm[0].fname = fnameProto
                mm[0].SAMObject.kernelString = ''
                minData = len(mm[0].L)
                Ntr = int(mySAMpy.ratioData * minData / 100)
            mm[k].modelLabel = mm[0].participantList[k]

        if mm[0].model_mode != 'temporal':
            [Yall, Lall, YtestAll, LtestAll] = mm[k].prepareData(mm[k].model_type, Ntr,
                                                                 randSeed=mm[0].experiment_number)
            mm[k].Yall = Yall
            mm[k].Lall = Lall
            mm[k].YtestAll = YtestAll
            mm[k].LtestAll = LtestAll
        elif mm[0].model_mode == 'temporal':
            [Xall, Yall, Lall, XtestAll, YtestAll, LtestAll] = mm[k].prepareData(mm[k].model_type, Ntr,
                                                                                 randSeed=mm[0].experiment_number)
            mm[k].Xall = Xall
            mm[k].Yall = Yall
            mm[k].Lall = Lall
            mm[k].XtestAll = XtestAll
            mm[k].YtestAll = YtestAll
            mm[k].LtestAll = LtestAll

        print 'minData = ' + str(minData)
        print 'ratioData = ' + str(mySAMpy.ratioData)
    print '-------------------------------------------------------------------------------------------------'
    if initMode == 'training':
        samOptimiser.deleteModel(modelPath, 'exp' + str(mm[0].experiment_number))
        for k in range(len(mm[0].participantList)):
            # for k = 0 check if multiple model or not
            if mm[0].participantList[k] != 'root':

                print "Training with ", mm[0].model_num_inducing, 'inducing points for ', \
                    mm[0].model_init_iterations, '|', mm[0].model_num_iterations

                mm[k].training(mm[0].model_num_inducing, mm[0].model_num_iterations,
                               mm[0].model_init_iterations, mm[k].fname, mm[0].save_model,
                               mm[0].economy_save, keepIfPresent=False, kernelStr=mm[0].kernelString)

                if mm[0].visualise_output:
                    ax = mm[k].SAMObject.visualise()
                    visualiseInfo = dict()
                    visualiseInfo['ax'] = ax
                else:
                    visualiseInfo = None
    else:
        for k in range(len(mm[0].participantList)):
            # for k = 0 check if multiple model or not
            if mm[0].participantList[k] != 'root':
                print "Training with ", mm[0].model_num_inducing, 'inducing points for ', \
                    mm[0].model_init_iterations, '|', mm[0].model_num_iterations

                mm[k].training(mm[0].model_num_inducing, mm[0].model_num_iterations,
                               mm[0].model_init_iterations, mm[k].fname, mm[0].save_model,
                               mm[0].economy_save, keepIfPresent=True, kernelStr=mm[0].kernelString)

    return mm


class TimeoutError(Exception):
    pass


class InterruptableThread(threading.Thread):
    def __init__(self, func, *args, **kwargs):
        threading.Thread.__init__(self)
        self._func = func
        self._args = args
        self._kwargs = kwargs
        self._result = None

    def run(self):
        self._result = self._func(*self._args, **self._kwargs)

    @property
    def result(self):
        return self._result


class timeout(object):
    def __init__(self, sec):
        self._sec = sec

    def __call__(self, f):
        def wrapped_f(*args, **kwargs):
            it = InterruptableThread(f, *args, **kwargs)
            it.start()
            it.join(self._sec)
            if not it.is_alive():
                return it.result
            raise TimeoutError('execution expired')
        return wrapped_f


def transformTimeSeriesToSeq(Y, timeWindow, normalised=False, reduced=False, noY=False):
    # TODO add parameter for number of points to skip between sampled windows
    Ntr, D = Y.shape
    if noY:
        blocksNumber = Ntr - timeWindow + 1
    else:
        blocksNumber = Ntr - timeWindow
    if normalised and reduced:
        X = np.zeros((blocksNumber, (timeWindow - 1) * D))
    else:
        X = np.zeros((blocksNumber, timeWindow * D))

    if not noY:
        Ynew = np.zeros((blocksNumber, D))
    else:
        Ynew = None

    for i in range(blocksNumber):
        tmp = Y[i:i + timeWindow, :].T

        if normalised:
            tmp = np.subtract(tmp, tmp[:, 0][:, None])
            if reduced:
                tmp = np.delete(tmp, 0, 1)
        X[i, :] = tmp.flatten().T

        if not noY:
            Ynew[i, :] = Y[i + timeWindow, :]

    return X, Ynew


def transformSeqToTimeSeries(X, Y, timeWindow):
    assert (X.shape[0] == Y.shape[0])
    N = X.shape[0] + timeWindow
    D = X.shape[1] / (timeWindow * 1.0)
    Ynew = np.zeros((N, D))
    for i in range(X.shape[0]):
        Ynew[i:i + timeWindow, :] = X[i, :].reshape(D, timeWindow).T
    Ynew[-1, :] = Y[-1, :]
    return Ynew


def test_transformSeries(Y, timeWindow):
    (xx, yy) = transformTimeSeriesToSeq(Y, timeWindow)
    return transformSeqToTimeSeries(xx, yy, timeWindow)


def gp_narx(m, x_start, N, Uts, ws, Ydebug=None):
    # m is a GP model from GPy.

    D = m.output_dim
    Q = x_start.shape[1]
    Y = np.empty((N, D,))
    Y[:] = np.NAN
    varY = Y.copy()
    assert (Q % ws == 0)
    assert (D == Q / ws)

    Xnew = m.X.copy()
    Ynew = m.Y.copy()

    curX = x_start

    varYpred = None

    for i in range(N):
        # Make sure the added x_add is a matrix (1,Q) and not (Q,)
        if len(curX.shape) < 2:
            curX = curX.reshape(1, curX.shape[0])
        varYpred_prev = varYpred
        # Ypred, varYpred = m._raw_predict(np.hstack((curX,curU)))
        # curU = Uts[i,:]
        # Ypred, varYpred = m._raw_predict(np.hstack((curX,Uts[i,:][None,:])))
        if Uts is not None:
            Ypred, varYpred = m.predict(np.hstack((curX, Uts[i, :][None, :])))
        else:
            Ypred, varYpred = m.predict(curX)

        Y[i, :] = Ypred
        varY[i, :] = varYpred

        # print i, ': ', Y[i,:] , ' | var: ', varYpred  #####

        if Ydebug is not None:
            if Uts is not None:
                print i, ': X=', str(curX.flatten()), 'U=', str(Uts[i, :].flatten()), 'Y=', str(Ydebug[i, :])
            else:
                print i, ': X=', str(curX.flatten()), 'U=None', 'Y=', str(Ydebug[i, :])

        if i == N - 1:
            break

        curX = np.hstack((curX[0, D:], Ypred[0, :]))

    return Y, varY


def random_data_split(Y, percentage=[0.5, 0.5]):
    N = Y.shape[0]
    N_1 = np.ceil(N*percentage[0])
    N_2 = np.floor(N*percentage[1])
    assert(N==N_1+N_2)
    perm = np.random.permutation(N)
    inds_1 = perm[0:N_1]
    inds_2 = perm[N_1:N_1+N_2]
    return Y[inds_1,:], Y[inds_2,:], inds_1, inds_2


class SURFProcessor:
    def __init__(self, imgHNew, imgWNew,n_clusters=20,SURFthresh=500,crop_thresholds=(40,160,40,160),magnify=1):
        self.imgHNew = imgHNew
        self.imgWNew = imgWNew
        self.n_clusters = n_clusters
        self.SURFthresh = SURFthresh
        self.crop_thresholds = crop_thresholds
        self.magnify = magnify # Magnify photos (scale up) before extracting SURF

        self._is_trained = False

    def _find_surf(self,Y,thresh=None,h=None,w=None,limits=None,magnify=None):
        import sys
        if thresh is None: thresh=self.SURFthresh
        if h is None: h = self.imgHNew
        if w is None: w = self.imgWNew
        if limits is None: limits = self.crop_thresholds
        if magnify is None: magnify = self.magnify

        print '# Finding SURF features from ' + str(Y.shape[0]) + ' images...',
        sys.stdout.flush()
        import cv2
        surf = cv2.SURF(thresh)
        descriptors = []
        desclabels = []
        for i in range(Y.shape[0]):
            if magnify is not 1:
                y_tmp = cv2.resize(Y[i,:].reshape(h,w),(h*magnify,w*magnify)).flatten()
                kp, des = surf.detectAndCompute(np.uint8(y_tmp.reshape(h*magnify, w*magnify)),None)
            else:
                y_tmp = Y[i,:]
                kp, des = surf.detectAndCompute(np.uint8(y_tmp.reshape(h, w)),None)

            for j in range(len(kp)):
                if (limits is not None) and (kp[j].pt[0] < limits[0] or kp[j].pt[0]>limits[1] or kp[j].pt[1] < limits[2] or kp[j].pt[1]>limits[3]):
                    continue
                desclabels.append(i)
                descriptors.append(des[j])

        descriptors = np.array(descriptors)
        desclabels = np.array(desclabels)
        print ' Found ' + str(len(desclabels)) + ' features.'
        return descriptors, desclabels

    def _make_BoW(self,N, c_trainPredict, desclabels, n_clusters=None):
        if n_clusters is None: n_clusters = self.n_clusters
        print('# Making BoW...')
        features = []
        for i in range(N):
            feature_counts = c_trainPredict[desclabels==i]
            feature_vector = np.zeros(n_clusters)
            for j in range(n_clusters):
                feature_vector[j] = np.where(feature_counts==j)[0].shape[0]
            features.append(feature_vector)

        return np.sqrt(np.array(features))

    def make_SURF_BoW(self,Y,normalize=True,imgHNew=None, imgWNew=None,n_clusters=None,SURFthresh=None,crop_thresholds=None):
        if imgHNew is None: imgHNew = self.imgHNew;
        if imgWNew is None: imgWNew = self.imgWNew;
        if n_clusters is None: n_clusters = self.n_clusters
        if SURFthresh is None: SURFthresh = self.SURFthresh
        if crop_thresholds is None: crop_thresholds = self.crop_thresholds
        self.normalize = normalize

        from sklearn.cluster import KMeans
        from scipy.spatial import distance

        #### APPROACH 1: Bag of feature approach:
        # Cluster all descriptors (for each image) and then replace each image with a histogram
        # saying how many descriptors it has that fall in cluster K. So, K dimensional feature vec.
        
        # Thresh to exclude corners...

        ## TRAINING DATA
        descriptors, desclabels = self._find_surf(Y,SURFthresh,imgHNew, imgWNew,crop_thresholds)

        # Agglomeratice only: Define the structure A of the data. Here a 10 nearest neighbors
        #from sklearn.neighbors import kneighbors_graph
        #connectivity = kneighbors_graph(descriptors, n_neighbors=10, include_self=False)

        # Compute clustering
        #ward = AgglomerativeClustering(n_clusters=n_clusters, connectivity=connectivity,linkage='ward',compute_full_tree=True).fit(descriptors)
        #Y_ = ward.labels_

        c = KMeans(n_clusters=n_clusters, max_iter=10000,random_state=1)

        c_trainFit = c.fit(descriptors)
        c_trainPredict = c_trainFit.predict(descriptors)

        #Equiv:
        #c = KMeans(n_clusters=n_clusters, max_iter=2000,random_state=1)
        #c_trainFitPredict=c.fit_predict(descriptors)

        #plt.plot(c_trainPredict, 'x')
        Z = self._make_BoW(Y.shape[0], c_trainPredict, desclabels, n_clusters)
        self.c_trainFit = c_trainFit

        #plt.matshow(Z);  plt.colorbar(); plt.gca().set_aspect('normal');plt.draw()
        #plt.matshow(mySAMpy.L);  plt.gca().set_aspect('normal');plt.draw()

        if normalize:
            Zmean = Z.mean()
            Zn = Z - Zmean
            Zstd = Zn.std()
            Zn /= Zstd
            self.Zmean = Zmean
            self.Zstd = Zstd
            Z = Zn

        self._is_trained = True
        return Z.copy(), descriptors, desclabels, c_trainFit

    def make_SURF_BoW_test(self,Ytest):
        # SURF for test data
        assert(self._is_trained, "First you have to do a train BoW.")
        descriptors, desclabels = self._find_surf(Ytest)
        c_testPredict = self.c_trainFit.predict(descriptors)
        Ztest = self._make_BoW(Ytest.shape[0], c_testPredict, desclabels, self.n_clusters)
        if self.normalize:
            Ztestn = Ztest - self.Zmean
            Ztestn /= self.Zstd
            Ztest = Ztest

        return Ztest.copy()
