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
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from SAM.SAM_Core import samOptimiser
from os import listdir
from os.path import join, isdir
import threading
import os
import subprocess
import time
import ipyparallel as ipp


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
                         'Quser', 'kernelString', 'ratioData', 'update_mode', 'model_mode',
                         'temporalModelWindowSize', 'optimiseRecall', 'classificationDict',
                         'useMaxDistance', 'calibrateUnknown']

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

            if parser.has_option(trainName, 'optimiseRecall'):
                mySAMpy.optimiseRecall = int(parser.get(trainName, 'optimiseRecall'))
            else:
                default = True
                mySAMpy.optimiseRecall = 200

            if parser.has_option(trainName, 'useMaxDistance'):
                mySAMpy.useMaxDistance = parser.get(trainName, 'useMaxDistance') == 'True'
            else:
                mySAMpy.useMaxDistance = False

            if parser.has_option(trainName, 'calibrateUnknown'):
                mySAMpy.calibrateUnknown = parser.get(trainName, 'calibrateUnknown') == 'True'
            else:
                mySAMpy.calibrateUnknown = False

            if parser.has_option(trainName, 'model_mode'):
                mySAMpy.model_mode = parser.get(trainName, 'model_mode')
                if mySAMpy.model_mode == 'temporal' and parser.has_option(trainName, 'temporalModelWindowSize'):
                        mySAMpy.temporalWindowSize = int(parser.get(trainName, 'temporalModelWindowSize'))
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
            mySAMpy.experiment_number = int(modelPath.split('__')[-1].replace('exp', ''))

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
                mySAMpy.temporalModelWindowSize = modelPickle['temporalModelWindowSize']
                mySAMpy.model_type = 'mrd'
            mySAMpy.model_num_inducing = modelPickle['model_num_inducing']
            mySAMpy.model_num_iterations = modelPickle['model_num_iterations']
            mySAMpy.model_init_iterations = modelPickle['model_init_iterations']
            mySAMpy.verbose = modelPickle['verbose']
            mySAMpy.Quser = modelPickle['Quser']
            mySAMpy.optimiseRecall = modelPickle['optimiseRecall']
            mySAMpy.kernelString = modelPickle['kernelString']
            mySAMpy.calibrated = modelPickle['calibrated']

            # try loading classification parameters for multiple model implementation
            try:
                mySAMpy.useMaxDistance = modelPickle['useMaxDistance']
            except:
                print 'Failed to load useMaxDistace. Possible reasons: Not saved or multiple model implementation'
            mySAMpy.classificationDict = modelPickle['classificationDict']
            mySAMpy.calibrateUnknown = modelPickle['calibrateUnknown']

            # try:
            #     mySAMpy.listOfModels = modelPickle['listOfModels']
            #     mySAMpy.classifiers = modelPickle['classifiers']
            #     mySAMpy.classif_thresh = modelPickle['classif_thresh']
            #     mulClassLoadFail = False
            #     print 'Successfully loaded multiple model classifiers'
            # except:
            #     mulClassLoadFail = True
            #     print 'Failed to load multiple model classifiers'
            #     pass
            #
            # # try loading classification parameters for single model implementation
            # try:
            #     mySAMpy.varianceDirection = modelPickle['varianceDirection']
            #     mySAMpy.varianceThreshold = modelPickle['varianceThreshold']
            #     mySAMpy.bestDistanceIDX = modelPickle['bestDistanceIDX']
            #     print 'Successfully loaded single model classifiers'
            #     singClassLoadFail = False
            # except:
            #     singClassLoadFail = True
            #     print 'Failed to load single model classifiers'
            #     pass

            # if mulClassLoadFail and singClassLoadFail:
            #     raise ValueError('Failed to load model classifiers')

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
        mySAMpy.X, mySAMpy.Y = transformTimeSeriesToSeq(mySAMpy.Y1, mySAMpy.temporalModelWindowSize)
        mySAMpy.L, mySAMpy.tmp = transformTimeSeriesToSeq(mySAMpy.U1, mySAMpy.temporalModelWindowSize)

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
            normaliseData = True
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
                normaliseData = True
            else:
                normaliseData = False
                mm[0].listOfModels = []
                mm[0].fname = fnameProto
                mm[0].SAMObject.kernelString = ''
                minData = len(mm[0].L)
                Ntr = int(mySAMpy.ratioData * minData / 100)
            mm[k].modelLabel = mm[0].participantList[k]

        if mm[0].model_mode != 'temporal':

            [Yall, Lall, YtestAll, LtestAll] = mm[k].prepareData(mm[k].model_type, Ntr,
                                                                 randSeed=mm[0].experiment_number,
                                                                 normalise=normaliseData)
            mm[k].Yall = Yall
            mm[k].Lall = Lall
            mm[k].YtestAll = YtestAll
            mm[k].LtestAll = LtestAll
        elif mm[0].model_mode == 'temporal':
            [Xall, Yall, Lall, XtestAll, YtestAll, LtestAll] = mm[k].prepareData(mm[k].model_type, Ntr,
                                                                                 randSeed=mm[0].experiment_number,
                                                                                 normalise=normaliseData)
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


def varianceClass(varianceDirection, x, thresh):
    if varianceDirection == ['greater', 'smaller']:
        return thresh[0] < x < thresh[1]
    elif varianceDirection == ['smaller', 'greater']:
        return thresh[0] > x > thresh[1]
    elif varianceDirection == ['greater']:
        return x > thresh
    elif varianceDirection == ['smaller']:
        return x < thresh


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


def plotKnownAndUnknown(varDict, colour, axlist, width=[0.2, 0.2], factor=[(0, 0.6), (0.4, 1)], plotRange=False):
    count = 0
    for k, j in enumerate(varDict.keys()):
        if len(varDict[j]) > 0 and 'Results' not in j:
            [mlist, vlist, rlist] = meanVar_varianceDistribution(varDict[j])
            axlist = plotGaussFromList(mlist, vlist, rlist, colour[count], j, width[count], factor[count], axlist, plotRange)
            count += 1

    return axlist


# This function provides a measure for the separability of two univariate gaussians
# Main purpose to get a distance that is used to optimise for separability
# between known and unknown classes
# TODO extend method to multivariate gaussians
def bhattacharyya_distance(mu1, mu2, var1, var2):
    t1 = float(var1/var2) + float(var2/var1) + 2
    t2 = np.log(0.25*t1)
    t3 = float(mu1-mu2)*float(mu1-mu2)
    t4 = t3/float(var1+var2)
    return 0.25*t2 + 0.25*t4


def plotGaussFromList(mlist, vlist, rlist, colour, label, width, factor, axlist, plotRange=False):
    numPlots = len(mlist)

    if len(axlist) == 0:
        # f, axlist = plt.subplots(1, numPlots, figsize=(24.0, 15.0))
        f, axlist = plt.subplots(1, numPlots, figsize=(12.0, 7.5))
        for k, j in enumerate(axlist):
            if k < numPlots - 2:
                j.set_title('D ' + str(k), fontsize=20)
            elif k == numPlots - 2:
                j.set_title('Sum', fontsize=20)
            elif k > numPlots - 2:
                j.set_title('Mean', fontsize=20)
            j.set_xticks([])
            j.set_yticks([])

    for j in range(numPlots):
        sigma = np.sqrt(vlist[j])
        rangeData = rlist[j][1] - rlist[j][0]
        x = np.linspace(rlist[j][0] - (rangeData / 2), rlist[j][1] + (rangeData / 2), 100)
        y = mlab.normpdf(x, mlist[j], sigma)
        axlist[j].plot(x, y, colour, label=label)
        if plotRange:
            axlist[j].plot([rlist[j][1], rlist[j][1]], [max(y)*factor[0], max(y)*factor[1]], '--'+colour, linewidth=width)
            axlist[j].plot([rlist[j][0], rlist[j][0]], [max(y)*factor[0], max(y)*factor[1]], '--'+colour, linewidth=width)

    return axlist


def solve_intersections(m1, m2, std1, std2):
    a = 1/(2*std1**2) - 1/(2*std2**2)
    b = m2/(std2**2) - m1/(std1**2)
    c = m1**2 / (2*std1**2) - m2**2 / (2*std2**2) - np.log(std2/std1)
    return np.roots([a, b, c])


def PfromHist(sample, hist, binWidth):
    idx = np.asarray(sample)//binWidth
    idx = idx.astype(np.int)
    pList = []
    for j in range(len(idx)):
        pList.append(hist[j][idx[j]])
    return pList


def meanVar_varianceDistribution(dataList):
    mlist = []
    vlist = []
    rlist = []

    dataArray = np.asarray(dataList)
    if len(dataArray.shape) == 1:
        dataArray = dataArray[:, None]

    numPlots = dataArray.shape[1]

    for j in range(numPlots):
        # if j < numPlots:
        h = dataArray[:, j]
        # elif j == numPlots:
        #     h = np.sum(dataArray, 1)
        # elif j == numPlots + 1:
        #     h = np.mean(dataArray, 1)

        mean = np.mean(h)
        variance = np.var(h)
        rlist.append((min(h), max(h)))
        mlist.append(mean)
        vlist.append(variance)

    return mlist, vlist, rlist


def bhattacharyya_dict(m, v):
    knownLabel = None
    unknownLabel = None
    dists = []

    for j in m.keys():
        if 'known' == j.lower().split(' ')[1]:
            knownLabel = j
        elif 'unknown' in j.lower().split(' ')[1]:
            unknownLabel = j

    if unknownLabel is not None and knownLabel is not None:
        numDists = len(m[knownLabel])
        for j in range(numDists):
            dists.append(
                bhattacharyya_distance(m[knownLabel][j], m[unknownLabel][j], v[knownLabel][j], v[unknownLabel][j]))
        return dists
    else:
        return None


def smooth1D(x, window_len=11, window='hanning'):
    """smooth the data using a window with requested size.

    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal
    (with the window size) in both ends so that transient parts are minimized
    in the beginning and end part of the output signal.

    input:
        x: the input signal
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
            flat window will produce a moving average smoothing.

    output:
        the smoothed signal

    NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
    """

    if x.ndim != 1:
        raise ValueError("smooth only accepts 1 dimension arrays.")

    if x.size < window_len:
        raise ValueError("Input vector needs to be bigger than window size.")

    if window_len < 3:
        return x

    if window not in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
        raise ValueError("Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'")

    s = np.r_[x[window_len-1:0:-1], x, x[-1:-window_len:-1]]
    # print(len(s))
    if window == 'flat':  # moving average
        w = np.ones(window_len, 'd')
    else:
        w = eval('np.'+window+'(window_len)')

    y = np.convolve(w/w.sum(), s, mode='valid')
    off = 0
    if window_len % 2 > 0:
        off = 1

    return y[(window_len/2-1):-((window_len/2)+off)]


def transformTimeSeriesToSeq(Y, timeWindow, offset=1, normalised=False, reduced=False, noY=False, doOffset=False):
    # TODO add parameter for number of points to skip between sampled windows
    Ntr, D = Y.shape
    if noY:
        blocksNumber = (Ntr - timeWindow + 1) // offset
    else:
        blocksNumber = (Ntr - timeWindow) // offset

    if normalised and reduced:
        X = np.zeros((blocksNumber, (timeWindow - 1) * D))
    else:
        X = np.zeros((blocksNumber, timeWindow * D))

    if not noY:
        Ynew = np.zeros((blocksNumber, D))
    else:
        Ynew = None

    for i in range(blocksNumber):
        base = i * offset
        tmp = Y[base:base + timeWindow, :].T

        if normalised:
            tmp = np.subtract(tmp, tmp[:, 0][:, None])
            if reduced:
                tmp = np.delete(tmp, 0, 1)
        X[i, :] = tmp.flatten().T

        if not noY:
            Ynew[i, :] = Y[base + timeWindow, :]
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
        assert self._is_trained, "First you have to do a train BoW."
        descriptors, desclabels = self._find_surf(Ytest)
        c_testPredict = self.c_trainFit.predict(descriptors)
        Ztest = self._make_BoW(Ytest.shape[0], c_testPredict, desclabels, self.n_clusters)
        if self.normalize:
            Ztestn = Ztest - self.Zmean
            Ztestn /= self.Zstd
            Ztest = Ztest

        return Ztest.copy()


def RepresentsInt(s):
    try:
        int(s)
        return True
    except ValueError:
        return False


class ipyClusterManager:
    def __init__(self, nodesDict, controllerIP, devnull, totalControl=True):
        self.expectedProcessors = 0
        self.actualProcessors = 0
        self.totalControl = totalControl
        self.controllerProc = []
        self.nodesDict = nodesDict
        self.controllerIP = controllerIP
        self.devnull = devnull

    def startCluster(self):
        try:
            self.controllerProc.append(
                subprocess.Popen(["ipcontroller", '--ip=' + self.controllerIP], stdout=self.devnull))
            for j in self.nodesDict.keys():
                if j != 'localhost':
                    cmd = 'scp ~/.ipython/profile_default/security/ipcontroller-engine.json ' + j + ':./'
                    os.system(cmd)
                    time.sleep(5)

            for j in self.nodesDict.keys():
                print j
                if j != 'localhost':
                    if self.totalControl:
                        cmd = ['ssh', j, 'ipengine', '--file=~/ipcontroller-engine.json', '&']
                    else:
                        cmd = 'ssh ' + j + ' ipengine --file=~/ipcontroller-engine.json &'
                else:
                    if self.totalControl:
                        cmd = ['ipengine', '&']
                    else:
                        cmd = 'ipengine &'

                for n in range(self.nodesDict[j]):
                    self.expectedProcessors += 1
                    print '\t' + ' '.join(cmd)
                    if self.totalControl:
                        self.controllerProc.append(subprocess.Popen(cmd, stdout=self.devnull))
                    else:
                        os.system(cmd)
                    time.sleep(2)

            print 'Waiting for engines to start'
            time.sleep(max(self.expectedProcessors, 10))
            success = True
            try:
                c = ipp.Client()
                self.actualProcessors = len(c._engines)
                c.close()
                del c
                print 'Controller started correctly'
            except:
                success = False
                self.terminateProcesses()
                print 'Controller failure'

            if self.actualProcessors == 0:
                success = False
                print 'Complete engine failure'
            else:
                print 'Engines started correctly'
        except:
            success = False
            self.terminateProcesses()
            print 'Failed to initialise controller'

        return success

    def terminateProcesses(self):
        for j in self.controllerProc:
            try:
                j.kill()
                j.wait()
            except:
                pass
            time.sleep(0.2)
        self.actualProcessors = 0
