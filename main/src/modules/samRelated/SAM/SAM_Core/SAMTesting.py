# """"""""""""""""""""""""""""""""""""""""""""""
# The University of Sheffield
# WYSIWYD Project
#
# A class includes the different testing methods implemented
# Currently tested only with Actions
#
# Created on 20 July 2016
#
# @author: Daniel Camilleri, Andreas Damianou
#
# """"""""""""""""""""""""""""""""""""""""""""""
from IPython.display import clear_output
from sklearn.mixture import GMM
from SAM.SAM_Core import SAM_utils as utils
import ipyparallel as ipp
import time
import matplotlib.pyplot as plt
import numpy.matlib
import sys
import copy
import psutil
import timeit
import numpy as np
from collections import Mapping, Container
from sys import getsizeof
from operator import gt

np.set_printoptions(threshold=np.nan)


# thisModel = None


def deep_getsizeof(o, ids):
    d = deep_getsizeof
    if id(o) in ids:
        return 0

    r = getsizeof(o)
    ids.add(id(o))

    if isinstance(o, str) or isinstance(0, unicode):
        return r

    if isinstance(o, Mapping):
        return r + sum(d(k, ids) + d(v, ids) for k, v in o.iteritems())

    if isinstance(o, Container):
        return r + sum(d(x, ids) for x in o)

    if 'SAM' in o.__class__.__name__:
        total = 0
        for attr, value in o.__dict__.iteritems():
            # print attr
            total += d(value, ids)
        return r + total

    return r


def calibrateModelRecall(thisModel):
    if len(thisModel) > 1:
        calibrateMultipleModelRecall(thisModel)
    elif hasattr(thisModel[0], 'allDataDict'):
        print 'calibrating model'
        calibrateSingleModelRecall(thisModel)
    else:
        print 'no calibration'


def calibrateSingleModelRecall(thisModel):
    yCalib = formatDataFunc(thisModel[0].allDataDict['Y'])
    print 'entering segment testing'
    labelList, confMatrix, ret, variancesKnown, variancesUnknown = segmentTesting(thisModel, yCalib,
                                                                                  thisModel[0].allDataDict['L'],
                                                                                  thisModel[0].verbose, 'calib',
                                                                                  serialMode=False,
                                                                                  optimise=thisModel[0].optimiseRecall,
                                                                                  calibrate=True)
    thisModel[0].classificationDict = dict()

    if thisModel[0].useMaxDistance:
        [mk, vk, rk] = utils.meanVar_varianceDistribution(variancesKnown)
        [muk, vuk, ruk] = utils.meanVar_varianceDistribution(variancesUnknown)

        distance = []
        for j in range(len(mk)):
            distance.append(utils.bhattacharyya_distance(mk[j], muk[j], vk[j], vuk[j]))

        if distance is not None:
            maxIdx = distance.index(max(distance))
        thisModel[0].classificationDict['bestDistanceIDX'] = maxIdx
        thisModel[0].classificationDict['bestDistance_props'] = {'KnownMean': mk[maxIdx], 'UnknownMean': muk[maxIdx],
                                                                 'KnownVar': vk[maxIdx], 'UnknownVar': vuk[maxIdx]}

        # if maxIdx < len(mk) - 2:
        #     thisModel[0].bestSegOperation = maxIdx
        # elif maxIdx == len(mk) - 2:
        #     thisModel[0].bestSegOperation = 'sum'
        # elif maxIdx == len(mk) - 1:
        #     thisModel[0].bestSegOperation = 'mean'

        intersection = utils.solve_intersections(mk[maxIdx], muk[maxIdx], np.sqrt(vk[maxIdx]), np.sqrt(vuk[maxIdx]))

        maxLim = max(rk[maxIdx][1], ruk[maxIdx][1])
        minLim = min(rk[maxIdx][0], ruk[maxIdx][0])

        delList = []
        for j in range(len(intersection)):
            if intersection[j] > maxLim or intersection[j] < minLim:
                delList.append(j)

        thisModel[0].classificationDict['segIntersections'] = np.delete(intersection, delList)
        thisModel[0].classificationDict['bhattaDistances'] = distance

        print 'Num Intersections: ', len(thisModel[0].classificationDict['segIntersections'])

        [thisModel[0].classificationDict['varianceThreshold'],
         thisModel[0].classificationDict['varianceDirection']] = \
            calculateVarianceThreshold(thisModel[0].classificationDict['segIntersections'], mk[maxIdx], muk[maxIdx],
                                       vk[maxIdx], vuk[maxIdx])

        print 'varianceThreshold', thisModel[0].classificationDict['varianceThreshold']
        print 'varianceDirection', thisModel[0].classificationDict['varianceDirection']
    else:
        variancesKnownArray = np.asarray(variancesKnown)
        variancesUnknownArray = np.asarray(variancesUnknown)
        varianceAllArray = np.vstack([variancesKnownArray, variancesUnknownArray])
        histKnown = [None] * (len(variancesKnownArray[0]) - 2)
        binEdges = [None] * (len(variancesKnownArray[0]) - 2)
        histUnknown = [None] * (len(variancesKnownArray[0]) - 2)

        thisModel[0].classificationDict['binWidth'] = thisModel[0].paramsDict['binWidth']
        thisModel[0].classificationDict['method'] = thisModel[0].paramsDict['method']

        numBins = np.ceil(np.max(varianceAllArray) / thisModel[0].classificationDict['binWidth'])

        bins = range(int(numBins))
        bins = np.multiply(bins, thisModel[0].classificationDict['binWidth'])

        for j in range(len(variancesKnown[0]) - 2):
            histKnown[j], binEdges[j] = np.histogram(variancesKnownArray[:, j], bins=bins)
            histKnown[j] = 1.0 * histKnown[j] / np.sum(histKnown[j])

            histUnknown[j], _ = np.histogram(variancesUnknownArray[:, j], bins=bins)
            histUnknown[j] = 1.0 * histUnknown[j] / np.sum(histUnknown[j])

        thisModel[0].classificationDict['histKnown'] = histKnown
        thisModel[0].classificationDict['binEdgesKnown'] = binEdges
        thisModel[0].classificationDict['histUnknown'] = histUnknown

    thisModel[0].calibrated = True


def calibrateMultipleModelRecall(thisModel):
    cmSize = len(thisModel[0].textLabels)
    confMatrix = np.zeros((cmSize, cmSize))

    # Create Validation set
    Y_valid = []
    Y_testing = []
    for i in range(len(thisModel)):
        if thisModel[i].SAMObject.model:
            # De-normalize from the model which stored this test data
            yy_test = thisModel[i].Ytestn.copy()
            yy_test *= thisModel[i].Ystd
            yy_test += thisModel[i].Ymean
            y_valid_tmp, y_test_tmp, _, _ = utils.random_data_split(yy_test, [0.5, 0.5])
            Y_valid.append(y_valid_tmp.copy())
            Y_testing.append(y_test_tmp.copy())

    # Compute familiarities in VALIDATION SET
    familiarities = [None] * (len(thisModel) - 1)
    for i in range(len(thisModel)):
        if thisModel[i].SAMObject.model:
            # N_test x N_labels matrix.
            familiarities[i - 1] = np.zeros((Y_valid[i - 1].shape[0], (len(thisModel) - 1)))
            print("## True label is " + thisModel[i].modelLabel)
            for k in range(Y_valid[i - 1].shape[0]):
                sstest = []
                print('# k=' + str(k))
                for j in range(len(thisModel)):
                    if thisModel[j].SAMObject.model:
                        yy_test = Y_valid[i - 1][k, :][None, :].copy()
                        # Normalize according to the model to predict
                        yy_test -= thisModel[j].Ymean
                        yy_test /= thisModel[j].Ystd
                        sstest.append(thisModel[j].SAMObject.familiarity(yy_test, optimise=thisModel[0].optimiseRecall))
                        familiarities[i - 1][k, j - 1] = sstest[-1]
                for j in range(len(sstest)):
                    if j == np.argmax(sstest):
                        print '   *',
                    else:
                        print '    ',
                    print('      Familiarity of model ' + thisModel[j + 1].modelLabel + ' given label: ' +
                          thisModel[i].modelLabel + ' in valid: ' + str(sstest[j]))

                confMatrix[i - 1, np.argmax(sstest)] += 1
    calculateData(thisModel[0].textLabels, confMatrix)

    # At this point we have:
    # familiarities[i][k,j] -> familiarity for true label i, instance k
    #                          predicted by model trained in label j
    # ############# Train Familiarity classifier in VALIDATION SET
    #
    classifiers = []
    classif_thresh = []
    familiarity_predictions = []
    tmp = []
    for i in range(len(thisModel[0].textLabels)):
        X_train = familiarities[0][:, i][:, None]
        y_train = np.zeros((familiarities[0][:, i][:, None].shape[0], 1))
        for j in range(1, len(thisModel[0].textLabels)):
            X_train = np.vstack((X_train, familiarities[j][:, i][:, None]))
            y_train = np.vstack((y_train, j + np.zeros((familiarities[j][:, i][:, None].shape[0], 1))))
        tmp.append(X_train)
        n_classes = len(np.unique(y_train))

        # Try GMMs using different types of covariances.
        classifiers.append(GMM(n_components=n_classes, covariance_type='full', init_params='wc', n_iter=2000))

        # Since we have class labels for the training data, we can
        # initialize the GMM parameters in a supervised manner.
        classifiers[-1].means_ = np.array([X_train[y_train == kk].mean(axis=0)
                                           for kk in xrange(n_classes)])[:, None]
        classifiers[-1].fit(X_train)
        familiarity_predictions.append(classifiers[-1].predict(X_train))

        # Find threshold of confident classification of model i predicting label i
        tmp_i = classifiers[i].predict_proba(X_train[y_train == i][:, None])[:, i]
        tmp_s = 0.8
        # If in the test phase we get a predict_proba which falls in the threshold i, then
        # model i is confident for this prediction.
        classif_thresh.append([tmp_i.mean() - tmp_s * tmp_i.std(), tmp_i.mean() + tmp_s * tmp_i.std()])

    thisModel[0].classifiers = classifiers
    thisModel[0].classif_thresh = classif_thresh
    thisModel[0].calibrated = True


def formatDataFunc(Ydata):
    yDataList = []
    for j in range(Ydata.shape[0]):
        yDataList.append(Ydata[j][None, :])
    return yDataList


def singleRecall(thisModel, testInstance, verbose, visualiseInfo=None, optimise=100):
    # Returns the predictive mean, the predictive variance and the axis (pp) of the latent space backwards mapping.
    # mm,vv,pp=self.SAMObject.pattern_completion(testFace, visualiseInfo=visualiseInfo)
    # if verbose:
    # print 'single model recall'
    textStringOut = ''
    # normalize incoming data
    testValue = testInstance - thisModel.Ymean
    testValue /= thisModel.Ystd

    ret = thisModel.SAMObject.pattern_completion(testValue, visualiseInfo=visualiseInfo, optimise=optimise)

    mm = ret[0]
    vv = list(ret[1][0])
    svv = sum(vv)
    mvv = svv/len(vv)
    vv.append(svv)
    vv.append(mvv)

    # find nearest neighbour of mm and SAMObject.model.X

    k = np.matlib.repmat(mm[0].values, thisModel.SAMObject.model.X.mean.shape[0], 1)
    pow2 = np.power(thisModel.SAMObject.model.X.mean - k, 2)
    s = np.power(np.sum(pow2, 1), 0.5)
    nn = np.argmin(s)
    min_value = s[nn]

    if thisModel.SAMObject.type == 'mrd':
        classLabel = thisModel.textLabels[int(thisModel.SAMObject.model.bgplvms[1].Y[nn, :])]
    elif thisModel.SAMObject.type == 'bgplvm':
        classLabel = thisModel.textLabels[int(thisModel.L[nn, :])]

    known = True
    if thisModel.calibrated:
        if thisModel.useMaxDistance:
            known = utils.varianceClass(thisModel.classificationDict['varianceDirection'],
                                vv[thisModel.classificationDict['bestDistanceIDX']],
                                thisModel.classificationDict['varianceThreshold'])

            details = str(thisModel.classificationDict['varianceThreshold']) + ' ' + \
                      str(thisModel.classificationDict['varianceDirection'])

            probClass = vv[thisModel.classificationDict['bestDistanceIDX']]
        else:
            P_Known_given_X = utils.PfromHist(vv[:-2], thisModel.classificationDict['histKnown'],
                                              thisModel.classificationDict['binWidth'])
            P_Unknown_given_X = utils.PfromHist(vv[:-2], thisModel.classificationDict['histUnknown'],
                                                thisModel.classificationDict['binWidth'])

            if thisModel.classificationDict['method'] == 'mulProb':
                s1 = reduce(lambda x, y: x * y, P_Known_given_X)
                s2 = reduce(lambda x, y: x * y, P_Unknown_given_X)
                known = s1 > s2
            else:
                s1 = np.sum(P_Known_given_X)
                s2 = np.sum(P_Unknown_given_X)
                known = s1 > s2

            if known:
                probClass = s1
                details = s1, ' > ', s2
            else:
                probClass = s2
                details = s2, ' > ', s1

    if thisModel.calibrated:
        if known:
            textStringOut = classLabel
        else:
            textStringOut = 'unknown'
            runnerUp = classLabel
    else:
        textStringOut = classLabel

    if verbose:
        if thisModel.calibrated:
            if textStringOut == 'unknown':
                print "With ", probClass, "prob. error the new instance is", runnerUp
                print 'But', details, 'than', probClass, 'so class as', textStringOut
            else:
                print "With ", probClass, "prob. error the new instance is", textStringOut
        else:
            print "With", vv, "prob. error the new instance is", textStringOut

    return [textStringOut, vv]


def multipleRecall_noCalib(thisModel, testInstance, verbose, visualiseInfo=None, optimise=True):
    result = []
    if verbose:
        pass
    # print 'multiple model recall'

    for j in thisModel:
        if j.SAMObject.model:
            tempTest = testInstance - j.Ymean
            tempTest /= j.Ystd
            yy_test = j.SAMObject.familiarity(tempTest, optimise=optimise)
            if verbose:
                print('Familiarity with ' + j.modelLabel + ' given current instance is: ' + str(yy_test))
            # yy_test -= thisModel[j].Ymean
            # yy_test /= thisModel[j].Ystd
            result.append(yy_test)
    maxIdx = np.argmax(result)

    if visualiseInfo:
        pass

    return [thisModel[0].textLabels[maxIdx - 1], result[maxIdx][0]]


def multipleRecall(thisModel, testInstance, verbose, visualiseInfo=None, optimise=100):
    cmSize = len(thisModel[0].textLabels)
    familiarities_tmp = []
    classif_tmp = []

    label = 'unknown'

    if not thisModel[0].classifiers:
        calibrateMultipleModelRecall(thisModel)

    for j in range(cmSize):
        tempTest = testInstance - thisModel[j + 1].Ymean
        tempTest /= thisModel[j + 1].Ystd
        yy_test = thisModel[j + 1].SAMObject.familiarity(tempTest, optimise=optimise)[:, None]
        # yy_test *= thisModel[j+1].Ystd
        # yy_test += thisModel[j+1].Ymean
        cc = thisModel[0].classifiers[j].predict_proba(yy_test)[:, j]
        if verbose:
            print('Familiarity with ' + thisModel[j + 1].modelLabel + ' given current instance is: ' + str(yy_test) +
                  ' ' + str(cc[0]))
        familiarities_tmp.append(yy_test)
        classif_tmp.append(cc)

    bestConfidence = np.argmax(classif_tmp)

    for j in range(cmSize):
        if thisModel[0].classif_thresh[j][0] <= classif_tmp[j] <= thisModel[0].classif_thresh[j][1]:
            bestConfidence = j
            label = thisModel[0].textLabels[j]

            # print 'min, classifier, max = ' + str(thisModel[0].classif_thresh[j][0]) + ' ' + str(classif_tmp[j]) + ' ' + \
            #       str(thisModel[0].classif_thresh[j][1])

    # if visualiseInfo:
    #     pass

    return [label, classif_tmp[bestConfidence][0]]


def plot_confusion_matrix(cm, targetNames, title='Confusion matrix', cmap=plt.cm.inferno):
    plt.figure()
    plt.imshow(cm, interpolation='nearest', cmap=cmap)
    plt.title(title)
    plt.colorbar()
    tick_marks = np.arange(len(targetNames))
    plt.xticks(tick_marks, targetNames, rotation=45)
    plt.yticks(tick_marks, targetNames)
    plt.tight_layout()
    plt.ylabel('True label')
    plt.xlabel('Predicted label')


def wait_watching_stdout(ar, dt=1, truncate=1000):
    while not ar.ready():
        stdouts = ar.stdout
        if any(stdouts):
            clear_output()
            print '-' * 30
            print "%.3fs elapsed" % ar.elapsed
            print ""
            for stdout in ar.stdout:
                if stdout:
                    print "\n%s" % (stdout[-truncate:])
            sys.stdout.flush()
        time.sleep(dt)


def testSegment(thisModel, Ysample, verbose, visualiseInfo=None, optimise=100):
    if len(thisModel) > 1:
        d = multipleRecall(thisModel, Ysample, verbose, visualiseInfo, optimise=optimise)
    else:
        d = singleRecall(thisModel[0], Ysample, verbose, visualiseInfo, optimise=optimise)
    return d


def segmentTesting(thisModel, Ysample, Lnum, verbose, label, serialMode=False, optimise=100, calibrate=False):
    def testFunc(data, lab):
        d = testSegment(thisModel, data, verbose, visualiseInfo=None, optimise=optimise)
        if verbose:
            if lab == d[0]:
                res = True
            else:
                res = False
            print 'Actual  ' + str(lab).ljust(11) + '  Classification:  ' + str(d[0]).ljust(11) + '  with ' + \
                  str(d[1])[:6] + ' confidence: ' + str(res) + '\n'
        return d

    print

    if type(Lnum).__module__ == np.__name__:
        useModelLabels = True
    else:
        useModelLabels = False

    if len(thisModel) > 1:
        labelList = copy.deepcopy(thisModel[0].textLabels)
        labelList.append('unknown')
    else:
        labelList = copy.deepcopy(thisModel[0].textLabels)
        labelList.append('unknown')

    confMatrix = np.zeros((len(labelList), len(labelList)))

    numItems = len(Ysample)

    off1 = 11
    off2 = 8
    off3 = len(str(numItems))
    if useModelLabels:
        Lsample = [thisModel[0].textLabels[int(Lnum[i])] for i in range(len(Lnum))]
    else:
        Lsample = Lnum

    if numItems < 1500:
        serialMode = True
    c = None
    print 'serialMode', serialMode
    if not serialMode and thisModel[0].parallelOperation:
        try:
            print 'Trying engines ...'
            c = ipp.Client()
            numWorkers = len(c._engines)
            print 'Number of engines:', numWorkers
        except:
            print "Parallel workers not found"
            thisModel[0].parallelOperation = False
            numWorkers = 1
    else:
        print serialMode, '= True'
        thisModel[0].parallelOperation = False
        numWorkers = 1
        print 'Number of engines:', numWorkers

    # average 5 classifications before providing this time
    vTemp = copy.deepcopy(verbose)
    verbose = False
    if len(Lsample) < 400:
        numTrials = len(Lsample)*0.1
        numTrials = int(numTrials)
    else:
        numTrials = 20
    t0 = time.time()
    for j in range(numTrials):
        testFunc(Ysample[j], Lsample[j])
    t1 = time.time()
    verbose = vTemp
    thisModel[0].avgClassTime = (t1 - t0) / numTrials
    print 'classification rate:', 1.0 / thisModel[0].avgClassTime, 'fps'
    print 'estimated time: ' + str(thisModel[0].avgClassTime * numItems / (60*numWorkers)) + 'mins for ' + str(numItems) + ' items with ' + str(numWorkers) + 'workers' 
    t0 = time.time()
    print t0
    # check size of model
    # modelSize is size in megabytes
    modelSize = deep_getsizeof(thisModel, set()) / 1024.0 / 1024.0
    print "modelSize: ", modelSize
    print "required testing size: ", (modelSize * numWorkers * 2) + 400, " MB"
    # check available system memory in megabytes
    freeSystemMem = float(psutil.virtual_memory()[4]) / 1024.0 / 1024.0
    print "free memory:", freeSystemMem, " MB"

    if modelSize > 100 or not thisModel[0].parallelOperation or serialMode:
        # serial testing
        print 'Testing serially'
        ret = []
        for j in range(len(Lsample)):
            print j, '/', len(Lsample)
            ret.append(testFunc(Ysample[j], Lsample[j]))
    else:
        # parallel testing
        print 'Testing in parallel'
        dview = c[:]  # not load balanced
        lb = c.load_balanced_view()  # load balanced

        # with dview.sync_imports():
        #     from SAM.SAM_Core import utils
        # if not thisModel[0].modelLoaded :
        dview.push({'thisModel': thisModel})
        dview.push({'verbose': verbose})
        dview.push({'optimise': optimise})
        # thisModel[0].modelLoaded = True
        syn = lb.map_async(testFunc, Ysample, Lsample)
        wait_watching_stdout(syn, dt=1, truncate=1000)
        ret = syn.get()
        # maybe these are upsetting the ipcluster
        # dview.clear()
        # dview.purge_results('all')
    t1 = time.time()
    print t1
    print 'Actual time taken =', t1-t0
    if calibrate:
        variancesKnown = []
        variancesUnknown = []
        for i in range(len(ret)):
            currLabel = Lsample[i]

            if verbose:
                if currLabel == ret[i][0]:
                    result = True
                else:
                    result = False
                print str(i).rjust(off3) + '/' + str(numItems) + ' Truth: ' + currLabel.ljust(off1) + ' Model: ' + ret[
                    i][0].ljust(off1) + ' with ' + str(ret[i][1])[:6].ljust(off2) + ' confidence: ' + str(result)

            if currLabel in thisModel[0].textLabels:
                knownLabel = True
            else:
                knownLabel = False
                currLabel = 'unknown'

            if knownLabel:
                variancesKnown.append(ret[i][1])
            else:
                variancesUnknown.append(ret[i][1])

            confMatrix[labelList.index(currLabel), labelList.index(ret[i][0])] += 1

        return labelList, confMatrix, ret, variancesKnown, variancesUnknown
    else:
        for i in range(len(ret)):
            currLabel = Lsample[i]
            retLabel = ret[i][0]

            if currLabel not in thisModel[0].textLabels:
                currLabel = 'unknown'

            if verbose:
                if currLabel == retLabel:
                    result = True
                else:
                    result = False
                print str(i).rjust(off3) + '/' + str(numItems) + ' Truth: ' + currLabel.ljust(off1) + ' Model: ' + \
                      retLabel.ljust(off1) + ' with ' + str(ret[i][1])[:6].ljust(off2) + \
                      ' confidence: ' + str(result)

            confMatrix[labelList.index(currLabel), labelList.index(retLabel)] += 1
        return labelList, confMatrix


def testSegments(thisModel, Ysample, Lnum, verbose, label, serialMode=False):
    labelList, confMatrix = segmentTesting(thisModel, Ysample, Lnum, verbose, label, serialMode=serialMode,
                                           optimise=thisModel[0].optimiseRecall, calibrate=False)

    dCalc = calculateData(labelList, confMatrix)

    return [dCalc[0], dCalc[1]]


def calculateVarianceThreshold(segIntersections, mk, muk, vk, vuk):
    thresh = None
    direction = None
    if len(segIntersections) == 0:
        # either gaussians exactly equal to each other(worst) or no overlap(best)
        if mk == muk:
            # gaussians on top of each other .. can only happen with 0 intersections when mean is identical
            # and var is identical
            thresh = [mk]
            direction = ['smaller']
        else:
            # gaussians completely separated, threshold set equidistant to means
            thresh = [(max(mk, muk) - min(mk, muk)) / 2 + min(mk, muk)]
            if thresh[0] > mk:
                direction = ['smaller']
    elif len(segIntersections) == 1:
        thresh = [segIntersections]
        if thresh[0] > mk:
            direction = ['smaller']
            # set threshold at this point
    elif len(segIntersections) == 2:
        if mk == muk:
            # set upper and lower bounds on threshold
            thresh = [min(segIntersections), max(segIntersections)]
            # works
            if vk > vuk:
                direction = ['smaller', 'greater']
            else:
                direction = ['greater', 'smaller']
        else:
            thresh = [np.ptp(segIntersections) / 2 + min(segIntersections)]
            if thresh[0] < muk:
                direction = ['smaller']
                # set threshold equidistant from intersection

    if direction is None:
        direction = ['greater']

    return [thresh, direction]


def calculateData(textLabels, confMatrix, numItems=None):
    print confMatrix
    if not numItems:
        numItems = np.sum(confMatrix)

    h = confMatrix
    total = h.astype(np.float).sum(axis=1)
    normConf = copy.deepcopy(h)
    normConf = normConf.astype(np.float)

    for l in range(h.shape[0]):
        if total[l] != 0:
            normConf[l, :] = normConf[l, :].astype(np.float) * 100 / total[l].astype(np.float)

    print normConf

    # if plotting
    # confMatLabels = copy.deepcopy(textLabels)
    # plot_confusion_matrix(normConf, confMatLabels)

    # percCorect = 100 * np.diag(h.astype(np.float)).sum() / numItems
    percCorect = 100 * np.diag(normConf.astype(np.float)).sum() / np.sum(normConf)

    print str(percCorect)[:5].ljust(7) + "% correct for training data"
    print
    for i in range(confMatrix.shape[0]):
        for j in range(confMatrix.shape[0]):
            print str(normConf[i, j])[:5].ljust(7) + '% of ' + str(textLabels[i]) + \
                  ' classified as ' + str(textLabels[j])
        print
    return [normConf, percCorect]


def combineClassifications(thisModel, labels, likelihoods):
    if len(thisModel) > 1:
        labelList = copy.deepcopy(thisModel[0].textLabels)
        labelList.append('unknown')
    else:
        labelList = copy.deepcopy(thisModel[0].textLabels)

    sumLikelihoods = [None] * (len(labelList))
    counts = [0] * (len(labelList))

    for i in range(len(labels)):
        idx = [j for j, k in enumerate(labelList) if k == labels[i]][0]
        counts[idx] += 1
        if sumLikelihoods[idx] is None:
            sumLikelihoods[idx] = likelihoods[i]
        else:
            sumLikelihoods[idx] += likelihoods[i]

    m = max(sumLikelihoods)
    maxIdx = [j for j, k in enumerate(sumLikelihoods) if k == m][0]
    return [labelList[maxIdx], m / counts[maxIdx]]
