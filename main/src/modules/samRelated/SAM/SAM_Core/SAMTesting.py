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
from SAM.SAM_Core import SAM_utils
import ipyparallel as ipp
import time
import matplotlib.pyplot as plt
import sys
import operator
import copy
import psutil
from scipy.spatial import distance
import numpy as np
from collections import Mapping, Container
from sys import getsizeof
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
            y_valid_tmp, y_test_tmp, _, _ = SAM_utils.random_data_split(yy_test, [0.5, 0.5])
            Y_valid.append(y_valid_tmp.copy())
            Y_testing.append(y_test_tmp.copy())

    # Compute familiarities in VALIDATION SET
    familiarities = [None] * (len(thisModel) - 1)
    for i in range(len(thisModel)):
        if thisModel[i].SAMObject.model:
            # N_test x N_labels matrix.
            familiarities[i-1] = np.zeros((Y_valid[i-1].shape[0], (len(thisModel) - 1)))
            print("## True label is " + thisModel[i].modelLabel)
            for k in range(Y_valid[i-1].shape[0]):
                sstest = []
                print('# k=' + str(k))
                for j in range(len(thisModel)):
                    if thisModel[j].SAMObject.model:
                        yy_test = Y_valid[i-1][k, :][None, :].copy()
                        # Normalize according to the model to predict
                        yy_test -= thisModel[j].Ymean
                        yy_test /= thisModel[j].Ystd
                        sstest.append(thisModel[j].SAMObject.familiarity(yy_test))
                        familiarities[i-1][k, j-1] = sstest[-1]
                for j in range(len(sstest)):
                    if j == np.argmax(sstest):
                        print '   *',
                    else:
                        print '    ',
                    print('      Familiarity of model ' + thisModel[j+1].modelLabel + ' given label: ' +
                          thisModel[i].modelLabel + ' in valid: ' + str(sstest[j]))

                confMatrix[i-1, np.argmax(sstest)] += 1
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


def formatDataFunc(Ydata):
    yDataList = []
    for j in range(Ydata.shape[0]):
        yDataList.append(Ydata[j][None, :])
    return yDataList


def singleRecall(thisModel, testInstance, verbose, visualiseInfo=None):
    # Returns the predictive mean, the predictive variance and the axis (pp) of the latent space backwards mapping.
    # mm,vv,pp=self.SAMObject.pattern_completion(testFace, visualiseInfo=visualiseInfo)
    # if verbose:
        # print 'single model recall'
    textStringOut = ''
    # normalize incoming data
    testValue = testInstance - thisModel.Ymean
    testValue /= thisModel.Ystd

    ret = thisModel.SAMObject.pattern_completion(testValue, visualiseInfo=visualiseInfo)

    mm = ret[0]
    vv = ret[1]
    # post = ret[3]

    # find nearest neighbour of mm and SAMObject.model.X
    dists = np.zeros((thisModel.SAMObject.model.X.shape[0], 1))

    for j in range(dists.shape[0]):
        dists[j, :] = distance.euclidean(thisModel.SAMObject.model.X.mean[j, :], mm[0].values)
    nn, min_value = min(enumerate(dists), key=operator.itemgetter(1))
    if thisModel.SAMObject.type == 'mrd':
        if verbose:
            print "With " + str(vv.mean()) + " prob. error the new instance is " \
                  + thisModel.textLabels[int(thisModel.SAMObject.model.bgplvms[1].Y[nn, :])]

        textStringOut = thisModel.textLabels[int(thisModel.SAMObject.model.bgplvms[1].Y[nn, :])]

    elif thisModel.SAMObject.type == 'bgplvm':
        if verbose:
            print "With " + str(vv.mean()) + " prob. error the new instance is " \
                  + thisModel.textLabels[int(thisModel.L[nn, :])]
        textStringOut = thisModel.textLabels[int(thisModel.L[nn, :])]

    # if(vv.mean()<0.00012):
    #     print "The action is " + textStringOut
    # elif(vv.mean()>0.00012):
    #     print "I think the action is " + textStringOut + " but I am not sure"

    # # Plot the training NN of the test image (the NN is found in the INTERNAl, compressed (latent) memory space!!!)
    # if visualiseInfo is not None:
    #     fig_nn = visualiseInfo['fig_nn']
    #     fig_nn = pb.figure(11)
    #     pb.title('Training NN')
    #     fig_nn.clf()
    #     pl_nn = fig_nn.add_subplot(111)
    #     pl_nn.imshow(numpy.reshape(self.SAMObject.recall(nn),(self.imgHeightNew, self.imgWidthNew)),
    #     cmap=plt.cm.Greys_r)
    #     pb.title('Training NN')
    #     pb.show()
    #     pb.draw()
    #     pb.waitforbuttonpress(0.1)
    # return pp
    # return ['push_object', np.array([1.24])]
    return [textStringOut, vv.mean()]


def multipleRecall_noCalib(thisModel, testInstance, verbose, visualiseInfo=None):
    result = []
    if verbose:
        pass
    #     print 'multiple model recall'

    for j in thisModel:
        if j.SAMObject.model:
            tempTest = testInstance - j.Ymean
            tempTest /= j.Ystd
            yy_test = j.SAMObject.familiarity(tempTest)
            if verbose:
                print('Familiarity with ' + j.modelLabel + ' given current instance is: ' + str(yy_test))
            # yy_test -= thisModel[j].Ymean
            # yy_test /= thisModel[j].Ystd
            result.append(yy_test)
    maxIdx = np.argmax(result)

    if visualiseInfo:
        pass

    return [thisModel[0].textLabels[maxIdx - 1], result[maxIdx][0]]


def multipleRecall(thisModel, testInstance, verbose, visualiseInfo=None):

    cmSize = len(thisModel[0].textLabels)
    familiarities_tmp = []
    classif_tmp = []

    label = 'unknown'

    # if verbose:
    #     pass
    #     print 'multiple model recall'

    if not thisModel[0].classifiers:
        calibrateMultipleModelRecall(thisModel)

    for j in range(cmSize):
        tempTest = testInstance - thisModel[j+1].Ymean
        tempTest /= thisModel[j+1].Ystd
        yy_test = thisModel[j+1].SAMObject.familiarity(tempTest)[:, None]
        # yy_test -= thisModel[j].Ymean
        # yy_test /= thisModel[j].Ystd
        cc = thisModel[0].classifiers[j].predict_proba(yy_test)[:, j]
        if verbose:
            print('Familiarity with ' + thisModel[j+1].modelLabel + ' given current instance is: ' + str(cc[0]))
        familiarities_tmp.append(yy_test)
        classif_tmp.append(cc)

    bestConfidence = np.argmax(classif_tmp)

    for j in range(cmSize):
        if thisModel[0].classif_thresh[j][0] <= classif_tmp[j] <= thisModel[0].classif_thresh[j][1]:
            label = thisModel[0].textLabels[j]

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


def testSegment(thisModel, Ysample, verbose, visualiseInfo=None):
    if len(thisModel) > 1:
        d = multipleRecall(thisModel, Ysample, verbose, visualiseInfo)
    else:
        d = singleRecall(thisModel[0], Ysample, verbose, visualiseInfo)
    return d


def testSegments(thisModel, Ysample, Lnum, verbose):

    def testFunc(data, lab):
        d = testSegment(thisModel, data, verbose)
        if verbose:
            if lab == d[0]:
                res = True
            else:
                res = False
            print 'Actual  ' + str(lab).ljust(11) + '  Classification:  ' + str(d[0]).ljust(11) + '  with ' + \
                  str(d[1])[:6] + ' confidence: ' + str(res) + '\n'
        return d

    print
    cmSize = len(thisModel[0].textLabels)
    if len(thisModel) > 1:
        confMatrix = np.zeros((cmSize+1, cmSize+1))
        labelList = copy.deepcopy(thisModel[0].textLabels)
        labelList.append('unknown')
    else:
        confMatrix = np.zeros((cmSize, cmSize))
        labelList = copy.deepcopy(thisModel[0].textLabels)

    numItems = len(Ysample)

    off1 = 11
    off2 = 8
    off3 = len(str(numItems))
    Lsample = [thisModel[0].textLabels[int(Lnum[i])] for i in range(len(Lnum))]

    print 'estimated time: ' + str(numItems/60) + 'mins for ' + str(numItems) + ' items'

    parallelOperation = True
    c = None
    try:
        c = ipp.Client()
        numWorkers = len(c._engines)
    except:
        print "Parallel workers not found"
        parallelOperation = False
        numWorkers = 1

    # check size of model
    # modelSize is size in megabytes
    modelSize = deep_getsizeof(thisModel, set()) / 1024.0 / 1024.0
    print "modelSize: ", modelSize
    print "required testing size: ", (modelSize*numWorkers*2)+400, " MB"
    # check available system memory in megabytes
    freeSystemMem = float(psutil.virtual_memory()[4]) / 1024.0 / 1024.0
    print "free memory:", freeSystemMem, " MB"

    if (modelSize*numWorkers*2) > 250 or not parallelOperation:
        # serial testing
        print 'Testing serially'
        ret = []
        for j in range(len(Lsample)):
            ret.append(testFunc(Ysample[j], Lsample[j]))
    else:
        # parallel testing
        print 'Testing in parallel'
        dview = c[:]

        # with dview.sync_imports():
        #     from SAM.SAM_Core import SAM_utils

        dview.push({'thisModel': thisModel})
        dview.push({'verbose': verbose})

        syn = dview.map_async(testFunc, Ysample, Lsample)
        wait_watching_stdout(syn, dt=1, truncate=1000)
        ret = syn.get()

    for i in range(len(ret)):
        currLabel = Lsample[i]

        if verbose:
            if currLabel == ret[i][0]:
                result = True
            else:
                result = False
            print str(i).rjust(off3) + '/' + str(numItems) + ' Truth: ' + currLabel.ljust(off1) + ' Model: ' + ret[
                i][0].ljust(off1) + ' with ' + str(ret[i][1])[:6].ljust(off2) + ' confidence: ' + str(result)

        confMatrix[labelList.index(currLabel), labelList.index(ret[i][0])] += 1

    return calculateData(labelList, confMatrix)


def calculateData(textLabels, confMatrix, numItems=None):
    if not numItems:
        numItems = np.sum(confMatrix)

    h = confMatrix
    total = h.astype(np.float).sum(axis=1)
    normConf = copy.deepcopy(h)
    normConf = normConf.astype(np.float)

    for l in range(h.shape[0]):
        if total[l] != 0:
            normConf[l, :] = normConf[l, :].astype(np.float)*100/total[l].astype(np.float)

    print normConf

    # if plotting
    # confMatLabels = copy.deepcopy(textLabels)
    # plot_confusion_matrix(normConf, confMatLabels)

    percCorect = 100*np.diag(h.astype(np.float)).sum()/numItems

    print str(percCorect)[:5].ljust(7) + "% correct for training data"
    print
    for i in range(confMatrix.shape[0]):
        for j in range(confMatrix.shape[0]):
            print str(normConf[i, j])[:5].ljust(7) + '% of ' + str(textLabels[i]) +\
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
    return [labelList[maxIdx], m/counts[maxIdx]]
