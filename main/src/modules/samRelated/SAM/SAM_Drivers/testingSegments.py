
from sklearn.metrics import confusion_matrix
from IPython.display import clear_output
import ipyparallel as ipp
import time
import matplotlib
import matplotlib.pyplot as plt
import sys 
import GPy
import copy
from os import listdir, walk, system
from os.path import isfile, join, isdir
from scipy.spatial import distance
import numpy as np

thisModel = None

def plot_confusion_matrix(cm, targetNames, title='Confusion matrix', cmap=plt.cm.inferno):
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

def testSegments(thisModel, Ysample, Lnum):

    def testFunc(data, lab):
        d = thisModel.testing(data, False)
        if(lab == d[0]):
            result = True
        else:
            result = False
        print 'Actual  ' + str(lab).ljust(11) + '  Model:  ' + str(d[0]).ljust(11) + '  with ' + str(d[1])[:6] + ' confidence: ' + str(result) + '\n'
        return d


    c = ipp.Client()
    dview = c[:]

    with dview.sync_imports():
        import SAM.SAM_Drivers 
    #always keep mySAMpy as label
    dview.push({'thisModel':thisModel})

    ss = []
    print
    off1 = 11
    off2 = 8

    cmSize = len(thisModel.textLabels)
    confMatrix = np.zeros((cmSize, cmSize))

    numItems = len(Ysample)

    off3 = len(str(numItems))
    Lsample = [thisModel.textLabels[int(Lnum[i])] for i in range(len(Lnum))]

    print 'estimated time: ' + str(numItems/60) + 'mins for ' + str(numItems) + ' items'
    #format training data

    syn = dview.map_async(testFunc, Ysample, Lsample)
    wait_watching_stdout(syn, dt=1, truncate=1000)
    ret = syn.get()
    # # clear_output()
    for i in range(len(ret)):

        currLabel = Lsample[i]

        if(currLabel == ret[i][0]):
            result = True
        else:
            result = False
        print str(i).rjust(off3) + '/' + str(numItems) + ' Truth: ' + currLabel.ljust(off1) + ' Model: ' + ret[i][0].ljust(off1) + ' with ' + str(1-ret[i][1])[:6].ljust(off2) + ' confidence: ' + str(result)
        confMatrix[thisModel.textLabels.index(currLabel),thisModel.textLabels.index(ret[i][0])] += 1
        ss.append(ret[i][0])

    confMatLabels = copy.deepcopy(thisModel.textLabels)
    #confMatLabels.sort()

    h = confusion_matrix(Lsample, ss)
    total = h.astype(np.float).sum(axis=1)
    normConf = copy.deepcopy(h)
    normConf = normConf.astype(np.float)

    for l in range(h.shape[0]):
        normConf[l,:] = normConf[l,:].astype(np.float)*100/total[l].astype(np.float)

    print normConf

    plot_confusion_matrix(normConf, confMatLabels)

    percCorect = 100*np.diag(h.astype(np.float)).sum()/numItems

    print str(percCorect)[:5].ljust(7) + "% correct for training data"
    print
    for i in range(cmSize):
        for j in range(cmSize):
            print str(normConf[i,j])[:5].ljust(7) + '% of ' + str(thisModel.textLabels[i]) + ' classified as ' + str(thisModel.textLabels[j])
        print

# def testSequence(thisModel):
#         