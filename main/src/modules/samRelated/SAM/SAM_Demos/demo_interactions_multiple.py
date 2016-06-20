#!/usr/bin/python

#
#The University of Sheffield
#WYSIWYD Project
#
#Example of implementation of SAMpy class for multiple streams
#
#Created on 2016
#
#@authors: Andreas Damianou
#
#

import matplotlib.pyplot as plt
from SAM.SAM_Drivers import SAMDriver_interaction
from SAM.SAM_Core import SAM_utils
import pylab as pb
import sys
import pickle
import os
import numpy
import time
import operator
import numpy as np
try:
    import yarp
    yarpRunning = True
except ImportError:
    print 'WARNING! Yarp was not found! Switching to offline mode'
    yarpRunning = False

from ConfigParser import SafeConfigParser


# Check configuration file
parser = SafeConfigParser()

file_candidates = ["config_faces.ini"]
section_candidates = ["config_options"]

configData = False

print 'Finding config file'
print '-------------------'
for loc in os.curdir, os.path.expanduser("~"), os.environ.get("WYSIWYD_DIR")+"/share/wysiwyd/contexts/visionDriver":
    print loc
    try:
        found = parser.read(os.path.join(loc,file_candidates[0]))
        if not found:
            pass
        else:
            pathFound = found
            print os.path.join(loc,file_candidates[0])
            if( parser.has_section(section_candidates[0]) == True ):
                dataPath = parser.get(section_candidates[0], 'data_path')
                modelPath = parser.get(section_candidates[0], 'model_path')
                participantList_val = parser.get(section_candidates[0], 'participants')
                participantList = participantList_val.split(',')
                configData = True
            else:
                print 'config_options not found...'
    except IOError:
        pass

if( configData == True ):
    print "config paths ready"
else:
    print "config paths failed"
    exit()

print '-------------------'
print 'Config file found: ' + pathFound[0]
print dataPath
print modelPath
print participantList
print '-------------------'


# Creates and opens ports for interaction with speech module
if yarpRunning:
    yarp.Network.init()
    inputInteractionPort = yarp.Port()
    inputInteractionPort.open("/sam/face/rpc:i");

    inputBottle = yarp.Bottle();
    outputBottle = yarp.Bottle();

imgHNew = 200
imgWNew = 200

# Creates a SAMpy object
mySAMpy = SAMDriver_interaction(True, imgH = 400, imgW = 400, imgHNew = imgHNew, imgWNew = imgWNew,inputImagePort="/CLM/imageSeg/out")

# Specification of the experiment number
experiment_number = 1009#1008#1007#42

# Location of face data
root_data_dir=dataPath

# Image format
image_suffix=".ppm"
# Array of participants to be recognised
participant_index=participantList

# Poses used during the data collection
pose_index=['Seg']
# Use a subset of the data for training
Ntr=300

# Pose selected for training
pose_selection = 0

# Specification of model type and training parameters
model_type = 'mrd'
model_num_inducing = 40
model_num_iterations = 6000
model_init_iterations = 2000
fname = modelPath + '/models/' + 'mActions_' + model_type + '_exp' + str(experiment_number) #+ '.pickle'

# Enable to save the model and visualise GP nearest neighbour matching
save_model=True
economy_save = True # ATTENTION!! This is still BETA!!
visualise_output=False
test_mode = False

# Reading face data, preparation of data and training of the model
mySAMpy.readData(root_data_dir, participant_index, pose_index)
(Yall, Lall, YtestAll, LtestAll) = mySAMpy.prepareData(model_type, Ntr, pose_selection, randSeed=experiment_number)

Lunique = numpy.unique(mySAMpy.L)
# L_index = dict()
# for i in range(len(Lunique)):
#     L_index[Lunique[i]] = numpy.where(mySAMpy.L == i)[0]




# Make all images to have values between 0 and 255
for i in range(Yall.shape[0]):
    Yall[i,:] = (Yall[i,:]-Yall[i,:].min())*255/(Yall[i,:].max()-Yall[i,:].min())

for i in range(YtestAll.shape[0]):
    YtestAll[i,:] = (YtestAll[i,:]-YtestAll[i,:].min())*255/(YtestAll[i,:].max()-YtestAll[i,:].min())




###### Extract SURF and use them as new features 

useSURF=False
if useSURF:
    # Ignore descriptors landing outside the following region (since faces are centered, we want to ignore background)
    crop_thresholds=(40,160,40,160)
    magnify=1
    SURF = SAM_utils.SURFProcessor(imgHNew, imgWNew, n_clusters=40,SURFthresh=500,crop_thresholds=tuple(magnify*x for x in crop_thresholds),magnify=magnify)

    (YallOrig, YtestAllOrig) = (Yall.copy(), YtestAll.copy())
    Yall = SURF.make_SURF_BoW(Yall, normalize=True)[0]
    YtestAll = SURF.make_SURF_BoW_test(YtestAll)

    # To visualize:
    #plt.matshow(YtestAll*SURF.Zstd+SURF.Zmean, aspect='auto')








# #### DEBUG
# Yall=np.hstack((Lall,Lall));Yall=Yall+np.random.randn(Yall.shape[0],Yall.shape[1])*0.05
# YtestAll=np.hstack((LtestAll,LtestAll));YtestAll=YtestAll+np.random.randn(YtestAll.shape[0],YtestAll.shape[1])*0.05
# ######




mm = []
for i in range(len(Lunique)):
    print('# Considering label: ' + str(Lunique[i]))
    cur = SAMDriver_interaction(True, imgH = 400, imgW = 400, imgHNew = imgHNew, imgWNew = imgWNew,inputImagePort="/CLM/imageSeg/out", openPorts=False)
    cur.Quser = 4
    ##############
    Y_cur = Yall[np.where(Lall==Lunique[i])[0],:].copy()
    Ytest_cur = YtestAll[np.where(LtestAll==Lunique[i])[0],:].copy()

    # Center data to zero mean and 1 std
    Ymean_cur = Y_cur.mean()
    Yn_cur = Y_cur - Ymean_cur
    Ystd_cur = Yn_cur.std()
    Yn_cur /= Ystd_cur
    # Normalise test data similarly to training data
    Ytestn_cur = Ytest_cur - Ymean_cur
    Ytestn_cur /= Ystd_cur

    cur.Ymean = Ymean_cur
    cur.Ystd = Ystd_cur


    cur.X=None
    cur.Y = {'Y':Yn_cur}
    cur.Ytestn = {'Ytest':Ytestn_cur}

    fname_cur = fname + '_L' + str(i)
    cur.training(model_num_inducing, model_num_iterations, model_init_iterations, fname_cur, save_model, economy_save)
    mm.append(cur)

ss = []
sstest = []

# for i in range(len(mm)):
#     mm[i].SAMObject.familiarity_reverse(Ytest=None, source_view=0)



### Old test##
if 0:
    for i in range(len(Lunique)):
        for j in range(len(Lunique)):
            ss = mm[i].SAMObject.familiarity(mm[j].Y['Y'])
            print('Familiarity of model ' + participantList[i] + ' given label: ' + participantList[j] + ' using training data is: ' + str(ss))
        print("")

    print("")
    print("")

    for i in range(len(Lunique)):
        for j in range(len(Lunique)):
            sstest = mm[i].SAMObject.familiarity(mm[j].Ytestn['Ytest'])
            print('i='+str(i)+': Familiarity of model ' + participantList[i] + ' given label: ' + participantList[j] + ' using testing data is: ' + str(sstest))
        print("")

### New test ##
    for i in range(len(participantList)):
        print("## True label is " + str(participantList[i]))
        for k in range(mm[i].Y['Y'].shape[0]):
            sstest = []
            print('# k='+str(k))
            for j in range(len(participantList)):
                yy_test = mm[j].SAMObject.familiarity(mm[i].Y['Y'][k,:][None,:])
                # De-normalize from the model which stored this test data
                # yy_test *= mm[i].Ystd
                # yy_test += mm[i].Ymean
                # Normalize according to the model to predict
                yy_test -= mm[j].Ymean
                yy_test /= mm[j].Ystd
                sstest.append(yy_test)
            for j in range(len(participantList)): 
                if j == np.argmax(sstest):
                    print '   *',
                else:
                    print '    ',
                print('Familiarity of model ' + participantList[j] + ' given label: ' + participantList[i] + ' in train: ' + str(sstest[j]))
        print('')
        print('')





### Create Validation set
Y_valid = []
Y_testing = []
for i in range(len(participantList)):
    # De-normalize from the model which stored this test data
    yy_test = mm[i].Ytestn['Ytest'].copy()
    yy_test *= mm[i].Ystd
    yy_test += mm[i].Ymean
    y_valid_tmp, y_test_tmp, _, _ = SAM_utils.random_data_split(yy_test, [0.5,0.5])
    ###TMP
    y_valid_tmp = y_valid_tmp[::5,:]
    y_test_tmp = y_test_tmp[::5,:]
    ###
    Y_valid.append(y_valid_tmp.copy())
    Y_testing.append(y_test_tmp.copy())

# Compute familiarities in VALIDATION SET
familiarities = [None]*len(participantList)
for i in range(len(participantList)):
    # N_test x N_labels matrix.
    familiarities[i] = np.zeros((Y_valid[i].shape[0],len(participantList)))
    print("## True label is " + str(participantList[i]))
    for k in range(Y_valid[i].shape[0]):
        sstest = []
        print('# k='+str(k))
        for j in range(len(participantList)):
            # yy_test = mm[i].Ytestn['Ytest'][k,:][None,:]
            # # De-normalize from the model which stored this test data
            # yy_test *= mm[i].Ystd
            # yy_test += mm[i].Ymean
            yy_test = Y_valid[i][k,:][None,:].copy()
            # Normalize according to the model to predict
            yy_test -= mm[j].Ymean
            yy_test /= mm[j].Ystd
            sstest.append(mm[j].SAMObject.familiarity(yy_test))
            familiarities[i][k,j] = sstest[-1]
        for j in range(len(participantList)): 
            if j == np.argmax(sstest):
                print '   *',
            else:
                print '    ',
            print('      Familiarity of model ' + participantList[j] + ' given label: ' + participantList[i] + ' in valid: ' + str(sstest[j]))

# At this point we have:
# familiarities[i][k,j] -> familiarity for true label i, instance k
#                          predicted by model trained in label j
############# Train Familiarity classifier in VALIDATION SET
from sklearn.mixture import GMM

classifiers = []
classif_thresh = []
familiarity_predictions = []
tmp = []
for i in range(len(participantList)):
    X_train = familiarities[0][:,i][:,None]
    y_train = np.zeros((familiarities[0][:,i][:,None].shape[0],1))
    for j in range(1,len(participantList)):
        X_train = np.vstack((X_train, familiarities[j][:,i][:,None]))
        y_train = np.vstack((y_train, j+np.zeros((familiarities[j][:,i][:,None].shape[0],1))))
    tmp.append(X_train)
    n_classes = len(np.unique(y_train))

    # Try GMMs using different types of covariances.
    classifiers.append(GMM(n_components=n_classes,covariance_type='full', init_params='wc', n_iter=2000))

    # Since we have class labels for the training data, we can
    # initialize the GMM parameters in a supervised manner.
    classifiers[-1].means_ = np.array([X_train[y_train == kk].mean(axis=0)
                              for kk in xrange(n_classes)])[:,None]
    classifiers[-1].fit(X_train)
    familiarity_predictions.append(classifiers[-1].predict(X_train))

    # Find threshold of confident classification of model i predicting label i
    tmp_i = classifiers[i].predict_proba(X_train[y_train==i][:,None])[:,i]
    tmp_s = 0.5
    # If in the test phase we get a predict_proba which falls in the threshold i, then 
    # model i is confident for this prediction.
    classif_thresh.append([tmp_i.mean()-tmp_s*tmp_i.std(), tmp_i.mean()+tmp_s*tmp_i.std()])

    # !!!!! For each model i, given y_test, test if:
    # classifiers[i].predict_proba(m[i].SAMObject.familiarity(y_test))[:,i]
    # is within classif_thresh[i]. If yes, then y_test is of class i with
    # high confidence.


# How well does model i predict label i? eg i=2:    
#familiarity_predictions[2][np.where(y_train==2)[0]]



#### Now do the actual classification of TEST familiarities
for i in range(len(participantList)):
    print('#### True label:' + str(i))
    for k in range(Y_testing[i].shape[0]):
        familiarities_tmp = []
        classif_tmp = []
        print('# k='+str(k))
        for j in range(len(participantList)):
            yy_test = Y_testing[i][k,:][None,:].copy() #### ORIG
            # yy_test = Y_valid[i][k,:][None,:].copy() ### COMMENT
            # Normalize according to the model to predict
            yy_test -= mm[j].Ymean
            yy_test /= mm[j].Ystd
            tmp = mm[j].SAMObject.familiarity(yy_test)[:,None]
            cc = classifiers[j].predict_proba(tmp)[:,j]
            familiarities_tmp.append(tmp)
            classif_tmp.append(cc)
        for j in range(len(participantList)): 
            if classif_tmp[j] >= classif_thresh[j][0] and \
               classif_tmp[j] <= classif_thresh[j][1]:
                print '   *',
            else:
                print '    ',
            print('      Familiarity/Classif of model ' + participantList[j] + ' given label: ' + participantList[i] + ' in test: ' + str(familiarities_tmp[j])+'/'+str(classif_tmp[j]))


######################################




STOP







###################### OLD: 
from sklearn.mixture import GMM

means=[]
covars=[]
for i in range(len(participantList)):
    gmm = GMM(n_components=1)
    gmm.fit(familiarities[i])
    means.append(gmm.means_)
    covars.append(gmm.covars_)


def my_logpdf(y, ymean, yvar):
   import numpy as np
   N = y.shape[0]
   ln_det_cov = N * np.log(yvar)
   return -0.5 * (np.sum((y - ymean) ** 2 / yvar) + ln_det_cov + N * np.log(2. * np.pi))



def gauss(y, mu, sigma):
    N = y.shape[0]
    ll = 0
    for i in range(N):
        ll += numpy.exp(-(y[i]-mu)**2/(2.*sigma**2))
    return ll/N

preds=[]
for i in range(len(participantList)):
    preds.append(gauss(familiarities[i], means[i], 0.01*covars[i]))
    # preds[i][j] = gauss(np.array([means[i][j]])[:,None], means[i][j], 0.00000000000000001*covars[i][j])
#######################










#########


#if yarpRunning:
 #   while( not(yarp.Network.isConnected("/speechInteraction/behaviour:o","/sam/face/interaction:i")) ):
 #       print "Waiting for connection with behaviour port..."
 #       pass

visualiseInfo = [];
# This is for visualising the mapping of the test face back to the internal memory
if visualise_output: 
    for i in range(len(Lunique)):
        ax = mm[i].SAMObject.visualise()
        visualiseInfo.append(dict())
        visualiseInfo[i]['ax']=ax
        ytmp = mm[i].SAMObject.recall(0)
        ytmp = numpy.reshape(ytmp,(mm[i].imgHeightNew,mm[i].imgWidthNew))
        #ytmp = numpy.reshape(ytmp,(mm[i].imgHeightNew,mm[i].imgWidthNew,3))
        fig_nn = pb.figure()
        pb.title('Training NN')
        pl_nn = fig_nn.add_subplot(111)
        ax_nn = pl_nn.imshow(ytmp, cmap=plt.cm.Greys_r)
        pb.draw()
        pb.show()
        visualiseInfo[i]['fig_nn']=fig_nn
    else:
        visualiseInfo[i]=None

# Read and test images from iCub eyes in real-time

#fig_input = pb.figure()
#subplt_input = fig_input.add_subplot(111)
numFaces = 1
minTestImages = 0
numImgs = 0
for i in range(len(participantList)):
    numImgs = mm[i].Ytestn['Ytest'].shape[0]
    if(i == 0):
        minTestImages = numImgs
    else:
        if(numImgs < minTestImages):
            minTestImages = numImgs

minTestImages = 10
result = np.zeros([len(participantList),minTestImages,len(participantList)])
responseIdx = np.zeros([result.shape[0],result.shape[1]])
responseVal = np.zeros([result.shape[0],result.shape[1]])
confusionMatrix = np.zeros([result.shape[0],result.shape[0]])

if(test_mode):
    fig_disp = pb.figure()
    pb.title('Current Testing Image')
    pl_nn = fig_disp.add_subplot(111)

    for i in range(result.shape[0]):
        print('Participant ' + str(i) + ': ' + participantList[i])
        print('')
        for j in range(result.shape[1]):
            
            for k in range(result.shape[2]):
                currTestData = mm[k].Ytestn['Ytest']
                currImage = np.reshape(currTestData[j,:],[1,currTestData[j,:].shape[0]])

                ytmp = numpy.reshape(currImage,(mm[i].imgHeightNew,mm[i].imgWidthNew))
                ax_nn = pl_nn.imshow(ytmp, cmap=plt.cm.Greys_r)
                fig_disp.canvas.draw()
                fig_disp.canvas.flush_events()

                resultTemp = mm[i].SAMObject.familiarity(currImage)
                result[i][j][k] = resultTemp

            #print(participantList[i] + ' Test Image ' + str(j) + ' familiarity =' + str(result[i][j][:]))
            maxIdx = np.argmax(result[i][j][:])
            responseIdx[i][j] = maxIdx
            responseVal[i][j] = result[i][j][maxIdx]


        for g in range(result.shape[0]):
            confusionMatrix[i][g] = 100*float(np.count_nonzero(responseIdx[i][:]==g))/float(responseIdx.shape[1])
            print('Percentage ' + participantList[i] + ' classified as ' + participantList[g] + ' = ' +  str(confusionMatrix[i][g]))
    #calculate optimal thresholds using familiarity thresholds from result matrix and confusion matrix percentages



else:
    while( True ):
        try: 
            if yarpRunning:
                print "Waiting for input"
                inputInteractionPort.read(inputBottle,True)

                print(inputBottle.get(0).asString() + 'received')

                if( inputBottle.get(0).asString() == "ask_name" ):

                    #testFace = numpy.zeros([numFaces,imgHNew*imgWNew*3])
                    testFace = numpy.zeros([numFaces,imgHNew*imgWNew])

                    for i in range(numFaces):
                        imageReceived = False
                        imageReceived = mySAMpy.readImageFromCamera()
                        if(imageReceived):
                            testFace[i,:] = mySAMpy.imageFlatten_testing
                        print "face" + str(i)
                    
                    #pp = mySAMpy.testing(testFace, choice, visualiseInfo)
                    ss=numpy.zeros(len(participantList))
                    for i in range(len(Lunique)):
                        testFacen = testFace;
                        #testFacen = testFace - testFace.mean()
                        #testFacen /= testFace.std()
                        #testFacen = testFace - mm[i].Ymean
                        #testFacen /= mm[i].Ystd
                        ss[i] = mm[i].SAMObject.familiarity(testFacen)
                        print('Familiarity with ' + participantList[i] + ' given current face is: ' + str(ss[i]))

                    outputBottle.clear()
                    #deciding response
                    maxIdx = np.argmax(ss)
                    maxVal = ss[maxIdx]
                    
                    threshold = 0.4
                    if(maxVal > threshold):
                        outputBottle.addString(participantList[maxIdx])
                        #outputBottle.addString("greg")
                    else:
                        #outputBottle.addString("Unknown")
                        outputBottle.addString("partner")
                    
                    print(outputBottle.get(0).asString())
                    #time.sleep(0.5)
                    #l = pp.pop()
                    #l.remove()
                    #pb.draw()
                    #pb.waitforbuttonpress(0.1)
                else:
                    outputBottle.clear()
                    outputBottle.addString("nack")
                    #outputBottle.addString("Greg")

                inputInteractionPort.reply(outputBottle)

            #del l
        except KeyboardInterrupt:
            print 'Interrupted'
            try:
                sys.exit(0)
            except SystemExit:
                os._exit(0)