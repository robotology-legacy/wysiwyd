#!/usr/bin/python

#""""""""""""""""""""""""""""""""""""""""""""""
#The University of Sheffield
#WYSIWYD Project
#
#SAMpy class for implementation of SAM module
#
#Created July 2015
#
#@authors: Luke Boorman, Uriel Martinez,  Andreas Damianou
#
#""""""""""""""""""""""""""""""""""""""""""""""

from SAM.SAM_Core import SAMCore
import matplotlib.pyplot as plt
#import matplotlib as mp
import pylab as pb
import sys
#import pickle
import numpy
import os
try:
    import yarp
    isYarpRunningGlobal = True
except ImportError:
    isYarpRunningGlobal = False
import cv2
import GPy
import time
from scipy.spatial import distance
#from scipy.signal import savgol_filter
from scipy.signal import medfilt
import operator



from SAM.SAM_Core import SAMDriver


#""""""""""""""""
#Class developed for the implementation of the action recognition task in real-time mode.
#""""""""""""""""

class SAMDriver_actions(SAMDriver):

#""""""""""""""""
#Initilization of the SAM class
#Inputs:
#    - isYarprunning: specifies if yarp is used (True) or not(False)
#    - imgH, imgW: original image width and height
#    - imgHNewm imgWNew: width and height values to resize the image
#
#Outputs: None
#""""""""""""""""
    def __init__(self, isYarpRunning = False, inputActionPort="/visionDriver/bodyPartPosition:o"):
        if not isYarpRunningGlobal and isYarpRunning:
            isYarpRunning = False
            print 'Warning! yarp was not found in the system.'

        self.isYarpRunning = isYarpRunning
        self.plotFlag=False
        self.plotPreProcessedData = False # plot preprocessed data
        
        self.inputActionPort=inputActionPort
        self.SAMObject=SAMCore.LFM()
        self.actionLabels = [] # list of actions is generated when loading data
        self.handLabels = [] # list of hands used is generated when loading data and place matches self.actionLabels

        self.dataFileName="data.log"
        self.bodyPartNames=['Face','Body','Left Arm','Right Arm','Left and Right Arms']
        self.bodyPartIndex=numpy.array([[0,1,2],[3,4,5],[6,7,8],[9,10,11]])

        self.Y = None
        self.L = None
        self.X = None
        self.Ytest = None
        self.Ltest = None
        self.Ytestn = None
        self.Ltestn = None
        self.Ymean = None
        self.Ystd = None
        self.Yn = None
        self.Ln = None
        self.data_labels = None
        self.participant_index = None
        self.cutEnds=True  # cut off first and last values (remove start and end effects e.g. trailling zero)
        self.preProcessDataFlag = True # Zero mean and median filter data        
        

        # Tags columns from input file that will be processed, e.g smoothed and differentiated
        self.indToProcess=(2,3,4,5,6,7,8,9,10,11,12,13) # index to grab from file for processing
        self.indTim=1

        # ############## Parameters for movement segmentation ################
        self.actionStopTime = 1 # Time in s for splitting each new movement
        self.minActionTime = 0.4 # Minimum length for action in s
        self.minimumMovementThreshold = 3 # Equivalent number of steps or pixels or mm that triggers movement  
        self.maxMovementTime = 3 # Greatest duration of movement in s        
        self.filterWindow=5 # Median Filter window length
        # real time options
        #self.sampleRate = 0 # sample rate calculated from data!
        self.fixedSampleRate = 20  #Hz data will be interpolated up to this sample rate 
        self.minSampleRate = 5 #Hz data rejected is sample rate is below this
        
        # Action labels
        self.labelName = []
        self.labelIndex = None
        self.actionName = []
        
        # Calcs realtime time vector
        self.minActionSteps=int(self.fixedSampleRate*self.minActionTime)
        self.maxActionSteps=int(self.fixedSampleRate*self.maxMovementTime)
        self.timRT=numpy.arange(0.0,self.maxMovementTime,1.0/self.fixedSampleRate)
        
        # GPy SAM model option
        self.model_num_inducing = 0
        self.model_num_iterations = 0
        self.model_init_iterations = 0
        


        if( self.isYarpRunning == True ):
            yarp.Network.init()
            self.createPorts()
            self.actionDataInputPort = yarp.BufferedPortBottle()#yarp.BufferedPortImageRgb()
            self.outputActionPrediction = yarp.Port()
            self.speakStatusPort = yarp.RpcClient()
            self.speakStatusOutBottle = yarp.Bottle()
            self.speakStatusInBottle = yarp.Bottle()
            self.actionDataInputPort.open("/sam/actions/actionData:i");
            self.outputActionPrediction.open("/sam/actions/actionPrediction:o")
            self.speakStatusPort.open("/sam/actions/speakStatus:i")
            self.speakStatusOutBottle.addString("stat")
            #self.openPorts()
            #self.createImageArrays()



        """
        #
        #Method to prepare the arrays to receive the RBG images from yarp
        #Inputs: None
        #Outputs: None
        #
        def createImageArrays(self):
        self.imageArray = numpy.zeros((self.imgHeight, self.imgWidth, 3), dtype=numpy.uint8)
        #self.newImage = yarp.ImageRgb()
        self.yarpImage = yarp.ImageRgb()
        self.yarpImage.resize(self.imgWidthNew,self.imgWidthNew)
        self.yarpImage.setExternal(self.imageArray, self.imageArray.shape[1], self.imageArray.shape[0])
        """
    def getColumns(self, inFile, delim="\t", header=True):
        """
        Get columns of data from inFile. The order of the rows is respected
    
        :param inFile: column file separated by delim
        :param header: if True the first line will be considered a header line
        :returns: a tuple of 2 dicts (cols, indexToName). cols dict has keys that 
        are headings in the inFile, and values are a list of all the entries in that
        column. indexToName dict maps column index to names that are used as keys in 
        the cols dict. The names are the same as the headings used in inFile. If
        header is False, then column indices (starting from 0) are used for the 
        heading names (i.e. the keys in the cols dict)
        """
        cols = {}
        indexToName = {}
        for lineNum, line in enumerate(inFile):
            if lineNum == 0:
                headings = line.split(delim)
                i = 0
                for heading in headings:
                    heading = heading.strip()
                    if header:
                        cols[heading] = []
                        indexToName[i] = heading
                    else:
                        # in this case the heading is actually just a cell
                        cols[i] = [heading]
                        indexToName[i] = i
                    i += 1
            else:
                cells = line.split(delim)
                i = 0
                for cell in cells:
                    cell = cell.strip()
                    cols[indexToName[i]] += [cell]
                    i += 1
        # Convert cols to numpy
        # check its rectangular, go through each column
        
        for colsChk in range(len(cols)):
            if (colsChk==0):
                colsRows=len(cols[0])
            else:
                #print "colsRows: " + str(colsRows)
                if (colsRows!=len(cols[colsChk])):
                    print "MATRIX is not rectangular cannot convert to NUMPY array"
                    return numpy.empty((1,1))
        
        dataReturn = numpy.empty([colsRows,len(cols)],dtype=numpy.float)
        
        for colsColumns in range(len(cols)):
            for colsRows in range(colsRows):
                dataReturn[colsRows,colsColumns]=float(cols[colsColumns][colsRows])
        
        if (self.cutEnds):
            #print dataLog.shape
            dataReturn = dataReturn[10:-10,:]
            #print dataLog.shape
                
        
        
        return dataReturn #cols, indexToName

    def preProcessData(self,dataIn,indToProcess,indTim):
        # This will cut and filter data prior to splitting
        # dataIn as data points (ts) by each ts (e.g. face x y z, body x y z....) 
        #zeroMeanData=[2,3,4,5,6,7,8,9,10,11,12,13] # -1=off, index to columns to zero mean -> Should only OPERATE ON FACE AND BODY DATA
        #preProcessData= [2,3,4,5,6,7,8,9,10,11,12,13]# -1 off, index to colums to filt currently medfilt. TODO Changes of less than three values about zeros are removed e.g. [0 0 # # 0 0]    
        
        # Optional plot raw        
        if (self.plotPreProcessedData):   
            #plotInd=(6,7,9,10) # ind to plot from indTo process            
            plotInd=(0,1,2,3,4,5,6,7,8,9,10,11) # ind to plot from indTo process              
            plt.figure(self.test_count+500)
            for currentInd in range(len(plotInd)):
                plt.subplot(len(plotInd),1,currentInd+1)
                plt.hold(True)
                lineRaw, = plt.plot(range(0,dataIn.shape[0]),dataIn[:,indToProcess[plotInd[currentInd]]],'r',label='raw')
        
        dataOut=numpy.empty([dataIn.shape[0],len(indToProcess)],dtype=float)
        dataDiff=numpy.empty([dataIn.shape[0]-1,len(indToProcess)],dtype=float)        
        #dataDiff2nd=numpy.empty([dataIn.shape[0]-2,len(indToProcess)],dtype=float)        
        
        tim=dataIn[:,indTim]
        diffTim=tim[:-1]        
        
        # Check for corrupt data        
        maxVal=numpy.max(dataIn[:,indToProcess])
        print "Max:" + str(maxVal)
        
        
        
        
        ## Main processing here....
        for indCount,currentXYZ in enumerate(indToProcess):
            # Zero mean
            dataOut[:,indCount]=dataIn[:,currentXYZ]-numpy.mean(dataIn[:,currentXYZ])   
        #for indCount in range(len(indToProcess)):    
            # Median filt -> window 5            
            dataOut[:,indCount]=medfilt(dataOut[:,indCount],self.filterWindow)
            # Diff data to find action movement
            dataDiff[:,indCount]=numpy.diff(dataOut[:,indCount])                
            # Median filt -> window 5            
            dataDiff[:,indCount]=medfilt(dataDiff[:,indCount],self.filterWindow)
            # Diff data to find action movement
            #dataDiff2nd[:,indCount]=numpy.diff(dataDiff[:,indCount])                
            # Median filt -> window self.filterWindow            
            #dataDiff2nd[:,indCount]=medfilt(dataDiff2nd[:,indCount],self.filterWindow)
            
        # Optional now overlay processed data
        if (self.plotPreProcessedData):
            for currentInd in range(len(plotInd)):
                plt.subplot(len(plotInd),1,currentInd+1)
                lineProc, = plt.plot(range(0,dataOut.shape[0]),dataOut[:,plotInd[currentInd]],'b',label='processed')
                lineDiff, = plt.plot(numpy.arange(dataDiff.shape[0],dtype=float)+0.5,dataDiff[:,plotInd[currentInd]],'g',label='proc diff')
                #lineDiff2nd, = plt.plot(numpy.arange(dataDiff2nd.shape[0],dtype=float)+0.5,dataDiff2nd[:,plotInd[currentInd]],'m',label='proc 2nd diff')
                plt.title(str(plotInd[currentInd]))
            plt.legend(handles=[lineRaw, lineProc,lineDiff])#,lineDiff2nd])

            
        return dataOut, dataDiff, tim, diffTim

    def findMovements(self, data, dataDiff, tim, ind2Check, actionSubList):
        # Segment Body part data
        # Version 2 Luke August 2015
        # Looks for maximum movement from any body region and then sections the data by finding periods of no movement
        # self.actionStopTime = 2 # Time in s for splitting each movement
        # self.minimumMovementThreshold = 3 # Equivalent number of pixels that triggers movement  
        # self.minActionTime = shortest time for action to be accepted
        # self.maxMovementTime = 5 # greatest movement period in s
        # Calc steps for movement off (e.g. still time)        
        
        sampleRate=1/numpy.mean(numpy.diff(tim))
        minActionSteps=int(sampleRate*self.minActionTime)

        print "SampleRate: ", sampleRate
        print "minActionTime: ", self.minActionTime

        newActionSteps=int(sampleRate*self.actionStopTime)
        maxLocalActionSteps=int(sampleRate*self.maxMovementTime)
        
        print "Min action steps:" + str(minActionSteps) 
        # Find maximum value across each time point (change in pos for all body parts and x,y,x)
        dataMax = numpy.max(numpy.abs(dataDiff[:,ind2Check]),axis=1)
        # Threshold data to find where the body part change is greater than the threshold
        #dataMoving = numpy.where(dataMax > self.minimumMovementThreshold)    
        dataMoving = numpy.where(dataMax > self.minimumMovementThreshold, dataMax, 0)
        # Now find consecutive zeros
        actionData = []
        timeData = []
        # For sub split actions (by gradient)
        actionDataPos = []
        actionDataNeg = []        
        timeDataPos = []
        timeDataNeg = []
        
        actionCount = 0
        
        actionFound = False
        zeroCount = 0
        # Separate actions depending on non-movement periods -> uses self.minActionTIme
        for currentInd in range(numpy.size(dataMoving)):
            # Check for movement (non-zero)
            if (dataMoving[currentInd]!=0):
                # New action, init whole thing
                if (not actionFound):
                    actionData.append(numpy.array(data[currentInd]))
                    timeData.append(numpy.array(tim[currentInd]))
                    actionFound = True
                else:
                # if action already found add values onto action
                    actionData[actionCount]=numpy.vstack((actionData[actionCount],data[currentInd]))
                    timeData[actionCount]=numpy.vstack((timeData[actionCount],tim[currentInd]))
                    #actionFound = True 
                # reset consecutive zeroCount
                zeroCount = 0
            # Case of zero... no movement
            else:
                # First check a new action has started, but we have not found too many zeros
                if (actionFound and zeroCount<newActionSteps):
                    zeroCount += 1 # increment zerocount
                    # Add data to action 
                    actionData[actionCount]=numpy.vstack((actionData[actionCount],data[currentInd]))
                    timeData[actionCount]=numpy.vstack((timeData[actionCount],tim[currentInd]))
                # Case for new action as enough zeros have been reached
                elif (actionFound and zeroCount>=newActionSteps):
                    # End of action as enough non-movement detected
                    actionFound = False
                    # Remove extra non-movement
                    actionData[actionCount]=actionData[actionCount][:-zeroCount,:]
                    timeData[actionCount]=timeData[actionCount][:-zeroCount]
                    
                    # Remove actions if too short
                    if (actionData[actionCount].shape[0]<int(minActionSteps)):
                        print "Removing action:" + str(actionCount) + " with length: " + str(actionData[actionCount].shape[0])  + " as too short"
                        actionData.pop(actionCount)
                        timeData.pop(actionCount)
                    else:
                        # Cut to largest movement allowed self.maxMovementTime
                        if (actionData[actionCount].shape[0]>maxLocalActionSteps): 
                            actionData[actionCount]=actionData[actionCount][:maxLocalActionSteps,:]
                            timeData[actionCount]=timeData[actionCount][:maxLocalActionSteps]
                        # Report and increment
                        print "Added action no: " + str(actionCount) + " with length: " + str(actionData[actionCount].shape[0])  
                        
                        
                        # LB August 2015 -> New section
                        # Sub segment actions                        

                        if (len(actionSubList)>1):
                            # This section will split actions by gradient
                            print "Subsplitting actions!"
                            # Find greatest movement                            
                            maxMovementInd=numpy.argmax(numpy.sum(numpy.abs(numpy.diff(actionData[actionCount],1,axis=0)),axis=0))
                            # Linear fit to data
                            linearFitPoly=numpy.polyfit(actionData[actionCount][:,maxMovementInd],range(actionData[actionCount].shape[0]),1)
                            # Check polarity +ve / -ve gradient
                            if (linearFitPoly[0]>0):
                                # 1. positive gradient
                                actionDataPos.append(actionData[actionCount])
                                timeDataPos.append(timeData[actionCount])
                            else:
                                # 2. Negative gradient
                                actionDataNeg.append(actionData[actionCount])
                                timeDataNeg.append(timeData[actionCount])
                        # Place action into pos by default!
                        else:
                            actionDataPos.append(actionData[actionCount])
                            timeDataPos.append(timeData[actionCount])                    
                        actionCount += 1
                        
        # Special cases either one action or no actions found
        # Nothing found check
        if (actionCount==0 and not actionFound):
            print "Nothing found"
            return [], [], [],[], [], []
        # One action found - repeat above process that was missed out!
        elif (actionCount==0 and actionFound):
            print "Single action found"            
            # Check if single action found is positive or negative
            if (actionDataPos):
                # Remove actions if too short
                if (actionDataPos[0].shape[0]<int(minActionSteps)):
                    print "Removing pos action:" + str(0) + " with length: " + str(actionDataPos[0].shape[0])  + " as too short"
                    actionDataPos.pop(0)
                    timeDataPos.pop(0)
                else:
                    # Cut to largest movement allowed self.maxMovementTime
                    if (actionDataPos[0].shape[0]>maxLocalActionSteps): 
                        actionDataPos[0]=actionDataPos[0][:maxLocalActionSteps,:]
                        timeDataPos[0]=timeDataPos[0][:maxLocalActionSteps]
                    print "Added pos action no: " + str(0) + " with length: " + str(actionDataPos[0].shape[0])
            # Check if single action found is positive or negative
            if (actionDataNeg):            
                # Remove actions if too short
                if (actionDataNeg[0].shape[0]<int(minActionSteps)):
                    print "Removing neg action:" + str(0) + " with length: " + str(actionDataNeg[0].shape[0])  + " as too short"
                    actionDataNeg.pop(0)
                    timeDataNeg.pop(0)
                else:
                    # Cut to largest movement allowed self.maxMovementTime
                    if (actionDataNeg[0].shape[0]>maxLocalActionSteps): 
                        actionDataNeg[0]=actionDataNeg[0][:maxLocalActionSteps,:]
                        timeDataNeg[0]=timeDataNeg[0][:maxLocalActionSteps]
                    print "Added neg action no: " + str(0) + " with length: " + str(actionDataNeg[0].shape[0])                   
        
        actionDataPos, timeDataPos, actionDataPosZeroPad = self.postProcessActions(actionDataPos,timeDataPos)
        
        if (actionDataNeg): # If sub actions found
            actionDataNeg, timeDataNeg, actionDataNegZeroPad = self.postProcessActions(actionDataNeg,timeDataNeg)
        else:
            actionDataNegZeroPad = []
            
        return actionDataPos, timeDataPos, actionDataPosZeroPad, actionDataNeg, timeDataNeg, actionDataNegZeroPad
                
    def postProcessActions(self,actionData,timeData):
        # Expand all actions to full maxMovementTime e.g. 5s
        # Zero pad after action upto maxMovementTime
        # Loop through all actions
        actionDataZeroPad=[]
        for currentAction in range(len(actionData)):
            
            # Interpolate data to max sample time!!!!!!!!!!!!
            actionDataTemp=actionData[currentAction]
            timeDataTemp=timeData[currentAction]
            # Process before return!
            # 1. Zero time to start
            timeDataTemp=timeDataTemp-timeDataTemp[0]
            # 2. Zero start of action                                
            actionDataTemp=actionDataTemp-numpy.tile(actionDataTemp[0,:],(actionDataTemp.shape[0],1))                                                      
            # Linearly interpolate data to 20Hz                                
            timeData[currentAction]=numpy.arange(0,timeDataTemp[-1],1.0/self.fixedSampleRate)
            actionData[currentAction]=numpy.zeros((timeData[currentAction].shape[0],actionDataTemp.shape[1]))
            # interpolate across all body parts
            for currentBP in range(actionData[currentAction].shape[1]):                            
                actionData[currentAction][:,currentBP]=numpy.interp(timeData[currentAction],timeDataTemp[:,0],actionDataTemp[:,currentBP])            

            if (actionData[currentAction].shape[0]<self.maxActionSteps):
                zeroLength=self.maxActionSteps-actionData[currentAction].shape[0]
                #print actionData[currentAction].shape
                #print str(zeroLength) + " " + str(actionData[currentAction].shape[1])
                actionDataZeroPad.append(numpy.vstack((actionData[currentAction],numpy.zeros((zeroLength,actionData[currentAction].shape[1])))))
            elif (actionData[currentAction].shape[0]>self.maxActionSteps):
                actionDataZeroPad.append(actionData[currentAction][:self.maxActionSteps,:])
                print "WARNING DATA TOO LONG: " +  str(actionData[currentAction].shape[0])
            else:
                actionDataZeroPad.append(actionData[currentAction])
            
        return actionData, timeData, actionDataZeroPad
    
#""""""""""""""""
#Method to read action data previously collected to be used in the traning phase.
#Here the loaded data is preprocessed to have the correct image size and one action per image.
#Inputs:
#    - root_data_dir: location of action data
#    - participant_inde: array of participants names
#    - pose_index: array of poses from the action data collected
#
#Outputs: None
#""""""""""""""""(root_data_dir,participant_index,hand_index,action_index)
    def readData(self, root_data_dir,participant_index,hand_index,action_index,action_splitting_index):
        self.Y
        self.L
        self.action_index = action_index
        self.participant_index = participant_index
        self.hand_index = hand_index
        self.action_splitting_index = action_splitting_index
        

        # Max action time -> 5s will be used to generate fixed array for each action
        #self.maxActionTime = 5 # maximum action time in s 
        
        # Check if root dir exists
        if not os.path.exists(root_data_dir):
            print "CANNOT FIND: " + root_data_dir
        else:
            print "PATH FOUND: " + root_data_dir

        #data_file_database={}
        #dataFile = file(root_data_dir+"/data.log")

        # Generate directory names -> structure will likely change
        # 1. action index
        self.test_count=1  
        actionsbyType=[]
        actionsbyTypeLabel=[]
        
        # LB -> this has got so complicated so....
        # Generate label index before loading files!
        self.labelName = []
        self.actionName = []
        #labelHand = []
        actionLabelCount = 0
        for currentHand in range(len(self.hand_index)):
            for currentAction in range (len(self.action_splitting_index)):
                for currentSubAction in range (len(self.action_splitting_index[currentAction])):                     
                    self.labelName.append(self.action_splitting_index[currentAction][currentSubAction]+ " " +self.hand_index[currentHand])
                    self.actionName.append(self.action_splitting_index[currentAction][currentSubAction])
                    #labelHand.append(self.hand_index[currentHand])
                    if (actionLabelCount==0):
                        self.labelIndex = numpy.array(actionLabelCount)
                    else:                        
                        self.labelIndex = numpy.vstack((self.labelIndex,actionLabelCount+(currentHand*100)))
                    actionLabelCount += 1
     
        #actionsbyTypeNeg=[]
        #actionsbyTypeLabelNeg=[]
        
        # NEED TO CHECK FOR sucessful finding of actions from each sub index for building the final array.....        
        
        minActionsOverall = 10000 # find least number of actions for every context....
        minActionDuration = 100000 # find the shortest length for all actions
        actionTypesFoundCount=0 # count successful actions found
        
        for actionInd in range(len(self.action_index)):
            # 2. Participant index
            actionsbyParticipant=[]
            actionsbyParticipantLabel=[]
            
            actionsbyParticipantNeg=[]
            actionsbyParticipantLabelNeg=[]
            minParticipant = 10000 # find min number of participants
            for partInd in range(len(self.participant_index)):
                # 3. hand index
                actionsbyHand=[]
                actionsbyHandLabel=[]
                
                actionsbyHandNeg=[]
                actionsbyHandLabelNeg=[]
                minHands = 10000 # find min number of hands

                for handInd in range(len(self.hand_index)):
                    # Check if root dir exists
                    dir_string=os.path.join(root_data_dir,(self.participant_index[partInd] + "_" + self.hand_index[handInd]\
                    + "_" + self.action_index[actionInd]))
                    
                    print dir_string
                    print self.participant_index[partInd]
                    print self.hand_index[handInd]

                    if not os.path.exists(dir_string):
                        print "CANNOT FIND: " + dir_string
                    else:
                        print "PATH FOUND: " + dir_string
                        # Check file in dir....
                        dataFilePath=os.path.join(dir_string,self.dataFileName)
                        if not os.path.exists(dataFilePath):
                            print "CANNOT FIND: " + dataFilePath
                        else:
                            print "PATH FOUND: " + dataFilePath
                            # Open file
                            dataFile = open(dataFilePath, 'rb')
                            logData = self.getColumns(dataFile, " ", False)
                            if (logData.size==0):
                                print "Log Data empty skipping"
                            else:
                                dataFile.close();
                                print "Mean: ",   numpy.mean(numpy.diff(logData[:,1]))
                                print "Sample rate: " + str(1/numpy.mean(numpy.diff(logData[:,1])))
                                if (self.preProcessDataFlag):
                                    dataProc, dataDiff, tim, diffTim = self.preProcessData(logData,self.indToProcess,self.indTim)
                                
                                self.test_count+=1
                                
                                # Segment body parts and Send sub actions                                
                                #actionData, timeData, actionDataZeroPad=self.findMovements(dataProc, dataDiff, tim,range(dataDiff.shape[1]), self.action_splitting_index[actionInd])
                                # Here psoitive will be used as action data, with neg secondary! Luke 2015                                
                                actionData, timeData, actionDataZeroPad, actionDataNeg, timeDataNeg, actionDataNegZeroPad =self.findMovements(dataProc, dataDiff, tim,range(dataDiff.shape[1]), self.action_splitting_index[actionInd])
                                
                                # Single Action type or Positive sub Action gradient Condition
                                # Check action found (only false if empty)
                                if actionData:
                                    # Sort counters here -> find minimum number of actions -> used later for cutting
                                    if len(actionDataZeroPad) < minActionsOverall:
                                        minActionsOverall = len(actionDataZeroPad)
                                    
                                    # Plot action output
                                    if (self.plotFlag):
                                        for currentAction in range(len(actionData)):
                                            # Generate and use same color throughout
                                            color_rand=numpy.random.rand(3,1)
                                            plt.figure(888+self.test_count)
                                            for currentBP in range(numpy.shape(actionData[currentAction])[1]):
                                                plt.subplot(numpy.shape(actionData[currentAction])[1],1,currentBP+1)
                                                plt.hold(True)                                    
                                                plt.plot(timeData[currentAction],actionData[currentAction][:,currentBP],c=color_rand)
                                            plt.subplot(numpy.shape(actionData[currentAction])[1],1,1)                                    
                                            plt.title("Pos " + dataFilePath)
                                            
                                            plt.figure(777+self.test_count)
                                            for currentBP in range(numpy.shape(actionData[currentAction])[1]):
                                                plt.subplot(numpy.shape(actionData[currentAction])[1],1,currentBP+1)
                                                plt.hold(True)                                    
                                                plt.plot(actionDataZeroPad[currentAction][:,currentBP],c=color_rand)
                                            plt.subplot(numpy.shape(actionData[currentAction])[1],1,1)                                    
                                            plt.title("Pos " + dataFilePath)
                                    
                                    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                                    # ~~~~~~~~~~~~~ FORMATTING THE DATA FOR THE SAM ~~~~~~~~~~~~~~~
                                    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
                                    # Make overall array
                                    # A. Data
                                    # Data as:
                                    # >> 1. Timeseries
                                    # >> 2. Body part and xyz
                                    # >> 3. Repeat actions
                                    # 4. Hand used
                                    # 5. Person
                                    # 6. Action Type
                                    actionsCombined=numpy.zeros((actionDataZeroPad[0].shape[0],actionDataZeroPad[0].shape[1],len(actionDataZeroPad)))
                                    for currentAction in range(len(actionDataZeroPad)):
                                        actionsCombined[:,:,currentAction]=actionDataZeroPad[currentAction]
                                        # Check for shortest action length
                                        if actionDataZeroPad[currentAction].shape[0]<minActionDuration:
                                            minActionDuration=actionDataZeroPad[currentAction].shape[0]
                                    # B. Labels                                
                                    # Initally built around the action name (based on action index)
                                    # Find matching label and assign       
                                    actionLabel=numpy.empty(0)       
                                    for testLabel in range(len(self.labelName)):
                                        #print "Testing: " + (self.action_splitting_index[actionInd][0] + " " + self.hand_index[handInd])
                                        #print "with: " + self.labelName[testLabel]
                                        if (str(self.action_splitting_index[actionInd][0] + " " + self.hand_index[handInd]) == str(self.labelName[testLabel])):
                                            actionLabel=numpy.zeros((actionDataZeroPad[0].shape[0],actionDataZeroPad[0].shape[1],len(actionDataZeroPad)))+self.labelIndex[testLabel]#len(self.actionLabels)#actionInd
                                            break
                                    if (actionLabel.size == 0):
                                        print "ERROR NO MATCHING ACTION FOUND"
                                        return 0
                                    # TODO THIS MUST BE UPDATED TO NEW LABELS otherwise its meaningless LB 2015
                                    #if (partInd==0):                                            
                                    #    self.actionLabels.append(self.action_splitting_index[actionInd][0])
                                    #    self.handLabels.append(self.hand_index[handInd])
                                    
                                    
                                    
                                    
                                    
                                #Dual Action type -> Negative sub Action gradient Condition
                                # Check action found (only false if empty)
                                if actionDataNeg:
                                    # Sort counters here -> find minimum number of actions -> used later for cutting
                                    if len(actionDataNegZeroPad) < minActionsOverall:
                                        minActionsOverall = len(actionDataNegZeroPad)
                                    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                                    # ~~~~~~~~~~~~~ FORMATTING THE DATA FOR THE SAM ~~~~~~~~~~~~~~~
                                    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
                                    # Make overall array
                                    # A. Data
                                    # Data as:
                                    # >> 1. Timeseries
                                    # >> 2. Body part and xyz
                                    # >> 3. Repeat actions
                                    # 4. Hand used
                                    # 5. Person
                                    # 6. Action Type
                                    actionsCombinedNeg=numpy.zeros((actionDataNegZeroPad[0].shape[0],actionDataNegZeroPad[0].shape[1],len(actionDataNegZeroPad)))
                                    for currentAction in range(len(actionDataNegZeroPad)):
                                        actionsCombinedNeg[:,:,currentAction]=actionDataNegZeroPad[currentAction]
                                        # Check for shortest action length
                                        if actionDataNegZeroPad[currentAction].shape[0]<minActionDuration:
                                            minActionDuration=actionDataNegZeroPad[currentAction].shape[0]
                                    # B. Labels                                
                                    # Initally built around the action name (based on action index)
                                    # TODO THIS MUST BE UPDATED TO NEW LABELS otherwise its meaningless LB 2015
                                            
                                    # Find matching label and assign       
                                    actionLabelNeg=False       
                                    for testLabel in range(len(self.labelName)):
                                        if (str(self.action_splitting_index[actionInd][1] + " " + self.hand_index[handInd]) == str(self.labelName[testLabel])):
                                            actionLabelNeg=numpy.zeros((actionDataZeroPad[0].shape[0],actionDataZeroPad[0].shape[1],len(actionDataZeroPad)))+self.labelIndex[testLabel]#len(self.actionLabels)#actionInd
                                            break
                                    if (actionLabelNeg.size == 0):
                                        print "ERROR NO MATCHING ACTION FOUND"
                                        return 0                                                 
                                    #if (partInd==0):
                                    #    self.actionLabels.append(self.action_splitting_index[actionInd][1])
                                    #    self.handLabels.append(self.hand_index[handInd])
                                    #actionLabelNeg=numpy.zeros((actionDataNegZeroPad[0].shape[0],actionDataNegZeroPad[0].shape[1],len(actionDataNegZeroPad)))+len(self.actionLabels)#+actionInd
                                    
                    # Combine arrays across actions hands
                    # Single / Pos condition
                    actionsbyHand.append(actionsCombined) 
                    actionsbyHandLabel.append(actionLabel)
                    # Negative condition 
                    if actionDataNeg:
                        actionsbyHandNeg.append(actionsCombinedNeg) 
                        actionsbyHandLabelNeg.append(actionLabelNeg)                    
                    
                # Combine arrays across actions participant
                # Single / Pos condition
                actionsbyParticipant.append(actionsbyHand) 
                actionsbyParticipantLabel.append(actionsbyHandLabel)
                # Check here for hand count should be 2 ->
                if len(actionsbyHand)<minHands:
                    minHands=len(actionsbyHand)
                # Negative condition    
                if actionDataNeg:
                    actionsbyParticipantNeg.append(actionsbyHandNeg) 
                    actionsbyParticipantLabelNeg.append(actionsbyHandLabelNeg)
                    # Check here for hand count should be 2 ->
                    if len(actionsbyHandNeg)<minHands:
                        minHands=len(actionsbyHandNeg)           
            # Combine arrays across actions participant
            # Single / Pos condition
            actionsbyType.append(actionsbyParticipant)
            actionsbyTypeLabel.append(actionsbyParticipantLabel)
            if len(actionsbyParticipant)<minParticipant:
                minParticipant=len(actionsbyParticipant)
            if actionsbyType: # false if empty
                actionTypesFoundCount+=1
            # Negative condition    
            if actionDataNeg:
                # @@@L@B@@@@ Key here adding neg grad actions to main array!
                actionsbyType.append(actionsbyParticipantNeg)
                actionsbyTypeLabel.append(actionsbyParticipantLabelNeg)
                #actionsbyTypeNeg.append(actionsbyParticipantNeg)
                #actionsbyTypeLabelNeg.append(actionsbyParticipantLabelNeg)
                if len(actionsbyParticipantNeg)<minParticipant:
                    minParticipant=len(actionsbyParticipantNeg)                
                # LB TODO CHECK HERE!!!!!!!!!
                #if actionsbyTypeNeg: # false if empty
                actionTypesFoundCount+=1
                    
        # Print outputs
        print str(actionTypesFoundCount) + " different actions found"     
        print "Shortest action length found is " + str(minActionDuration) + " steps"        
        print "Found " + str(minActionsOverall) + " actions for every participant and hand used"
        print "Found minimum of data recorded from " + str(minHands) + " hands for all pariticipants involved"         
        print "Found " + str(minParticipant) + " participants completeing all actions"
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # ~~~~~~~~~~~~~~ COMBINE AND CUT THE DATA FOR THE SAM ~~~~~~~~~
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Build the nump matrix
        # Data as:
        # 1. Timeseries         0
        # 2. Body part and xyz  1
        # 3. Repeat actions     2
        # 4. Hand used          3
        # 5. Person             4
        # 6. Action Type        5
        allActions=numpy.zeros((minActionDuration,len(self.indToProcess),minActionsOverall,minHands,minParticipant,actionTypesFoundCount))
        allLabels=numpy.zeros((minActionDuration,len(self.indToProcess),minActionsOverall,minHands,minParticipant,actionTypesFoundCount))        
        # 6. Action type
        for actionInd in range(actionTypesFoundCount):
            # 5. Participant index
            for partInd in range(minParticipant):
                # 4. hand index
                for handInd in range(minHands):
                    allActions[:,:,:,handInd,partInd,actionInd]=actionsbyType[actionInd][partInd][handInd][:minActionDuration,:len(self.indToProcess),:minActionsOverall]
                    allLabels[:,:,:,handInd,partInd,actionInd]=actionsbyTypeLabel[actionInd][partInd][handInd][:minActionDuration,:len(self.indToProcess),:minActionsOverall]
                            #strftime("%a, %d %b %Y %H:%M:%S +0000", gmtime())
                
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # ~~~~~~~~~~~~~~ ALTERNATIVE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # ~~~~~~~~~~~~~~ 1. Combines actions Type to include hand e.g. waving right , ud left
        # ~~~~~~~~~~~~~~ 2. flatten timeseries and body part xyz and participant
        # ~~~~~~~~~~~~~~ COMBINE AND CUT THE DATA FOR THE SAM ~~~~~~~~~
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Build the numpy matrix
        # Data as:
        # 1. Timeseries (0) + Body part and xyz (1)
        # 2. Repeat actions (2) + person (4)
        # 3. Action Type (5) and Hand used (3)
        combinedActions=numpy.zeros((minActionDuration*len(self.indToProcess),minActionsOverall*minParticipant,minHands*actionTypesFoundCount))
        combinedLabels=numpy.zeros((minActionDuration*len(self.indToProcess),minActionsOverall*minParticipant,minHands*actionTypesFoundCount))        
        # Reshape timeseries, body part xyz
        temp1=numpy.reshape(numpy.transpose(allActions,(2,3,4,5,1,0)),(minActionsOverall,minHands,minParticipant,actionTypesFoundCount,minActionDuration*len(self.indToProcess)))      
        temp2=numpy.reshape(numpy.transpose(allLabels,(2,3,4,5,1,0)),(minActionsOverall,minHands,minParticipant,actionTypesFoundCount,minActionDuration*len(self.indToProcess)))      
        # !!!!!!UPDATE LABELS TO MARK BOTH Left and right hands for combined hands and action type 
        #for handInd in range(minHands):        
        #    temp2[:,handInd,:,:]=temp2[:,handInd,:,:]+(handInd*100) # added differential index: original label = left hand, original label + 100 = right hand
        # Combine Repeat actions and participant
        temp1=numpy.reshape(numpy.transpose(temp1,(4,1,3,0,2)),(minActionDuration*len(self.indToProcess),minHands,actionTypesFoundCount,minActionsOverall*minParticipant))
        temp2=numpy.reshape(numpy.transpose(temp2,(4,1,3,0,2)),(minActionDuration*len(self.indToProcess),minHands,actionTypesFoundCount,minActionsOverall*minParticipant))
        # Combine hands and action type    
        combinedActions=numpy.reshape(numpy.transpose(temp1,(0,3,1,2)),(minActionDuration*len(self.indToProcess),minActionsOverall*minParticipant,minHands*actionTypesFoundCount))
        combinedLabels=numpy.reshape(numpy.transpose(temp2,(0,3,1,2)),(minActionDuration*len(self.indToProcess),minActionsOverall*minParticipant,minHands*actionTypesFoundCount))
       
        plt.show()#block=True)

        self.Y=combinedActions
        self.L=combinedLabels


#""""""""""""""""
#Method to process some important features from the action data required for the classification model such as mean and variance.
#Inputs:
#    - model: type of model used for the SAM object
#    - Ntr: Number of training samples
#    - pose_selection: participants pose used for training of the SAM object
#
#Outputs: None
#""""""""""""""""
    def prepareActionData(self, model='mrd', Ntr = 50):    
        #""--- Now Y has 3 dimensions: 
        #1. time points (body part x,y,z and participant)
        #2. repeated actions 
        #3. Action type (waving. up/down. left / right)     
        #
        #We can prepare the action data using different scenarios about what to be perceived.
        #In each scenario, a different LFM is used. We have:
        #- gp scenario, where we regress from images to labels (inputs are images, outputs are labels)
        #- bgplvm scenario, where we are only perceiving images as outputs (no inputs, no labels)
        #- mrd scenario, where we have no inputs, but images and labels form two different views of the output space.
        #
        #The store module of the LFM automatically sees the structure of the assumed perceived data and 
        #decides on the LFM backbone to be used.
        #
        #! Important: The global variable Y is changed in this section. From the multi-dim. matrix of all
        #modalities, it turns into the training matrix of action data and then again it turns into the 
        #dictionary used for the LFM.
        #---""" 
        #OLD
        #0. Pixels = ts 0
        #1. Images = repeat actions 1
        #2. Person = actiontype 2
        #3. Movement (Static. up/down. left / right)
    
    
    
        # Take all poses if pose selection ==-1
        """  if pose_selection == -1:
        ttt=numpy.transpose(self.Y,(0,1,3,2))
        ttt=ttt.reshape((ttt.shape[0],ttt.shape[1]*ttt.shape[2],ttt.shape[3])) 
        else:
            ttt=self.Y[:,:,:,pose_selection]
        """    
        ttt=numpy.transpose(self.Y,(0,2,1))
        self.Y=ttt.reshape(ttt.shape[0],ttt.shape[2]*ttt.shape[1]) 
        self.Y=self.Y.T
        #N=self.Y.shape[0]
        """
        if pose_selection == -1:
            ttt=numpy.transpose(self.L,(0,1,3,2))
            ttt=ttt.reshape((ttt.shape[0],ttt.shape[1]*ttt.shape[2],ttt.shape[3]))
        else:
    		ttt=self.L[:,:,:,pose_selection]
        """ 
        
        ttt=numpy.transpose(self.L,(0,2,1))
        self.L=ttt.reshape(ttt.shape[0],ttt.shape[2]*ttt.shape[1]) 
        self.L=self.L.T
        self.L=self.L[:,:1]

        # 

        Nts=self.Y.shape[0]-Ntr
   
        perm = numpy.random.permutation(self.Y.shape[0])
        indTs = perm[0:Nts]
        indTs.sort()
        indTr = perm[Nts:Nts+Ntr]
        indTr.sort()
        self.Ytest = self.Y[indTs]
        self.Ltest = self.L[indTs]
        self.Y = self.Y[indTr]
        self.L = self.L[indTr]
    
        # Center data to zero mean and 1 std
        self.Ymean = self.Y.mean()
        self.Yn = self.Y - self.Ymean
        self.Ystd = self.Yn.std()
        self.Yn /= self.Ystd
        # Normalise test data similarly to training data
        self.Ytestn = self.Ytest - self.Ymean
        self.Ytestn /= self.Ystd

        # As above but for the labels
        self.Lmean = self.L.mean()
        self.Ln = self.L - self.Lmean
        self.Lstd = self.Ln.std()
        self.Ln /= self.Lstd
        self.Ltestn = self.Ltest - self.Lmean
        self.Ltestn /= self.Lstd

        if model == 'mrd':    
            self.X=None     
            self.Y = {'Y':self.Yn,'L':self.L}
            self.data_labels = self.L.copy()
        elif model == 'gp':
            self.X=self.Y.copy()
            self.Y = {'L':self.Ln.copy()+0.08*numpy.random.randn(self.Ln.shape[0],self.Ln.shape[1])}
            self.data_labels = None
        elif model == 'bgplvm':
            self.X=None     
            self.Y = {'Y':self.Yn}
            self.data_labels = self.L.copy()


#""""""""""""""""
#Method to test the learned model with actions read from the iCub eyes in real-time
#Inputs:
#    - testaction: image from iCub eyes to be recognized
#    - visualiseInfo: enable/disable the result from the testing process
#
#Outputs:
#    - pp: the axis of the latent space backwards mapping
#""""""""""""""""
    def testing(self, testAction, choice, objectFlag, visualiseInfo=None):
        # Returns the predictive mean, the predictive variance and the axis (pp) of the latent space backwards mapping.            
        ret = self.SAMObject.pattern_completion(testAction, visualiseInfo=visualiseInfo)
        mm = ret[0]
        vv = ret[1]
        post = ret[3]        
        # find nearest neighbour of mm and SAMObject.model.X
        dists = numpy.zeros((self.SAMObject.model.X.shape[0],1))

        facePredictionBottle = yarp.Bottle()
    
        for j in range(dists.shape[0]):
            dists[j,:] = distance.euclidean(self.SAMObject.model.X.mean[j,:], mm[0].values)
        nn, min_value = min(enumerate(dists), key=operator.itemgetter(1))
       
        
        if( int(self.SAMObject.model.bgplvms[1].Y[nn,:]) >= 100 ):
            actionIndex = int(self.SAMObject.model.bgplvms[1].Y[nn,:]) - 100
            handLabel = "right"
        else:
            actionIndex = int(self.SAMObject.model.bgplvms[1].Y[nn,:])
            handLabel = "left"        
        
        print "ACTION INDEX: ", actionIndex
                      
        
        if self.SAMObject.type == 'mrd':
            ret_y = self.SAMObject.model.bgplvms[1]._raw_predict(post.X)
            vv_y = ret_y[1]
            print "NN: ", int(self.SAMObject.model.bgplvms[1].Y[nn,:])
            # Generate dialogue here
            #print "With " + str(vv.mean()) + "(" + str(vv_y) + ")" +" prob. error the new image is " + self.labelName[int(self.SAMObject.model.bgplvms[1].Y[nn,:])]
            #print "With " + str(vv.mean()) + "(" + str(vv_y) + ")" +" prob. error the new image is " + self.participant_index[int(self.SAMObject.model.bgplvms[1].Y[nn,:])]
#            textStringOut=handLabel + " " + self.action_index[actionIndex]

            if (objectFlag):
                textStringOut = "you did nothing"
                if (self.actionName[actionIndex] == 'waving' ): # waving
                    textStringOut="You were " + self.actionName[actionIndex] + " using your " + handLabel + " hand"
                elif (self.actionName[actionIndex] == 'left'): # Moved left
                    if (handLabel == 'left'):
                        textStringOut="You pushed the object, with your " + handLabel + " hand"
                    else:
                        textStringOut="You pulled the object, with your " + handLabel + " hand"             
                elif (self.actionName[actionIndex] == 'right' ): # Moved right
                    if (handLabel == 'left'):
                        textStringOut="You pulled the object, with your " + handLabel + " hand"
                    else:
                        textStringOut="You pushed the object, with your " + handLabel + " hand"
                elif (self.actionName[actionIndex] == 'up'): # Up
                    textStringOut="You lifted the object up, with your " + handLabel + " hand"
                elif (self.actionName[actionIndex] == 'down' ): # Down
                    textStringOut="You put the object down, with your " + handLabel + " hand"
                else:
                    textStringOut = "No action!"
            else:
                textStringOut = "you did nothing"
                if (self.actionName[actionIndex] == 'waving' ): # waving
                    textStringOut="You were " + self.actionName[actionIndex] + " using your " + handLabel + " hand"
                elif (self.actionName[actionIndex] == 'left' or self.actionName[actionIndex] == 'right' ): # left and right
                    textStringOut="You moved your " + handLabel + " hand to the " + self.actionName[actionIndex]
                else: # Up and down
                    textStringOut="You moved your " + handLabel + " hand " + self.actionName[actionIndex]
                
        elif self.SAMObject.type == 'bgplvm':
            print "With " + str(vv.mean()) +" prob. error the new image is " + self.participant_index[int(self.L[nn,:])]
            textStringOut=self.labelName[int(self.L[nn,:])]
            
        if( choice.get(0).asInt() == 18 ):
            facePredictionBottle.addString(textStringOut) 
        
        # Plot the training NN of the test image (the NN is found in the INTERNAl, compressed (latent) memory space!!!)
        
        if visualiseInfo is not None:
            fig_nn = visualiseInfo['fig_nn']
            fig_nn = pb.figure(11)
            pb.title('Training NN')
            fig_nn.clf()
            #pl_nn = fig_nn.add_subplot(111)
            #print "SAM RECALL: ", self.SAMObject.recall(nn)
            #pl_nn.imshow(numpy.reshape(self.SAMObject.recall(nn),(self.imgHeightNew, self.imgWidthNew)), cmap=plt.cm.Greys_r)
            pb.title('Training NN')
            pb.show()
            pb.draw()
            pb.waitforbuttonpress(0.1)
        
            
        self.speakStatusPort.write(self.speakStatusOutBottle, self.speakStatusInBottle)
        if( self.speakStatusInBottle.get(0).asString() == "quiet"):
            self.outputActionPrediction.write(facePredictionBottle)
        facePredictionBottle.clear()

        return ret[2]
 
#""""""""""""""""
#Method to real-time read action data from the iCub used for the action recognition task
#Inputs: None
#Outputs:
#    - imageFlatten_testing: image from iCub eyes in row format for testing by the SAM model
# Detect action and send out detected actions......
# Loops here to find next action....        
#""""""""""""""""
    def readActionFromRobot(self):
        #self.sampleRate = 0 # sample rate calculated from data!
        #self.fixedSampleRate = 20  #Hz data will be interpolated up to this sample rate 
        #self.minSampleRate = 8 #Hz data rejected is sample rate is below this
        # Init variables 
        dataCount = 0
        
        zeroCount = 0
        actionFound = False
        
        actionData = []
        timeData = []
        

        
        while(True):
            
            
            try:
                newData = yarp.Bottle
                newData = self.actionDataInputPort.read(True) # read values from yarp port
                currentTime = time.time() # time stamp data as it arrives
 
            except KeyboardInterrupt:
                print 'Interrupted'
                try:
                    sys.exit(0)
                except SystemExit:
                    os._exit(0)

            if not(newData == None ):
                #self.yarpImage.copy(self.newImage)
                                
                # Get length of bottle
                data = numpy.zeros((newData.size(),1),dtype=int)
                for currentData in range(newData.size()):                
                    data[currentData] = newData.get(currentData).asInt()
                # Add to data store
                if (dataCount == 0):
                    dataStoreRT=data
                    baseTime = currentTime # baseline time to subtract
                    timeStoreRT = numpy.array([0]) # init time store
                    dataCount+=1
                else:
                    #print data
                    # Check appropiate sample rate!
                    if ((1/numpy.diff(timeStoreRT[-2:]))<self.minSampleRate):
                        print "CLEARING DATA: Sample rate too low: " + str(1/numpy.diff(timeStoreRT[-2:]))
                        actionFound = False
                        zeroCount = 0                        
                        dataCount = 0
                    else:
                        dataCount+=1
                        dataStoreRT=numpy.hstack((dataStoreRT,data))
                        timeStoreRT=numpy.hstack((timeStoreRT,currentTime-baseTime))
                        
                # Wait for at least 3 x filter window length data points to arrive before startin analysis
                if (dataCount>(2*self.filterWindow)):
                    # Initial sample rate from data and action detection times
                    #if (self.sampleRate == 0):
                    self.sampleRate = 1/numpy.mean(numpy.diff(timeStoreRT[-self.filterWindow:-1])) 
                    newActionSteps=int(self.sampleRate*self.actionStopTime) 
                    #else:
                    #    print "Real-time sampleRate = " + str(1/numpy.mean(numpy.diff(timeStoreRT[-self.filterWindow:-1]))) + "vs fixed rate " + str(self.sampleRate)    
                    # This runs in real time so we cannot use the normal file loading
                    # Apply median filter to data...
                    # 1. Preprocess data -> median filter and take first derivative then medin filter
                    # Iterate median filter over data 
                    dataMed=numpy.zeros((dataStoreRT.shape[0],self.filterWindow+1))
                    for currentWindow in range(self.filterWindow+1):
                        dataMed[:,-(currentWindow+1)]=numpy.median(dataStoreRT[:,-(self.filterWindow+currentWindow+1):-(currentWindow+1)],axis=1)
                    # Differentiate it
                    dataDiff=numpy.diff(dataMed,1,axis=1)
                    dataDiffMed=numpy.median(dataDiff,axis=1)
                
                    #2. Find action
                    # Find maximum action across all body parts x,y,z
                    dataMax=numpy.max(numpy.abs(dataDiffMed))
                    
                    # Separate actions depending on non-movement periods -> uses self.minActionTIme
                    # Check movement is above threshold
                    if (dataMax>self.minimumMovementThreshold):
                        #print "Action detected!!!!!!!!!!!!!! " #+ str(actionCount)
                        #actionCount+=1
                        if (not actionFound):
                            actionData=numpy.array([dataMed[:,-1]])
                            timeData=numpy.array([timeStoreRT[-1]])
                            actionFound = True
                        else:
                        # if action already found add values onto action
                            actionData=numpy.vstack((actionData,dataMed[:,-1]))
                            timeData=numpy.vstack((timeData,timeStoreRT[-1]))
                        # reset consecutive zeroCount as nonzero found
                        zeroCount = 0
                    else: # case of not enough movement
                        # First check a new action has started, but we have not found too many zeros to complete the action
                        if (actionFound and zeroCount<newActionSteps):
                            zeroCount += 1 # increment zerocount
                            # Add data to action 
                            actionData=numpy.vstack((actionData,dataMed[:,-1]))
                            timeData=numpy.vstack((timeData,timeStoreRT[-1]))
                        # Case for new action as enough zeros have been reached
                        elif (actionFound and zeroCount>=newActionSteps):
                            # End of action as enough non-movement detected
                            # Remove extra non-movement
                            actionDataTemp=actionData[:-zeroCount,:]
                            timeDataTemp=timeData[:-zeroCount]

                            zeroCount = 0
                            actionFound = False
                            # Process before return!
                            # 1. Zero time to start
                            timeDataTemp=timeDataTemp-timeDataTemp[0]
                            # 2. Zero start of action -> all actions from zero  -> LB disables used LATER                              
                            # actionDataTemp=actionDataTemp-numpy.tile(actionDataTemp[0,:],(actionDataTemp.shape[0],1))                                                      
                            # Linearly interpolate data to 20Hz                                
                            timeData=numpy.arange(0,timeDataTemp[-1],1.0/self.fixedSampleRate)
                            actionData=numpy.zeros((timeData.shape[0],actionDataTemp.shape[1]))
                            
                            # interpolate across all body parts
                            for currentBP in range(actionData.shape[1]):                            
                                actionData[:,currentBP]=numpy.interp(timeData,timeDataTemp[:,0],actionDataTemp[:,currentBP])
                            
                            # Remove actions if too short
                            if (actionData.shape[0]<int(self.minActionSteps)):
                                print "Removing action with length: " + str(actionData.shape[0])  + " as too short"
                                                                
                                #actionData.pop(actionCount)
                                #timeData.pop(actionCount)
                            else:
   
                                # Cut to largest movement allowed self.maxMovementTime
                                if (actionData.shape[0]>self.maxActionSteps): 
                                    actionData=actionData[:self.maxActionSteps,:]
                                    timeData=timeData[:self.maxActionSteps]
                                # Report and increment
                                print "Found action with length: " + str(actionData.shape[0])
                                
                                # 3. Zero pad data to max time
                                # Expand all actions to full maxMovementTime e.g. 5s
                                # Zero pad after action upto maxMovementTime
                                if (actionData.shape[0]<self.maxActionSteps):
                                    zeroLength=self.maxActionSteps-actionData.shape[0]
                                    #print actionData[currentAction].shape
                                    #print str(zeroLength) + " " + str(actionData[currentAction].shape[1])
                                    
                                    actionDataZeroPad=numpy.vstack((actionData,numpy.zeros((zeroLength,actionData.shape[1]))))
                                elif (actionData.shape[0]>self.maxActionSteps):
                                    actionDataZeroPad=actionData[:self.maxActionSteps,:]
                                    print "WARNING DATA TOO LONG: " +  str(actionData.shape[0])
                                else:
                                    actionDataZeroPad=actionData
                                
                                # Process data ready for the testing with the model
                                # Zero start of action -> all actions from zero  -> LB disables used LATER                              
                                actionDataZeroPad=actionDataZeroPad-numpy.tile(actionDataZeroPad[0,:],(actionDataZeroPad.shape[0],1))                                                                                  
                   
                                actionFormattedTesting = numpy.reshape(actionDataZeroPad.T,(actionDataZeroPad.shape[0]*actionDataZeroPad.shape[1]))
                                actionFormattedTesting = actionFormattedTesting - self.Ymean
                                actionFormattedTesting = actionFormattedTesting/self.Ystd#
                                actionFormattedTesting = actionFormattedTesting[:,None].T
                                     
                                return  actionData, actionDataZeroPad, actionFormattedTesting, timeData                              
                                #actionCount += 1
                    print "Zeros found: " + str(zeroCount)

                # Reject if data size gets too big!
                if (dataCount==10000):
                    print "WARNING!!!!!!! No actions found found for long period exiting"
                    return 0,0,0,0
       
  
                
                #if (data[0]==1000):
                #    break
        return 0,0,0,0

