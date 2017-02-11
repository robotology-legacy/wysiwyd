#!/usr/bin/python

#""""""""""""""""""""""""""""""""""""""""""""""
#The University of Sheffield
#WYSIWYD Project
#
#SAMpy class for implementation of SAM module
#
#Created on 26 May 2015
#
#@authors: Uriel Martinez, Luke Boorman, Andreas Damianou
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
from scipy.signal import lfilter
from scipy.io import wavfile, loadmat
from scipy.fftpack import dct
from sklearn.mixture import GMM
try:
    import yarp
    isYarpRunningGlobal = True
except ImportError:
    isYarpRunningGlobal = False
import cv2
import GPy
import time
from scipy.spatial import distance
import operator
"""
try:
    from SAM import SAM
except ImportError:
    import SAM
"""
import time
from SAM.SAM_Core import SAMDriver

# """""""""""""""""""""""""""""""""""

# Changes still necessary
# port addresses

# """"""""""""""""""""""""""""""""""


#""""""""""""""""
#Class developed for the implementation of the face recognition task in real-time mode.
#""""""""""""""""

class SAMDriver_speech(SAMDriver):

#""""""""""""""""
#Initilization of the SAM class
#Inputs:
#    - isYarprunning: specifies if yarp is used (True) or not(False)
#    - imgH, imgW: original image width and height
#    - imgHNewm imgWNew: width and height values to resize the image
#
#Outputs: None
#""""""""""""""""
    # ---------------------------------Change----------------------------------------#
    def __init__(self, isYarpRunning = False, delta=False, context=2, n_mixtures=25, gmm_atts=None, inputImagePort="/visionDriver/image:o", openPorts=True):
    # -------------------------------------------------------------------------------#
        # Call parent class init
        if not isYarpRunningGlobal:
            # if I can't find Yarp, I'll overwrite the given flag
            isYarpRunning = False
        SAMDriver.__init__(self, isYarpRunning)

        # Extra stuff needed for the specific driver
        self.inputImagePort=inputImagePort

        self.delta = delta    
        self.context = context
        self.n_mixtures = n_mixtures
        self.file_suffix=".wav"
        self.verbose_L=0

        if not gmm_atts==None:
            self.gmm_data = gmm_atts
        else:
            self.gmm_data = {}

        self.participant_index = None

        if( isYarpRunning == True and openPorts == True):
            yarp.Network.init()
            self.createPorts()
            self.openPorts()
            self.createImageArrays()

# ------------------------------------Change-------------------------------------------#

    #""""""""""""""""
    #Methods to create the ports for reading images from iCub eyes
    #Inputs: None
    #Outputs: None
    #""""""""""""""""
    def createPorts(self):
        if self.isYarpRunning:
            self.imageDataInputPort = yarp.BufferedPortImageRgb()
            self.outputFacePrection = yarp.Port()
            self.speakStatusPort = yarp.RpcClient();
            self.speakStatusOutBottle = yarp.Bottle()
            self.speakStatusInBottle = yarp.Bottle()
            self.imageInputBottle = yarp.Bottle()

    #""""""""""""""""
    #Method to open the ports. It waits until the ports are connected
    #Inputs: None
    #Outputs: None
    #""""""""""""""""
    def openPorts(self):
        if self.isYarpRunning:
            print "open ports"
            self.imageDataInputPort.open("/sam/face/imageData:i");
            self.outputFacePrection.open("/sam/face/facePrediction:o")
            self.speakStatusPort.open("/sam/face/speakStatus:i")
            self.speakStatusOutBottle.addString("stat")

        #print "Waiting for connection with imageDataInputPort..."
#        while( not(yarp.Network.isConnected(self.inputImagePort,"/sam/imageData:i")) ):
#            print "Waiting for connection with imageDataInputPort..."
#            pass

    def closePorts(self):
        if self.isYarpRunning:
            print "open ports"
            self.actionDataInputPort.close();
            self.outputActionPrediction.close()
            self.speakStatusPort.close()
        #self.speakStatusOutBottle.addString("stat")
        #print "Waiting for connection with actionDataInputPort..."
        #while( not(yarp.Network.isConnected(self.inputActionPort,"/sam/actionData:i")) ):
        #    print "Waiting for connection with actionDataInputPort..."
        #    pass

# ------------------------------------------------------------------------------------#

#---- Extra methods for the specific driver.

#""""""""""""""""
#Method to test the learned model with faces read from the iCub eyes in real-time
#Inputs:
#    - testFace: image from iCub eyes to be recognized
#    - visualiseInfo: enable/disable the result from the testing process
#
#Outputs:
#    - pp: the axis of the latent space backwards mapping
#"""""""""""""""

    def testing(self, testUtt, choice, visualiseInfo=None):
        pass

    def dist(self, result, model):
        ret=self.SAMObject.pattern_completion(result[None,:], visualiseInfo=None)
        dists = numpy.zeros(model.model.X.shape[0])
        for idx,x in enumerate(model.model.X.mean):
            dists[idx] = distance.euclidean(x,ret[0])
        return numpy.vstack(dists)
#""""""""""""""""
#Method to read face data previously collected to be used in the traning phase.
#Here the loaded data is preprocessed to have the correct image size and one face per image.
#Inputs:
#    - root_data_dir: location of face data
#    - participant_index: array of participants names
#    - pose_index: array of poses from the face data collected
#
#Outputs: None
#""""""""""""""""
    def readData(self, root_data_dir, participant_index, pose_index):
        self.Y
        self.L
        self.verbose_L
        self.participant_index = participant_index

        if not os.path.exists(root_data_dir):
            print "CANNOT FIND:" + root_data_dir
        else:
            print "PATH FOUND"

	    ## Find and build index of available images.......

        data_file_count=numpy.zeros([len(self.participant_index),len(pose_index)])                                                  # Number of images for each participant (and pose)
        data_file_database={}
        for count_participant, current_participant in enumerate(self.participant_index):                                            # For each participant
            data_file_database_part={}
            for count_pose, current_pose in enumerate(pose_index):                                                                  # For each pose
                current_data_dir=os.path.join(root_data_dir,current_participant+current_pose)
                data_file_database_p=numpy.empty(0,dtype=[('orig_file_id','a11'),('file_id','i2'),('speech_fname','a100')])
                data_utt_count=0
                if os.path.exists(current_data_dir):                                                                                # Check participant folder exists
                    for file in os.listdir(current_data_dir):                                                                       # For each file in said folder
	                    #parts = re.split("[-,\.]", file)
                        fileName, fileExtension = os.path.splitext(file)
                        if fileExtension==self.file_suffix:                                                                         # Check for speech file
                            file_ttt=numpy.empty(1, dtype=[('orig_file_id','a11'),('file_id','i2'),('speech_fname','a100')])
                            file_ttt['orig_file_id'][0]=fileName                                                                    # Index file with file ID,
                            file_ttt['speech_fname'][0]=file                                                                        # file name and 
                            file_ttt['file_id'][0]=data_utt_count                                                                   # count of file in search
                            data_file_database_p = numpy.append(data_file_database_p,file_ttt,axis=0)
                            data_utt_count += 1
                    data_file_database_p=numpy.sort(data_file_database_p,order=['orig_file_id'])                                    # Sort before inserting
                data_file_database_part[pose_index[count_pose]]=data_file_database_p
                data_file_count[count_participant,count_pose]=len(data_file_database_p)
            data_file_database[self.participant_index[count_participant]]=data_file_database_part                                   # Add the sections to the main database

	    # To access use both dictionaries data_file_database['Luke']['LR']
	    # Cutting indexes to smllest number of available files -> Using file count
        min_no_utts=int(numpy.min(data_file_count))

	    # Data size
        print "Found minimum number of utterances:" + str(min_no_utts)
        print "Utterance count:", data_file_count

# -------------------------------------New---------------------------------------------#

        utt_data=numpy.zeros([self.n_mixtures*len(participant_index), min_no_utts, len(self.participant_index),len(pose_index)])
        utt_label_data=numpy.zeros([self.n_mixtures*len(participant_index), min_no_utts, len(self.participant_index),len(pose_index)],dtype=int)
        verb_label_data=numpy.zeros([1, min_no_utts, len(self.participant_index),len(pose_index)],dtype='a11')

        for count_pose, current_pose in enumerate(pose_index): # ----------------------------------------------------------------- # For each pose
            tmp_utt_data = {}
            for count_participant, current_participant in enumerate(self.participant_index): # ------------------------------------ # For each participant
                tmp_utt_data[current_participant] = {}
                for current_utt in range(min_no_utts): # -------------------------------------------------------------------------- # For each utt
                    tmp_utt_data[current_participant][current_utt] = {}
                    # Read in the file as a wav
                    current_utt_path=os.path.join(os.path.join(root_data_dir,self.participant_index[count_participant]+
                        pose_index[count_pose]+"/"+
                        data_file_database[self.participant_index[count_participant]][pose_index[count_pose]][current_utt][2]))
                    data_utt = readFromFile(current_utt_path, delta=self.delta, context=self.context, spectrum_power=1)
                    tmp_utt_data[current_participant][current_utt]['mfcc'] = data_utt

                # ----------------------------------------------------------------------------------------------------------------- # End utt
                
                # Train the GMM on this participants
                par_gmm = GMM(n_components=self.n_mixtures,covariance_type='full',n_iter=5)                                                      
                tr_dat = numpy.vstack(x['mfcc'] for x in tmp_utt_data[current_participant].values())
                print tr_dat.shape, current_participant
                self.gmm_data[current_participant] = par_gmm.fit(tr_dat)

            # --------------------------------------------------------------------------------------------------------------------- # End participant
        
        # ------------------------------------------------------------------------------------------------------------------------- # End pose
        
        # Make Supervectors for each utterance for each participant
        print 'Training Speaker GMMs'
        for count_pose, current_pose in enumerate(pose_index): # ----------------------------------------------------------------- # For each pose
            for count_participant, current_participant in enumerate(self.participant_index): # ------------------------------------ # For each participant
                print 'GMM for', current_participant
                for current_utt in range(min_no_utts): # -------------------------------------------------------------------------- # For each utt
                    # For each of the gmms:
                    tmp = []
                    for g in self.gmm_data.values():
                        # Find the average posteriors for each mixture's mean for each frame of the utterance.
                        tmp.append(numpy.mean(g.predict_proba(tmp_utt_data[current_participant][current_utt]['mfcc']),axis=0))
                    utt_data[:, current_utt, count_participant, count_pose] = numpy.hstack(tmp)
                    utt_label_data[:,current_utt,count_participant,count_pose]=numpy.zeros(self.n_mixtures*len(participant_index),dtype=int)+count_participant
                    tmp = data_file_database[current_participant][current_pose][current_utt]['orig_file_id']
                    verb_label_data[0,current_utt,count_participant,count_pose] = tmp


        self.Y=utt_data
        self.L=utt_label_data
        self.verbose_L = verb_label_data
        print self.Y.shape

# -------------------------------------------------------------------------------------#

#""""""""""""""""
#Method to process some important features from the face data required for the classification model such as mean and variance.
#Inputs:
#    - model: type of model used for the SAM object
#    - Ntr: Number of training samples
#    - pose_selection: participants pose used for training of the SAM object
#
#Outputs: None
#""""""""""""""""
    def prepareData(self, model='mrd', Ntr = 50, pose_selection = 0, randSeed=0):    
        #""--- Now Y has 4 dimensions: 
        #1. Pixels
        #2. Images
        #3. Person
        #4. Movement (Static. up/down. left / right)     
        #
        #We can prepare the face data using different scenarios about what to be perceived.
        #In each scenario, a different LFM is used. We have:
        #- gp scenario, where we regress from images to labels (inputs are images, outputs are labels)
        #- bgplvm scenario, where we are only perceiving images as outputs (no inputs, no labels)
        #- mrd scenario, where we have no inputs, but images and labels form two different views of the output space.
        #
        #The store module of the LFM automatically sees the structure of the assumed perceived data and 
        #decides on the LFM backbone to be used.
        #
        #! Important: The global variable Y is changed in this section. From the multi-dim. matrix of all
        #modalities, it turns into the training matrix of image data and then again it turns into the 
        #dictionary used for the LFM.
        #---""" 
# -------------------------------------Change------------------------------------------#

    	# Take all poses if pose selection ==-1
        if pose_selection == -1:
            ttt=numpy.transpose(self.Y,(0,1,3,2))
            ttt=ttt.reshape((ttt.shape[0],ttt.shape[1]*ttt.shape[2],ttt.shape[3])) 
        else:
            ttt=self.Y[:,:,:,pose_selection]
        ttt=numpy.transpose(ttt,(0,2,1))
        self.Y=ttt.reshape(ttt.shape[0],ttt.shape[2]*ttt.shape[1]) 
        self.Y=self.Y.T
        #N=self.Y.shape[0]

        if pose_selection == -1:
            ttt=numpy.transpose(self.L,(0,1,3,2))
            ttt=ttt.reshape((ttt.shape[0],ttt.shape[1]*ttt.shape[2],ttt.shape[3]))
        else:
            ttt=self.L[:,:,:,pose_selection]
        ttt=numpy.transpose(ttt,(0,2,1))
        self.L=ttt.reshape(ttt.shape[0],ttt.shape[2]*ttt.shape[1]) 
        self.L=self.L.T
        self.L=self.L[:,:1]

        ret = SAMDriver.prepareData(self, model, Ntr, randSeed=randSeed)
        return ret

    def pre_process(self, chunk):
        """
        Takes an audio segment and performs pre-processing on it.
        It should be noted that this is designed to be used with real-time audio, and it is assumed that
        the model has already been trained. 
        """
        h = []
        f = feature_extract(chunk, delta=False)
        for g in self.gmm_data.values():
            #print g.means_.shape
            #print f.shape
            h.append(numpy.mean(g.score_samples(f)[1], axis=0))
            #h.append(numpy.mean(g.predict_proba(f), axis=0))
        
        return numpy.hstack(h)
        #return numpy.hstack(numpy.mean(g.predict_proba(feature_extract(chunk)), axis=0) for g in self.gmm_data.values())

# -------------------------------Helper functions-------------------------------------#


def read_file(file_name):
    """
    Reads the given file and returns an array-like
    """
    return wavfile.read(file_name)

def frame(arr, nfft=None, window_size=25, shift=10, hamming=True, sample_rate=8):
    """
    Takes the signal as an array and windows (with window size supplied in ms)
    Also applies a Hamming window unless specified otherwise.
    """
    window_size = window_size * sample_rate 
    shift = shift * sample_rate
    num_windows = numpy.ceil(arr.shape[0] / window_size)
    if nfft == None:
        nfft = window_size*sample_rate
    window = numpy.hamming(window_size)
    x=0
    frames=[]
    while x*shift+window_size < len(arr):
        sh_val = x*shift
        fr_range = range(sh_val,sh_val+window_size)
        frames.append(arr[fr_range])
        x+=1
    frames = numpy.vstack(frames)
    
    if hamming:
        for idx,fr in enumerate(frames):
            frames[idx] = numpy.multiply(window, fr)
            frames[idx] = list(frames[idx])
    return frames

def spectra(y, nfft=None, p=1):
    """Returns an array of spectra for an array of waveform frames"""
    if nfft==None:
        nfft = y.shape[1]+1
    sp=[]
    for fr in y:
        fr = numpy.absolute(numpy.fft.fft(numpy.array(fr)))**p
        sp.append(list(fr))
    return 1.0 / len(sp[0]) * numpy.vstack(sp)

def round_base(x, base, r):
    """Rounds a number, x, to the nearest base"""
    return int(base * r(float(x)/base))

def make_mfccs(spec, energy=None, delta=True, d_context=1, nfilt=25):
    x = filter_banks(spec.shape[1],nfilt)
    feats = numpy.dot(spec,x.T)
    feats = numpy.where(feats == 0,numpy.finfo(float).eps,feats) # if feats is zero, we get problems with log
    feats = dct(numpy.log10(feats), axis=-1, norm='ortho')[:,:13]
    if energy != None:
        feats[:,0] = energy
    if delta:
        delts = make_deltas(feats, context=d_context)
        dd = make_deltas(delts, context=d_context)
        delts = numpy.vstack(numpy.append(delts[i],dd[i]) for i in range(delts.shape[0]))
        feats = numpy.vstack(numpy.append(feats[i],delts[i]) for i in range(feats.shape[0]))
    return feats

def make_deltas(feats, context=1):
    d = numpy.zeros(feats.shape)
    for i in range(feats.shape[0]):
        tmp = numpy.zeros(feats.shape[1])
        for j in range(1,context+1):
            if i-j < 0:
                r = j*feats[i+j]
            elif i+j >= feats.shape[0]:
                r = j*feats[i-j]
            else:
                r = j*calc_range(feats[i+j],feats[i-j])
            tmp = tmp+r
#         tmp = tmp / numpy.sum([i**2 for i in range(1,context+1)])
        d[i,:] = tmp
    return d

def calc_range(x,y):
    return x - y

def readFromFile(current_utt_path, spectrum_power=1, delta=True, context=1):
    data_utt = read_file(current_utt_path)
    return feature_extract(data_utt, spectrum_power=spectrum_power, delta=delta, context=context)

def feature_extract(data, spectrum_power=1, delta=True, context=1):
    # Pre-emphasis
    rate = data[0] / 1000
    try:
        mono_data = data[1].sum(axis=1) / float(data[1].shape[1])
    except:
        mono_data = data[1]
    test_file = preemp(numpy.array(mono_data), 0.97)

    # Frame correctly - Make def with window_size,shift,hamming, etc as options
    test_fr = frame(test_file,sample_rate=rate)

    # Take the fourier transform of the signal - 
    test_sp = spectra(test_fr,p=spectrum_power) # Power Spectrum for each frame
    test_en = numpy.sum(test_sp,axis=1) # Energy for each frame
    test_en = numpy.where(test_en == 0,numpy.finfo(float).eps,test_en) # if energy is zero, we get problems with log
    
    # Get the MFCCs and make the deltas and delta-deltas
    feats = make_mfccs(test_sp, energy=test_en, delta=delta, d_context=context)

    # Add to data set        
    return feats

# -------------------------------------------------------------------------------------------- #
# Adapted from James Lyons' 'python_speech_features' as allowed under MIT License
# https://github.com/jameslyons/python_speech_features

def hz2mel(hz):
    """Convert a value in Hertz to Mels
    :param hz: a value in Hz. This can also be a numpy array, conversion proceeds element-wise.
    :returns: a value in Mels. If an array was passed in, an identical sized array is returned.
    """
    return 2595 * numpy.log10(1+hz/700.0)
#     return 1127.01048 * numpy.log(hz/700 +1)
    
def mel2hz(mel):
    """Convert a value in Mels to Hertz
    :param mel: a value in Mels. This can also be a numpy array, conversion proceeds element-wise.
    :returns: a value in Hertz. If an array was passed in, an identical sized array is returned.
    """
    return 700*(10**(mel/2595.0)-1)
#     return (numpy.exp(mel / 1127.01048) - 1) * 700
    
def preemp(input, p):
    """Pre-emphasis filter."""
    return lfilter([1., -p], 1, input)
#     return numpy.append(input[0],input[1:]-p*input[:-1])

def filter_banks(nfft, nfilt=25, sample_rate=16, lowfreq=0, highfreq=None):
    """
    Creates the mel filterbanks used to create the MFCCs
    :param nfilt:
    :param sample_rate:
    :param lowfreq:
    :param highfreq:
    
    :returns: The filter banks for calculating the Mel-Frequency coefficients
    """
    # Sets highfrequency to half sample rate if not set
    # or too large
    if highfreq==None or highfreq>sample_rate*1000/2:
        highfreq=sample_rate*1000 / 2
    
    # Compute locations for the mel banks
    l_mel = hz2mel(lowfreq)
    h_mel = hz2mel(highfreq)
    mel_banks = numpy.linspace(l_mel,h_mel,nfilt+2) # +2 for the start and end values
    bins = numpy.floor(mel2hz(mel_banks) * (nfft) /(sample_rate*1000))
    
    # Build the filter banks
    fbank = numpy.zeros([nfilt,nfft])
    for j in xrange(0,nfilt):
        for i in xrange(int(bins[j]),int(bins[j+1])):
            fbank[j,i] = (i - bins[j])/(bins[j+1]-bins[j])
        for i in xrange(int(bins[j+1]),int(bins[j+2])):
            fbank[j,i] = (bins[j+2]-i)/(bins[j+2]-bins[j+1])
    return fbank

# -------------------------------------------------------------------------------------------- #