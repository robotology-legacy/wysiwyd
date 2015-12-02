#!/usr/bin/python

#
#The University of Sheffield
#WYSIWYD Project
#
#Example of implementation of SAM class
#
#Created on 29 May 2015
#
#@authors: Uriel Martinez, Luke Boorman, Andreas Damianou
#
#

import matplotlib.pyplot as plt
from SAM.SAM_Drivers import SAMDriver_faces
import pylab as pb
import sys
import pickle
import os
import numpy
import time
import operator
from ConfigParser import SafeConfigParser
try:
    import yarp
    yarpRunning = True
except ImportError:
    print 'WARNING! Yarp was not found! Switching to offline mode'
    yarpRunning = False





######################################  READ CONFIG AND DATA ########################################
#####################################################################################################

# Check configuration file
parser = SafeConfigParser()

file_candidates = ["config_faces.ini"]
section_candidates = ["config_options"]

configData = False

for loc in os.curdir, os.path.expanduser("~"), os.environ.get("WYSIWYD_DIR")+"/share/wysiwyd/contexts/visionDriver":
    print loc
    try: 
        found = parser.read(os.path.join(loc,file_candidates[0]))
        if not found:
            pass
        else:
            print os.path.join(loc,file_candidates[0])
            if( parser.has_section(section_candidates[0]) == True ):
                dataPath = parser.get(section_candidates[0], 'data_path')
                modelPath = parser.get(section_candidates[0], 'model_path')
                participantList = parser.get(section_candidates[0], 'participants')
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

# Hack: Sometimes the participant list is returned as a list of letters, instead of names. 
# Check if this is the case and, if yes, fix.
if participantList.index(","):
    participantList = participantList[:].split(',')

print dataPath
print modelPath
print participantList

imgHNew = 200
imgWNew = 200

# Creates a SAMpy object
mySAMpy = SAMDriver_faces(True, imgH = 400, imgW = 400, imgHNew = imgHNew, imgWNew = imgWNew,inputImagePort="/visionDriver/image:o")

# Specification of the experiment number
experiment_number = 1#45

# Location of face data
#root_data_dir="/home/icub/dataDump/actionFilm"
root_data_dir=dataPath
# Image format
image_suffix=".ppm"
# Array of participants to be recognised
#participant_index=('Andreas','Uriel','Tony','Daniel')
participant_index=participantList
# Poses used during the data collection
pose_index=['Seg']
# Use a subset of the data for training
Ntr=450

# Pose selected for training
pose_selection = 0

# Specification of model type and training parameters
model_type = 'bgplvm'#'mrd'
model_num_inducing = 40
model_num_iterations = 200
model_init_iterations = 1200
#fname = 'm_' + model_type + '_exp' + str(experiment_number) #+ '.pickle'
fname = modelPath + '/models/mFaces_' + model_type + '_exp' + str(experiment_number) #+ '.pickle'


# Enable to save the model and visualise GP nearest neighbour matching
save_model=True
economy_save = True # ATTENTION!! This is still BETA!!
visualise_output=True








#####################################  SET-UP and TRAIN #############################################
#####################################################################################################


# Reading face data, preparation of data and training of the model
mySAMpy.readData(root_data_dir, participant_index, pose_index)
mySAMpy.prepareFaceData(model_type, Ntr, pose_selection,randSeed=experiment_number)
mySAMpy.training(model_num_inducing, model_num_iterations, model_init_iterations, fname, save_model, economy_save)






#####################################  VISUALISATION    #############################################
#####################################################################################################


# This is for visualising the mapping of the test face back to the internal memory
if visualise_output: 
    ax = mySAMpy.SAMObject.visualise()
    visualiseInfo=dict()
    visualiseInfo['ax']=ax
    ytmp = mySAMpy.SAMObject.recall(0)
    ytmp = numpy.reshape(ytmp,(mySAMpy.imgHeightNew,mySAMpy.imgWidthNew))
    fig_nn = pb.figure()
    pb.title('Training NN')
    pl_nn = fig_nn.add_subplot(111)
    ax_nn = pl_nn.imshow(ytmp, cmap=plt.cm.Greys_r)
    pb.draw()
    pb.show()
    visualiseInfo['fig_nn']=fig_nn
else:
    visualiseInfo=None






###################################  PROACTIVE TAGGING ##############################################
#####################################################################################################
if model_type == 'bgplvm':
    print('\n\n\n\n##### PROACTIVE TAGGING...\n\n\n\n')
    import SAM
    pb.figure()
    labels_est=SAM.SAM_Core.latent_cluster_estimate(mySAMpy.SAMObject, alpha=0.1,plot=True, which_indices=(1,0))
    K_est = len(numpy.unique(labels_est))
    print('> I found ' + str(K_est) + ' different people.')

    # Give the correct number of clusters. TODO: May be done with robot interaction
    print('> How many different people are there?)')
    K=int(raw_input())
    #K = 3 
    pb.figure()
    labels = SAM.SAM_Core.latent_cluster(mySAMpy.SAMObject, n_clusters = K, plot=True, which_indices=(1,0))
    pb.figure()
    (cntr, covars) = SAM.SAM_Core.latent_cluster_centers(mySAMpy.SAMObject, None, labels, 'gaussian', True, (1,0), experiment_number,None)
    Nsamp = 6
    c=1
    figFaces = pb.figure()
    for i in range(K):
        for j in range(Nsamp):
            splot = plt.subplot(K, Nsamp, 0 + c)
            sample_x = numpy.random.multivariate_normal(cntr[i,:],covars[i])
            sample_y = mySAMpy.SAMObject.fantasy_memory(sample_x[None,:])[0]
            ff = numpy.reshape(sample_y, (imgHNew,imgWNew))
            plt.imshow(ff, cmap=plt.cm.Greys_r)
            c+=1
	figFaces.canvas.draw()
	figFaces.canvas.flush_events()
        print('> Who is this person? (TODO, add answer and take response...)')
        name=raw_input()
        pb.title(name)










########################################  TEST ######################################################
#####################################################################################################


# Read and test images from iCub eyes in real-time

#fig_input = pb.figure()
#subplt_input = fig_input.add_subplot(111)

while( True ):
    try:
        if yarpRunning:
	    print "Reading from Camera"
            testFace = mySAMpy.readImageFromCamera()
        else:
            pass
            #TODO mysampy.ytestn
        print "Face data shape 0 " + str(testFace.shape[0]) + " Face data shape 1 " + str(testFace.shape[1])
        #subplt_input.imshow(testFace, cmap=plt.cm.Greys_r)
        mySAMpy.testing(testFace, visualiseInfo)
        #pp = mySAMpy.testing(testFace, visualiseInfo)
        #time.sleep(0.5)
        #l = pp.pop(0)
        #l.remove()
        #pb.draw()
        #del l
        pb.waitforbuttonpress(0.1)
        
    except KeyboardInterrupt:
        print 'Interrupted'
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)

