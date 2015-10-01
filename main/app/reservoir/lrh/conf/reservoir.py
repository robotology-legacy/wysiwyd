# -*- coding: utf-8 -*-
"""
Created on 4 july 2014
author: Colas DROIN
colas.droin@insa-lyon.fr
"""
#===============================================================================
#    Import librairies
#===============================================================================
### python modules
import numpy as np
#===============================================================================
#    ### Main Class
#===============================================================================
class Reservoir:
    def __init__(self, nb_neurons, spectral_radius,  input_scaling, leak_rate, ridge=10**-5, sparcity=None, w=None, w_in=None, w_out=None ):
        ### parameters
        self.N = nb_neurons
        self.sr = spectral_radius
        self.iss = input_scaling
        self.leak = leak_rate
        self.ridge = ridge
        if sparcity is None:    self.sparcity = 10.0/self.N #10 connection per neuron in average
        else:   self.sparcity = sparcity
        self.w = w
        self.w_in = w_in
        self.w_out = w_out

    #generate the main matrix of the reservoir
    def generate_w(self):
    #TODO : replace current matrix with scipy sparce matrix to improve performance
        if self.w is None:
            self.w = np.random.random((self.N,self.N ))-0.5
            self.w *= np.absolute(np.random.random((self.N,self.N ))) < self.sparcity
            self.w *= self.sr / max(abs(np.linalg.eig(self.w)[0]))

    #generate the input matrix of the reservoir
    def generate_w_in(self, w_in, dim_input):
        if self.w_in is None:
            self.w_in = np.random.random((self.N,dim_input+1 ))
            self.w_in *= self.iss

    #computation of the weight of w_out
    def tikhonov_regularization(self, X_tot, Yt, dim_input):
        #concatenating of the signals of the different meanings and different sentences
        X_c=np.concatenate(X_tot, axis=1)
        Yt_c=np.concatenate(Yt, axis=0)
        Yt_c=Yt_c.T        
        X_c_T = X_c.T      
        self.w_out = np.dot( np.dot(Yt_c,X_c_T), np.linalg.inv( np.dot(X_c,X_c_T) + self.ridge*np.eye(1+dim_input+self.N) ) )
#------------------------------------------------------------------------------ 
    #training of the reservoir
    def train(self, meanings_input_train, teacher):
        ###matrix initialization
        self.generate_w()
        self.generate_w_in(self, meanings_input_train[0].shape[1])
        X_tot=[] #X_tot will contain the internal states of the reservoir for all sentences and all timesteps
        ###X computation
        for u in meanings_input_train:
            #X will contain all the internal states of the reservoir for all timesteps
            X=np.zeros((1+meanings_input_train[0].shape[1]+self.N, meanings_input_train[0].shape[0]))
            for t in range(meanings_input_train[0].shape[0]):
                if t==0:
                    #x or x_prev contain the internal state of the reservoir for one timestep    
                    x_prev=np.array([0]*self.w.shape[0])
                else: 
                    x_prev=x
                x_temp=np.tanh(np.dot(self.w_in,np.concatenate(([1],u[t]), axis=0))+np.dot(self.w,x_prev))
                x=(1-self.leak)*x_prev+self.leak*x_temp
                X[:,t]=np.concatenate( ([1],u[t],x),axis=0)
            X_tot.append(X)    
        ###Tikhononv regularization    
        self.tikhonov_regularization(X_tot,teacher, meanings_input_train[0].shape[1])
        ###output_signal computation
        sentences_output_train=[]
        for X in X_tot:
            sentences_output_train.append(np.dot(self.w_out,X).T)
        return sentences_output_train, X_tot

    #testing of the reservoir
    #this function works exactly like the training function except w_out is already known
    #so there is no Thikhonov regularization
    def test(self, meanings_input_test):
        sentences_output_test=[]
        X_tot=[]
        for num_meaning,u in enumerate(meanings_input_test):
            X=np.zeros((1+meanings_input_test[0].shape[1]+self.N, meanings_input_test[0].shape[0]))
            for t in range(meanings_input_test[0].shape[0]):
                if t==0:    
                    x_prev=np.array([0]*self.w.shape[0])
                    x_temp=np.tanh(np.dot(self.w_in,np.concatenate(([1],u[t]),axis=0))+np.dot(self.w,x_prev))
                    x=(1-self.leak)*x_prev+self.leak*x_temp
                    X[:,t]=np.concatenate( ([1],u[t],x),axis=0)
                    y=np.array(  np.dot(self.w_out , np.concatenate( ([1],u[t],x), axis=0 )),  ndmin=2 )
                    sentences_output_test.append( y  )
                else: 
                    x_prev=x
                    x_temp=np.tanh(np.dot(self.w_in,np.concatenate(([1],u[t]),axis=0))+np.dot(self.w,x_prev))
                    x=(1-self.leak)*x_prev+self.leak*x_temp
                    X[:,t]=np.concatenate( ([1],u[t],x),axis=0)
                    y=np.array(  np.dot(self.w_out , np.concatenate( ([1],u[t],x), axis=0 )),  ndmin=2 )
                    sentences_output_test[num_meaning]=np.concatenate( (sentences_output_test[num_meaning],  y  ) , axis=0 )
            X_tot.append(X)
        return sentences_output_test, X_tot  


