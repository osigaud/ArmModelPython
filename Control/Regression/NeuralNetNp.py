#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: NeuralNet

Description: A eficient fastforward NeuralNet
             Can not be train, use NeuralNetTF for that
'''

import numpy as np


from Regression import regression

def sigmoid(weight):
    s = 1.0 / (1.0 + np.exp(-1.0 * weight))
    return s

def softmax(w):
    maxes = np.amax(w, axis=1)
    maxes = maxes.reshape(maxes.shape[0], 1)
    e = np.exp(w - maxes)
    dist = e / np.sum(e, axis=1)
    return dist

def relu(w):
    return np.maximum(w, 0)

def linear(w):
    return w

layersDict={"linear" : linear, "sigmoid" : sigmoid, "tanh" : np.tanh, "softmax" : softmax, "relu" : relu}



class NeuralNetNp(regression):
    
    
 
 
    def __init__(self, rs):
        regression.__init__(self,rs)
        
        self.activation = []
        self.W=[]
        self.b=[]
        self.size=0
        
        


        
        #no hidden Layer
        if(len(rs.hiddenLayers)==0):
            #connection between input and output Layer
            self.W.append(np.empty((rs.inputDim,rs.outputDim)))
            self.b.append(np.empty(rs.outputDim))
            self.activation.append(layersDict[rs.outputLayer])
            self.size=rs.inputDim*rs.outputDim+rs.outputDim
        else :
            #hidden Layers
            precDim=rs.inputDim
            for layer in rs.hiddenLayers:
                self.W.append(np.empty((precDim,layer[1])))
                self.b.append(np.empty(layer[1]))
                self.activation.append(layersDict[layer[0]])
                precDim=layer[1]
                self.size+=precDim*layer[1]+layer[1]
             
            #outputLayer
            self.W.append(np.empty((precDim,rs.outputDim)))
            self.b.append(np.empty(rs.outputDim))
            self.activation.append(layersDict[rs.outputLayer])
            self.size+=precDim*rs.outputDim+rs.outputDim
            self.theta=np.empty(self.size)

        
            
    def setTheta(self, theta):
        cpt=0
        for i in range(len(self.W)):
            height = self.W[i].shape[0]
            width = self.W[i].shape[1]
            self.W[i] = theta[cpt:cpt+height*width].reshape((height,width))
            cpt+=height*width
            self.b[i] = theta[cpt:cpt+self.b[i].shape[0]]
            cpt+=self.b[i].shape[0]


    def getTheta(self):
        cpt=0
        for bias, weight in self.b, self.W:
            height = weight.shape[0]
            width = weight.shape[1]
            self.theta[cpt:height*width]=weight.reshape(height*width)
            cpt+=height*width
            self.theta[cpt:bias.shape[0]]=bias
            cpt+=bias.shape[0]


    def load(self,thetaFile):
        '''
        load wheight of the neural network from the thetafile
        '''
        self.setTheta((np.loadtxt(thetaFile+".theta")))
        #print ("theta LOAD : ", self.net.params)
        return self.theta

    def getTrainingData(self, inputData, outputData):
        '''
        Verifies the validity of the given input and output data
        Data should be organized by columns
        
        Input:      -inputdata, numpy N-D array
                    -outputData, numpy N-D array
        '''
        regression.getTrainingData(self,inputData, outputData)


    def train(self):
        raise NotImplementedError("This neural Network can not be train")

        
        #trainer.trainUntilConvergence(maxEpochs=10, verbose=True)
        #trainer.trainEpochs(10)
    def computeOutput(self, inputVal):
        '''
        Returns the output depending on the given input and theta
        
        Input:      -inputVal: numpy N-D array
                    -theta: numpy N-D array
        
        Output:     -fa_out: numpy N-D array, output approximated
        '''
        output=inputVal
        for i in range(len(self.activation)):
            output=self.activation[i](output.dot(self.W[i])+self.b[i])
        return output





  
