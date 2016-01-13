#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Olivier Sigaud

Module: NeuralNet

Description: A NeuralNet to be trained as the arm controller
'''

import numpy as np
from pybrain.datasets            import SupervisedDataSet
from pybrain.utilities           import percentError
from pybrain.tools.shortcuts     import buildNetwork
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.structure.modules   import SoftmaxLayer

from Utils.CartesianProduct import cartesian

class NeuralNet():
    
    def __init__(self, inputDim, outputDim):
        '''
	Initializes class parameters
	
	Input:   

        '''
        self.inputDimension = inputDim
        self.outputDimension = outputDim
        self.net = buildNetwork(inputDim, 5, outputDim)
        self.ds = SupervisedDataSet(self.inputDimension, self.outputDimension)

        print "dimensions : " + str(self.inputDimension) + "x" +  str(self.outputDimension)

    def setTheta(self, theta):
        self.net._setParameters(theta)

    def getTheta(self):
        return self.net.params

    def loadTheta(self,thetaFile):
        self.net._setParameters(np.loadtxt(thetaFile))
        #print ("theta LOAD : ", self.net.params)
        return self.net.params

    def saveTheta(self,fileName):
        '''
        Records theta under numpy format
        
        Input:    -fileName: name of the file where theta will be recorded
        '''
        np.savetxt(fileName, self.net.params)

    def getTrainingData(self, inputData, outputData):
        '''
        Verifies the validity of the given input and output data
        Data should be organized by columns
        
        Input:      -inputdata, numpy N-D array
                    -outputData, numpy N-D array
        '''
        self.inputData = inputData
        self.outputData = outputData

        #Getting input and output dimensions and number of sample
        numberOfInputSamples = len(inputData)
        numberOfOutputSamples = len(outputData)
        #check if there are the same number of samples for input and output data
        assert(numberOfInputSamples == numberOfOutputSamples), "Number of samples not equal for output and input"
        #check dimensions

        assert(len(inputData[0]) == self.inputDimension), "Mismatch in input dimension"
        assert(len(outputData[0]) == self.outputDimension), "Mismatch in output dimension"

        self.numberOfSamples = numberOfInputSamples
        self.minInputData = np.min(self.inputData, axis = 0)
        self.maxInputData = np.max(self.inputData, axis = 0)

        for i in range(self.numberOfSamples):
            self.ds.addSample(inputData[i],outputData[i])

    def train(self):
        '''
        Perform batch regression
        '''
        trainer = BackpropTrainer(self.net, self.ds)
        trainer.train()

    def computeOutput(self, inputVal):
        '''
        Returns the output depending on the given input and theta
        
        Input:      -inputVal: numpy N-D array
                    -theta: numpy N-D array
        
        Output:     -fa_out: numpy N-D array, output approximated
        '''
        assert(inputVal.shape[0]==self.inputDimension), "NeuralNet: Bad input format"
        return self.net.activate(inputVal)

