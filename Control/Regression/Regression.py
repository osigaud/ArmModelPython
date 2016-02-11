#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: Regression

Description: 
'''

import numpy as np

class regression(object) :
    
    def __init__(self, rs):
        '''
        Initializes class parameters

        '''
        self.inputDimension = rs.inputDim
        self.outputDimension = rs.outputDim
        print ("dimensions : " + str(self.inputDimension) + "x" +  str(self.outputDimension))

    
    
    def setTheta(self, theta):
        '''
            set a new theta vector
            surcharge this methode if our theta is not a vector
        '''
        self.theta = theta

    def getTheta(self):
        return self.theta

    def load(self,fileName):
        '''
        load a regression previously train
        '''
        raise NotImplementedError("load not implemented")

    def saveTheta(self,fileName):
        '''
        Records theta under numpy format
        
        Input:    -fileName: name of the file where theta will be recorded
        '''
        print(self.getTheta())
        np.savetxt(fileName, self.getTheta())

    def getTrainingData(self, inputData, outputData):
        '''
        Verifies the validity of the given input and output data
        Data should be organized by columns
        
        Input:      -inputdata, numpy N-D array
                    -outputData, numpy N-D array
        '''
        self.inputData = inputData
        self.outputData = outputData

        #Getting input and output dimensions and number of samples
        numberOfInputSamples = len(inputData)
        numberOfOutputSamples = len(outputData)
        #check if there are the same number of samples for input and output data
        assert(numberOfInputSamples == numberOfOutputSamples), "Number of samples not equal for output and input, " + str(numberOfInputSamples)+"/"+str(numberOfOutputSamples)
        #check dimensions
        assert(len(inputData[0]) == self.inputDimension), "Mismatch in input dimension" + str(len(inputData[0]))+"/"+str(self.inputDimension)
        assert(len(outputData[0]) == self.outputDimension), "Mismatch in output dimension" + str(len(outputData[0]))+"/"+str(self.outputDimension)

        self.numberOfSamples = numberOfInputSamples
        self.minInputData = np.min(self.inputData, axis = 0)
        self.maxInputData = np.max(self.inputData, axis = 0)

    def computeOutput(self, inputVal):
        '''
        Returns the output depending on the given input and theta
        
        Input:      -inputVal: numpy N-D array
                    -theta: numpy N-D array
        
        Output:     -fa_out: numpy N-D array, output approximated
        '''
        raise NotImplementedError("computeOutput not implemented")

    def train(self):
        raise NotImplementedError("train not implemented")

def UnitTest():
    raise NotImplementedError("UnitTest not implemented")
