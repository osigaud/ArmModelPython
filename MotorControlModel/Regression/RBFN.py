#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: RBFN

Description: We find here functions which allow to compute a RBFN regression
'''

import numpy as np
from Utils.CartesianProduct import cartesian

class rbfn():
    
    def __init__(self, nbFeatures, inputDim, outputDim):
        '''
	Initializes class parameters
	
	Input:     -nbFeature: int, number of feature in order to perform the regression

        '''
        self.nbFeat = nbFeatures
        self.inputDimension = inputDim
        self.outputDimension = outputDim
        print "dimensions : " + str(self.inputDimension) + "x" +  str(self.outputDimension)
        self.theta = np.zeros((self.nbFeat, self.outputDimension))

    def setTheta(self, theta):
        self.theta = theta

    def loadTheta(self,thetaFile):
        self.theta = np.loadtxt(thetaFile)
        #print ("theta LOAD : ", self.theta)
        return self.theta

    def saveTheta(self,fileName):
        '''
        Records theta under numpy format
        
        Input:    -fileName: name of the file where theta will be recorded
        '''
        np.savetxt(fileName, self.theta)

    def saveFeatures(self,fileName):
        '''
        Records the RBFN structure under numpy format
        
        Input:    -fileName: name of the file where theta will be recorded
        '''
        struct = []
        struct.append(self.minInputData)
        struct.append(self.maxInputData)
        np.savetxt(fileName,struct)

    def loadFeatures(self,filename):
        struct = np.loadtxt(filename)
        self.minInputData = struct[0]
        self.maxInputData = struct[1]
        self.setCentersAndWidths()

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
        assert(numberOfInputSamples == numberOfOutputSamples), "Number of samples not equal for output and input"
        #check dimensions
        assert(len(inputData[0]) == self.inputDimension), "Mismatch in input dimension"
        assert(len(outputData[0]) == self.outputDimension), "Mismatch in output dimension"

        self.numberOfSamples = numberOfInputSamples
        self.minInputData = np.min(self.inputData, axis = 0)
        self.maxInputData = np.max(self.inputData, axis = 0)
           
    def setCentersAndWidths(self):
        '''
        Sets the centers and widths of Gaussian features.
        Uses linspace to evenly distribute the features.
        '''
        #get max and min of the input data
        rangeForEachDim = self.maxInputData - self.minInputData
        #set the sigmas
        widthConstant = 2.0*rangeForEachDim / self.nbFeat
        #create the diagonal matrix of sigmas to compute the gaussian
        self.widths = np.diag(widthConstant)
         #coef for Gaussian features
        self.norma = 1/np.sqrt(((2*np.pi)**self.inputDimension)*np.linalg.det(self.widths))
        #print ("RBFN : constante de normalisation : ", self.norma)
        self.invcovar = np.linalg.pinv(self.widths)
        linspaceForEachDim = []
        #set the number of gaussian used and allocate them in each dimensions
        for i in range(self.inputDimension):
            linspaceForEachDim.append(np.linspace(self.minInputData[i], self.maxInputData[i], self.nbFeat))
            #get matrix with all the possible combinations to find each centers
        self.centersInEachDimensions = cartesian(linspaceForEachDim)
        self.nbFeatures = len(self.centersInEachDimensions)
        print("nbfeatures:", self.nbFeatures)

    def train_rbfn(self):
        '''
        Perform batch regression
        '''
        self.theta = []
        for i in range(self.outputDimension):
            K = []
            for val in self.inputData:
                 K.append(self.computeAllWeights(val))
            Kmat = np.matrix(K).T
            A = np.dot(Kmat, Kmat.T)
            #inv = np.linalg.pinv(A)
            vec = self.outputData.T
            y = np.array(vec[i].T)
            b = np.dot(Kmat, y).T            
            #self.theta.append(np.dot(inv,b))            
            self.theta.append(np.linalg.lstsq(A,b))

    def train_reg_rbfn(self, lamb):
        '''
        Perform batch regularized regression
        '''
        self.theta = []
        for i in range(self.outputDimension):
            K = []
            for val in self.inputData:
                 K.append(self.computeAllWeights(val))
            Kmat = np.matrix(K).T
            A = np.dot(Kmat, Kmat.T)
            B = lamb*np.identity(np.shape(A)[0])
            C = A + B
            #inv = np.linalg.pinv(C)
            vec = self.outputData.T
            y = np.array(vec[i].T)
            b = np.dot(Kmat, y).T
            #self.theta.append(np.dot(inv,b))
            self.theta.append(np.linalg.lstsq(C,b))
    
    def computeFeatureWeight(self, inputVal, gauss):
        '''
        Computes the value of an input with respect to one Gaussian feature
        
        Input:     -inputVal: one point in the input space (an input vector)
        
        Output:    -phi: a number: the value of phi(inputVal)
        '''
        xu = inputVal - gauss
        xus = np.dot(xu, self.invcovar)
        xg = np.dot(xus,xu.T)
        phi = self.norma*np.exp(-0.5*xg)
        return phi

    def computeAllWeights(self, inputVal):
        '''
        Computes the value of an input with respect to all Gaussian features
        
        Input:     -inputVal: one point in the input space (an input vector)
        
        Output:    -phi: a vector of values of all phi(x) for the input x and all phi
        '''
        retour = []
        for x in self.centersInEachDimensions:
            phi = self.computeFeatureWeight(inputVal, x)
            retour.append(phi)
        return retour

    def computeOutput(self, inputVal):
        '''
        Returns the output depending on the given input and theta
        
        Input:      -inputVal: numpy N-D array
                    -theta: numpy N-D array
        
        Output:     -fa_out: numpy N-D array, output approximated
        '''
        assert(inputVal.shape[0]==self.inputDimension), "RBFN: Bad input format"
        output = []
        for i in range(self.outputDimension):
            tmp = self.computeAllWeights(inputVal)
            output.append(np.dot(tmp, self.theta[i]))
        return output
