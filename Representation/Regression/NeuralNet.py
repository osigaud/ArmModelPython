#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Olivier Sigaud

Module: NeuralNet

Description: A NeuralNet to be trained as the arm controller
'''

import random as rd
import numpy as np
from pybrain.datasets            import SupervisedDataSet
from pybrain.utilities           import percentError
from pybrain.tools.shortcuts     import buildNetwork
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.structure.modules   import SoftmaxLayer

class NeuralNet():
    
    def __init__(self, inputDim, outputDim):
        '''
	Initializes class parameters
	
	Input:   

        '''
        self.inputDimension = inputDim
        self.outputDimension = outputDim
        self.net = buildNetwork(inputDim, 4, outputDim)
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

    def getTrainingData(self):
        for i in range(200):
            for j in range(200):
                output = []
                x = i/200.0
                y = j/200.0
                if x>0.5 and y>0.5:
                    output.append(rd.random()/2+1.5)
                else:
                    output.append(rd.random()/2)
                self.ds.addSample([x, y],output)

    def train(self):
        '''
        Perform batch regression
        '''
        self.getTrainingData()
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

