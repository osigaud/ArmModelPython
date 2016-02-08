#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Olivier Sigaud

Module: NeuralNet

Description: A NeuralNet to be trained as the arm controller
'''
import random
import numpy as np
from pybrain.datasets            import SupervisedDataSet
from pybrain.utilities           import percentError
from pybrain.tools.shortcuts     import buildNetwork
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.structure.modules   import SoftmaxLayer
from pybrain.structure.modules   import LinearLayer, SigmoidLayer, TanhLayer
from pybrain.structure           import FullConnection
from pybrain.structure           import FeedForwardNetwork

from Utils.CartesianProduct import cartesian
from Regression import *

class NeuralNet(regression):
    
    def __init__(self, inputDim, outputDim):
        '''
	Initializes class parameters
	
	Input:   

        '''
        regression.__init__(self,inputDim, outputDim)
        #self.net = buildNetwork(inputDim, 4, 4, outputDim)
        self.net = FeedForwardNetwork()
        inLayer = LinearLayer(inputDim)
        hiddenLayer1 = TanhLayer(10)
        hiddenLayer2 = TanhLayer(10)
        outLayer = SigmoidLayer(outputDim)
        self.net.addInputModule(inLayer)
        self.net.addModule(hiddenLayer1)
        self.net.addModule(hiddenLayer2)
        self.net.addOutputModule(outLayer)

        in_to_hidden1 = FullConnection(inLayer, hiddenLayer1)
        hidden1_to_hidden2=FullConnection(hiddenLayer1,  hiddenLayer2)
        hidden2_to_out = FullConnection(hiddenLayer2, outLayer)
        self.net.addConnection(in_to_hidden1)
        self.net.addConnection(hidden1_to_hidden2)
        self.net.addConnection(hidden2_to_out)

        self.net.sortModules()

        self.ds = SupervisedDataSet(self.inputDimension, self.outputDimension)
        

    def setTheta(self, theta):
        self.net._setParameters(theta)

    def getTheta(self):
        return self.net.params

    def load(self,thetaFile):
        '''
        load wheight of the neural network from the thetafile
        '''
        self.net._setParameters(np.loadtxt(thetaFile+".theta"))
        #print ("theta LOAD : ", self.net.params)
        return self.net.params

    def getTrainingData(self, inputData, outputData):
        '''
        Verifies the validity of the given input and output data
        Data should be organized by columns
        
        Input:      -inputdata, numpy N-D array
                    -outputData, numpy N-D array
        '''
        regression.getTrainingData(self,inputData, outputData)

        for i in range(self.numberOfSamples):
            self.ds.addSample(inputData[i],outputData[i])

    def train(self):
        '''
        Perform batch regression
        '''
        trainer = BackpropTrainer(self.net, self.ds, learningrate=0.0005, momentum=0.9)
        try:
            while(True):
                print(trainer.train())
        except:
            print("NeuralNet L71")
        
        #trainer.trainUntilConvergence(maxEpochs=10, verbose=True)
        #trainer.trainEpochs(10)
    def computeOutput(self, inputVal):
        '''
        Returns the output depending on the given input and theta
        
        Input:      -inputVal: numpy N-D array
                    -theta: numpy N-D array
        
        Output:     -fa_out: numpy N-D array, output approximated
        '''
        assert(inputVal.shape[0]==self.inputDimension), "NeuralNet: Bad input format"
        output=self.net.activate(inputVal)
        #print(output)
        return output




def UnitTest():
    fa = NeuralNet(2,3)
    input, output = [], []
    for i in range(10000):
        x,y = 3*random.random(), 3*random.random()
        input.append([x,y])
        output.append([x*y, x-y, x+y])
    fa.getTrainingData(np.vstack(np.array(input)), np.vstack(np.array(output)))
    fa.train()
    fa.saveTheta("test.theta")

    fa.load("test")
    for i in range(20):
        x,y = 3*random.random(), 3*random.random()
        approx = fa.computeOutput(np.array([x,y]))
        print("in:", [x,y])
        print(" out:", approx)
        print(" real:",  [x*y, x-y, x+y])
  
