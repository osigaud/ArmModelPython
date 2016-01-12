#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Olivier Sigaud

Module: RunRegressionNN

Description: Functions to run a neural net to create a controller
'''

import os
import numpy as np
import random as rd

from Utils.ReadSetupFile import ReadSetupFile
from Utils.FileReading import getStateAndCommandData, dicToArray, stateAndCommandDataFromTrajs, loadStateCommandPairsByStartCoords

from Regression.NeuralNet import NeuralNet
from ArmModel.Arm import Arm

from GlobalVariables import BrentTrajectoriesFolder, pathDataFolder

def initNNController(rs):
    '''
	Initializes the controller allowing to compute the output from the input and the vector of parameters theta
	
	Input:		-rs: ReadSetupFile, parameters
	'''
    #Initializes the function approximator
    fa = NeuralNet(rs.inputDim,rs.outputDim)
    return fa

def runNN(name,fromStruct):
    ''' 
    Takes the Brent trajectories as input, shuffles them, and then runs the NN regression algorithm
    '''
    rs = ReadSetupFile()
    #state, command = getStateAndCommandData(BrentTrajectoriesFolder)
    #stateAll, commandAll = dicToArray(state), dicToArray(command)
    #print ("old:", stateAll[0])

    stateAll, commandAll = stateAndCommandDataFromTrajs(loadStateCommandPairsByStartCoords(pathDataFolder + "TrajRepository/"))
    #stateAll, commandAll = stateAndCommandDataFromTrajs(loadStateCommandPairsByStartCoords(BrentTrajectoriesFolder))
    #print ("len:", len(commandAll[0]))
    stateAll = np.vstack(np.array(stateAll))
    commandAll = np.vstack(np.array(commandAll))
    #print ("len global:", len(commandAll))
    #print ("new:", commandAll[0])

    #this works because both shuffles generate the same order, due to the seed
    np.random.seed(0)
    np.random.shuffle(stateAll)
    np.random.seed(0)
    np.random.shuffle(commandAll)

    print("nombre d'echantillons: ", len(stateAll))
    fa = NeuralNet(rs.inputDim,rs.outputDim)
    fa.getTrainingData(stateAll, commandAll)
    fa.train(rs.lamb)
    fa.saveTheta(rs.NNpath + name+".theta")
    #test(fa, stateAll)
    
def test(fa, state):
    for el in state:
        if rd.random()<0.06:
            retour = fa.computeOutput(el)
            print("in:", el)
            print(" out:", retour)

def UnitTestNN():
    fa = NeuralNet(2,3)
    input, output = [], []
    for i in range(10000):
        x,y = rd.random(), rd.random()
        input.append([x,y])
        output.append([x*y, x-y, x+y])
    fa.getTrainingData(np.vstack(np.array(input)), np.vstack(np.array(output)))
    fa.train()
    fa.saveTheta("test.theta")

    fa.loadTheta("test.theta")
    for i in range(20):
        x,y = 3*rd.random(), 3*rd.random()
        approx = fa.computeOutput(np.array([x,y]))
        print("in:", [x,y])
        print(" out:", approx)
        print(" real:",  [x*y, x-y, x+y])
  
def UnitTestNNController():
    '''
    Tests the approximated command obtained from training states
    '''
    rs = ReadSetupFile()
    fa = initNNController(rs)

    stateAll, commandAll = stateAndCommandDataFromTrajs(loadStateCommandPairsByStartCoords(pathDataFolder + "TrajRepository/"))
    #stateAll, commandAll = stateAndCommandDataFromTrajs(loadStateCommandPairsByStartCoords(BrentTrajectoriesFolder))
    #print ("len:", len(commandAll[0]))
    state = np.vstack(np.array(stateAll))
    command = np.vstack(np.array(commandAll))

    fa.getTrainingData(state, command)
    fa.train()
    fa.saveTheta("test")
    fa.loadTheta("test")

    for el in os.listdir(BrentTrajectoriesFolder):
            for i in range(len(state)):
                if rd.random()<0.06:
                    outNN = fa.computeOutput(np.array(state[i]))
                    print("Real  :", command[i]) 
                    print("Learn :",outNN)
