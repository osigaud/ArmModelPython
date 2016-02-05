#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: RunRegression

Description: Functions to run a regression to create a controller
'''

import os
import numpy as np
import random as rd

from Utils.ReadSetupFile import ReadSetupFile
from Utils.FileReading import getStateAndCommandData, dicToArray,stateAndCommandDataFromTrajs,loadStateCommandPairsByStartCoords


from RBFN import rbfn
from NeuralNet import NeuralNet
from ArmModel.Arm import Arm

from GlobalVariables import BrentTrajectoriesFolder, pathDataFolder

def initController(rs,fileName):
    '''
	Initializes the controller allowing to compute the output from the input and the vector of parameters theta
	
	Input:		-rs: ReadSetupFile, parameters
                        -fileName: fichier theta pour Neura network
                                   fichier features pour rbfn
	'''
    #Initializes the function approximator
    if(rs.regression=="RBFN"):
        fa = rbfn(rs.inputDim, rs.outputDim, rs.numfeats)
    else :
        fa = NeuralNet(rs.inputDim, rs.outputDim)
    fa.load(fileName)
    return fa


def run(fileName,name, fromStruct=None):
    ''' 
    Takes the Brent trajectories as input, shuffles them, and then runs the NN regression algorithm
    '''
    rs = ReadSetupFile(fileName)
    #state, command = getStateAndCommandData(BrentTrajectoriesFolder)
    #stateAll, commandAll = dicToArray(state), dicToArray(command)
    #print ("old:", stateAll[0])

    stateAll, commandAll = stateAndCommandDataFromTrajs(loadStateCommandPairsByStartCoords(pathDataFolder + "TrajRepository/", 0.5))
    print("RunRegression L52")
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
    if(rs.regression=="RBFN"):
        fa = rbfn(rs.inputDim, rs.outputDim, rs.numfeats)
        fa.getTrainingData(stateAll, commandAll)
        if(fromStruct == True):
            fa.initFromExistingStruct(rs.path+name+".struct")
        else:
            fa.initFromData(rs.path+name+".struct")
        fa.train(rs.lamb)
    else :
        fa = NeuralNet(rs.inputDim, rs.outputDim)
        fa.getTrainingData(stateAll, commandAll)
        fa.train()
    fa.saveTheta(rs.path+ name+".theta")

def test(fa, state):
    for el in state:
        if rd.random()<0.06:
            retour = fa.computeOutput(el)
            print("in:", el)
            print(" out:", retour)


def UnitTestController(fileName):
    '''
    Tests the approximated command obtained from training states
    '''
    rs = ReadSetupFile(fileName)
    fa = initController(rs)

    stateAll, commandAll = stateAndCommandDataFromTrajs(loadStateCommandPairsByStartCoords(pathDataFolder + "TrajRepository/"))
    #stateAll, commandAll = stateAndCommandDataFromTrajs(loadStateCommandPairsByStartCoords(BrentTrajectoriesFolder))
    #print ("len:", len(commandAll[0]))
    state = np.vstack(np.array(stateAll))
    command = np.vstack(np.array(commandAll))

    fa.getTrainingData(state, command)
    if(rs.regression=="RBFN"):
        fa.train(rs.lamb)
    else:
        fa.train()
    fa.saveTheta("test")
    fa.loadTheta("test")

    for el in os.listdir(BrentTrajectoriesFolder):
            for i in range(len(state)):
                if rd.random()<0.06:
                    outNN = fa.computeOutput(np.array(state[i]))
                    print("Real  :", command[i]) 
                    print("Learn :",outNN)

# a replacer plus intelligemment

def UnitTestArmModel(fileName):
    '''
    Tests the next state 
    '''
    rs = ReadSetupFile(fileName)

    arm = Arm()
    arm.setDT(rs.dt)

    state, estimState, command, noisycommand, nextEstimState, nextState = {}, {}, {}, {}, {}, {}
    for el in os.listdir(BrentTrajectoriesFolder):
            state[el], estimState[el], command[el], noisycommand[el], nextEstimState[el], nextState[el] = [], [], [], [], [], []
            data = np.loadtxt(BrentTrajectoriesFolder + el)
            for i in range(data.shape[0]):
                estimState[el].append(np.array([data[i][4], data[i][5], data[i][6], data[i][7]]))
                state[el].append(np.array([data[i][8], data[i][9], data[i][10], data[i][11]]))
                noisycommand[el].append(np.array([data[i][12], data[i][13], data[i][14], data[i][15], data[i][16], data[i][17]]))
                command[el].append(np.array([data[i][18], data[i][19], data[i][20], data[i][21], data[i][22], data[i][23]]))
                nextEstimState[el].append(np.array([data[i][24], data[i][25], data[i][26], data[i][27]]))
                nextState[el].append(np.array([data[i][28], data[i][29], data[i][30], data[i][31]]))

    for el in os.listdir(BrentTrajectoriesFolder):
            for i in range(len(state[el])):
                if rd.random()<0.06:
                    outNextStateNoisy = arm.computeNextState(noisycommand[el][i],state[el][i])
                    outNextState = arm.computeNextState(command[el][i],state[el][i])
                    
                    print("U      :", command[el][i]) 
                    print("UNoisy :", noisycommand[el][i])
                    print("---------------------------------------------------------")
                    print("Real :", nextState[el][i]) 
                    print("ArmN :", outNextStateNoisy)
                    print("Arm :", outNextState)
                    print("---------------------------------------------------------")
