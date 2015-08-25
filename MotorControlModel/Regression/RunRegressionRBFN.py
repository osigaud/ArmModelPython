#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: RunRegressionRBFN

Description: We find here the function to run rbfn algorithm to create the controller
'''

import os
import numpy as np
import random as rd

from Utils.ReadSetupFile import ReadSetupFile
from Utils.FileReading import getStateAndCommandData, dicToArray,stateAndCommandDataFromTrajs,loadStateCommandPairsByStartCoords

from Regression.RBFN import rbfn
from ArmModel.Arm import Arm
from Experiments.TrajMaker import initRBFNController

from GlobalVariables import BrentTrajectoriesFolder

def runRBFN(name):
    ''' 
    Takes the Brent trajectories as input, shuffles them, and then runs the RBFN regression algorithm
    '''
    rs = ReadSetupFile()
    #state, command = getStateAndCommandData(BrentTrajectoriesFolder)
    #stateAll, commandAll = dicToArray(state), dicToArray(command)
    #print ("old:", stateAll[0])

    stateAll, commandAll = stateAndCommandDataFromTrajs(loadStateCommandPairsByStartCoords(BrentTrajectoriesFolder))
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
    fa = rbfn(rs.numfeats,rs.inputDim,rs.outputDim)
    fa.setTrainingData(stateAll, commandAll)
    fa.train_rbfn()
    saveThetaControllers(rs,name,fa)
    #test(fa, stateAll)

def saveThetaControllers(rs, name, fa):
    savename = rs.RBFNpath + name
    fa.saveTheta(savename)
    
def test(fa, state):
    for el in state:
        if rd.random()<0.06:
            retour = fa.computeOutput(el)
            print("in:", el)
            print(" out:", retour)

def UnitTest():
    fa = rbfn(3,2,3)
    input, output = [], []
    for i in range(10000):
        x,y = rd.random(), rd.random()
        input.append([x,y])
        output.append([x*y, x-y, x+y])
    fa.setTrainingData(np.vstack(np.array(input)), np.vstack(np.array(output)))
    fa.train_rbfn()
    fa.saveTheta("test")

    fa.loadTheta("test")
    for i in range(20):
        x,y = 3*rd.random(), 3*rd.random()
        approx = fa.computeOutput(np.array([x,y]))
        print("in:", [x,y])
        print(" out:", approx)
        print(" real:",  [x*y, x-y, x+y])
  
def UnitTestRBFNController():
    '''
    Tests the approximated command obtained from training states
    '''
    rs = ReadSetupFile()
    fa = initRBFNController(rs)
    fa.train_rbfn()
    fa.saveTheta("test")

    fa.loadTheta("test")

    state, command = {}, {}
    for el in os.listdir(BrentTrajectoriesFolder):
            state[el], command[el] = [], []
            data = np.loadtxt(BrentTrajectoriesFolder + el)
            for i in range(data.shape[0]):
                state[el].append((data[i][8], data[i][9], data[i][10], data[i][11]))
                command[el].append((data[i][18], data[i][19], data[i][20], data[i][21], data[i][22], data[i][23]))

    for el in os.listdir(BrentTrajectoriesFolder):
            for i in range(len(state[el])):
                if rd.random()<0.06:
                    outrbfn = fa.computeOutput(np.array(state[el][i]))
                    print("Real  :", command[el][i]) 
                    print("Learn :",outrbfn)
  
def UnitTestArmModel():
    '''
    Tests the next state 
    '''
    rs = ReadSetupFile()

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

    
    
