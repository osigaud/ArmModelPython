#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher
Module: FileReading
Description: Functions to read project data

Organisation of the file :

data [0 - 5] = target in joint space
data [6 - 11] = estimated current state in joint space
data [12 - 17] =  actual current state in joint space
data [18 - 25] = noisy muscular activations
data [26 - 33] = noiseless muscular activations
data [34 - 39] = estimated next state in joint space
data [40 - 45] = actual next state in joint space
data [46 - 47] = elbow position in cartesian space
data [48 - 49] = wrist position in cartesian space
data [50 - 51] = hand position in cartesian space
'''
import random as rd
import numpy as np
import os

from ArmModel.Arm import Arm

def loadStateCommandPairsByStartCoords(foldername):
    '''
    Get all the data from a set of trajectories, sorted by the starting xy coordinates
    
    Output : dictionary of data whose keys are y then x coordinates
    '''
    arm = Arm()
    dataOut = {}
#    j = 0
    for el in os.listdir(foldername):
#        j = j+1
#        if j>4500 or rd.random()<0.5:
            data = np.loadtxt(foldername + el)
            coordHand = (data[0][50], data[0][51])
            x,y = str(coordHand[0]), str(coordHand[1])
            if not y in dataOut.keys():
                dataOut[y] = {}
            if not x in dataOut[y].keys():
                dataOut[y][x] = []
            traj = []
            for i in range(data.shape[0]):
                currentState = (data[i][12], data[i][13], data[i][14], data[i][15], data[i][16], data[i][17])
                noisyActiv = (data[i][18], data[i][19], data[i][20], data[i][21], data[i][22], data[i][23], data[i][24], data[i][25])
                pair = (currentState, noisyActiv)
                traj.append(pair)
            dataOut[y][x].append(traj)
    return dataOut

def stateAndCommandDataFromTrajs(data):
        '''
        Reorganizes trajectory data into an array of trajectories made of (state, command) pairs
        
        Input:     -data: dictionary
        Output:    -dataA: numpy array
        '''
        state, command = [], []
        for key, v in data.items():
            #if float(key) < 0.58:
                for k2, xvals in data[key].items():
                    for i in range(len(xvals)):
                        stateVec, commandVec = [], []
                        for j in range(len(xvals[i])):
                            stateVec.append(xvals[i][j][0])
                            commandVec.append(xvals[i][j][1])
                        state.append(stateVec)
                        command.append(commandVec)
        '''
        state = np.vstack(np.array(state))
        command = np.vstack(np.array(command))
        '''
        return state,command

# -------------------------------------------------------------------------------------------
    

def getInitPos(foldername):
    '''
    Get all the initial positions from a set of trajectories, in xy coordinates
    
    Output : dictionary of initial position of all trajectories
    '''
    arm = Arm()
    xy = {}
    for el in os.listdir(foldername):
            data = np.loadtxt(foldername + el)
            coordHand = (data[0][50], data[0][51])
            #if coordHand[1]<0.58:
            xy[el] = (coordHand[0], coordHand[1])
    return xy
  
def getStateAndCommandData(foldername):
    '''
    Put all the states and commands of trajectories generated by the Brent controller into 2 different dictionaries
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
                -command: dictionary keys = filenames, values = array of data
    '''
    state, command = {}, {}
    for el in os.listdir(foldername):
        #if rd.random()<0.06:
            state[el], command[el] = [], []
            data = np.loadtxt(foldername + el)
            for i in range(data.shape[0]):
                currentState = (data[i][12], data[i][13], data[i][14], data[i][15], data[i][16], data[i][17])
                noisyActiv = (data[i][18], data[i][19], data[i][20], data[i][21], data[i][22], data[i][23], data[i][24], data[i][25])
                state[el].append(currentState)
                #command[el].append((data[i][18], data[i][19], data[i][20], data[i][21], data[i][22], data[i][23]))
                #we use the noisy command because it is filtered
                #com = np.array([data[i][12], data[i][13], data[i][14], data[i][15], data[i][16], data[i][17]])
                #com = muscleFilter(com)
                #It seems that filtering prevents learning...
                command[el].append(noisyActiv)
    return state, command
  
def getCommandData(foldername):
    '''
    Put all the commands of trajectories generated by the Brent controller into 2 different dictionaries
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    -command: dictionary keys = filenames, values = array of data
    '''
    command = {}
    for el in os.listdir(foldername):
            command[el] = []
            data = np.loadtxt(foldername + el)
            for i in range(data.shape[0]):
                #command[el].append((data[i][18], data[i][19], data[i][20], data[i][21], data[i][22], data[i][23]))
                #we use the noisy command because it is filtered
                noisyActiv = (data[i][18], data[i][19], data[i][20], data[i][21], data[i][22], data[i][23], data[i][24], data[i][25])
                command[el].append(noisyActiv)
    return command
  
def getNoiselessCommandData(foldername):
    '''
    Put all the states and commands of trajectories generated by the Brent controller into 2 different dictionaries
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    -command: dictionary keys = filenames, values = array of data
    '''
    command = {}
    for el in os.listdir(foldername):
            command[el] = []
            data = np.loadtxt(foldername + el)
            for i in range(data.shape[0]):
                noislessActiv = (data[i][26], data[i][27], data[i][28], data[i][29], data[i][30], data[i][31], data[i][32], data[i][33])
                command[el].append(noislessActiv)
                #we use the noisy command because it is filtered
                #command[el].append([data[i][12], data[i][13], data[i][14], data[i][15], data[i][16], data[i][17]])
    return command

def getStateData(foldername):
    '''
    Put all the states of trajectories generated by the Brent controller into a dictionary
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    '''
    state = {}
    for el in os.listdir(foldername):
        state[el] = []
        data = np.loadtxt(foldername + el)
        for i in range(data.shape[0]):
            currentState = (data[i][12], data[i][13], data[i][14], data[i][15], data[i][16], data[i][17])
            state[el].append(currentState)
    return state

def getXYHandData(foldername):
    '''
    Put all the states of trajectories generated by the Brent controller into a dictionary
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    '''
    arm = Arm()
    xy = {}
    for el in os.listdir(foldername):
        xy[el] = []
        data = np.loadtxt(foldername + el)
        for i in range(data.shape[0]):
           coordHand = (data[0][50], data[0][51])
           xy[el].append((coordHand[0], coordHand[1]))
    return xy

def getXYWristData(foldername):
    '''
    Put all the states of trajectories generated by the Brent controller into a dictionary
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    '''
    arm = Arm()
    xy = {}
    for el in os.listdir(foldername):
        xy[el] = []
        data = np.loadtxt(foldername + el)
        for i in range(data.shape[0]):
           coordWrist = (data[0,48], data[0,49])
           xy[el].append((coordWrist[0], coordWrist[1]))
    return xy

def getXYElbowData(foldername):
    '''
    Put all the states of trajectories generated by the Brent controller into a dictionary
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    '''
    arm = Arm()
    xy = {}
    for el in os.listdir(foldername):
        xy[el] = []
        data = np.loadtxt(foldername + el)
        for i in range(data.shape[0]):
           coordElbow = (data[0,46], data[0,47])
           xy[el].append((coordElbow[0], coordElbow[1]))
    return xy

def getEstimatedStateData(foldername):
    '''
    Put all the states of trajectories generated by the Brent controller into a dictionary
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    '''
    state = {}
    for el in os.listdir(foldername):
        state[el] = []
        data = np.loadtxt(foldername + el)
        for i in range(data.shape[0]):
            estimState = (data[i][6], data[i][7], data[i][8], data[i][9], data[i][10], data[i][11])
            state[el].append(estimState)
    return state
    
def getEstimatedXYHandData(foldername):
    '''
    Put all the states of trajectories generated by the Brent controller into a dictionary
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    '''
    arm = Arm()
    xyEstim = {}
    for el in os.listdir(foldername):
        xyEstim[el] = []
        data = np.loadtxt(foldername + el)
        for i in range(data.shape[0]):
            estimState = (data[i][6], data[i][7], data[i][8], data[i][9], data[i][10], data[i][11])
            coordHand = arm.mgdEndEffector(np.array(estimState[3], estimState[4], estimState[5]))
            xyEstim[el].append((coordHand[0], coordHand[1]))
    return xyEstim
    
def getXYEstimError(foldername):
    '''
    Returns the error estimations in the trajectories from the given foldername
    
    Outputs:    -errors: dictionary keys = filenames, values = array of data
    '''
    arm = Arm()
    errors = {}
    for el in os.listdir(foldername):
        errors[el] = []
        data = np.loadtxt(foldername + el)
        for i in range(data.shape[0]):
            currentState = (data[i][12], data[i][13], data[i][14], data[i][15], data[i][16], data[i][17])
            estimState = (data[i][6], data[i][7], data[i][8], data[i][9], data[i][10], data[i][11])
            statePos = (currentState[3], currentState[4], currentState[5])
            estimStatePos = (estimState[3], estimState[4], estimState[5])
            errors[el].append(arm.estimErrorReduced(statePos,estimStatePos))
    return errors
    
def getXYEstimErrorOfSpeed(foldername):
    '''
    Returns the error estimations in the trajectories as a function of speed from the given foldername
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    '''
    arm = Arm()
    errors = {}
    for el in os.listdir(foldername):
        errors[el] = []
        data = np.loadtxt(foldername + el)
        for i in range(data.shape[0]):
            currentState = (data[i][12], data[i][13], data[i][14], data[i][15], data[i][16], data[i][17])
            speed = arm.cartesianSpeed(currentState)
            estimState = (data[i][6], data[i][7], data[i][8], data[i][9], data[i][10], data[i][11])
            statePos = (currentState[3], currentState[4], currentState[5])
            estimStatePos = (estimState[3], estimState[4], estimState[5])
            error = arm.estimErrorReduced(statePos,estimStatePos)
            errors[el].append((speed, error))
    return errors
    
def getCostData(foldername):
    '''
    Put all the states of trajectories generated by the Brent controller into a dictionary
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    '''
    costDico = {}
    for el in os.listdir(foldername):
        costDico[el] = []
        data = np.loadtxt(foldername + el)
        for i in range(data.shape[0]):
           x = data[i][0]
           y = data[i][1]
           cost = data[i][2]
           costDico[el].append((x, y, cost))
    return costDico
    
def getTrajTimeData(foldername):
    '''
    Put all the states of trajectories generated by the Brent controller into a dictionary
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    '''
    trajTimeDico = {}
    for el in os.listdir(foldername):
        trajTimeDico[el] = []
        data = np.loadtxt(foldername + el)
        for i in range(data.shape[0]):
           x = data[i][0]
           y = data[i][1]
           trajTime = data[i][2]
           trajTimeDico[el].append((x, y, trajTime))
    return trajTimeDico
    
def getLastXData(foldername):
    '''
    Put all the states of trajectories generated by the Brent controller into a dictionary
    
    Outputs:    -state: dictionary keys = filenames, values = array of data
    '''
    xDico = {}
    for el in os.listdir(foldername):
        xDico[el] = []
        data = np.loadtxt(foldername + el)
        for i in range(data.shape[0]):
            xDico[el].append(data[i])
    return xDico

def dicToArray(data):
        '''
        This function transform a dictionary into an array
        
        Input:     -data: dictionary
        Output:    -dataA: numpy array
        '''
        retour = []
        for k, v in data.items():
            retour.append(v)
        return np.vstack(np.array(retour))
            
    


    



