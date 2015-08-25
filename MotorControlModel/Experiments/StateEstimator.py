#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Olivier Sigaud

Module: StateEstimator

Description: Used to estimate the current state, reproducing the human control motor delay.
'''
import numpy as np
import random as rd

#from ArmModel.MuscularActivation import getNoisyCommand

def isNull(vec):
    for el in vec:
        if el!=0:
            return False
    return True

class StateEstimator:
    
    def __init__(self, dimCommand, delay, arm):
        '''
    	Initializes parameters to uses the function implemented below
    	
    	inputs:		-dimCommand: dimension of the muscular activation vector U, int (here, 6)
    			-delay: delay in time steps with which we give the observation to the filter, int
    			-arm, armModel, class object
    	'''
        self.name = "StateEstimator"
        self.dimCommand = dimCommand
        self.delay = delay
        self.arm = arm

    def initStore(self, state):
        '''
    	Initialization of the observation storage
    
    	Input:		-state: the state stored
    	'''
        self.stateStore = []
        self.commandStore = []
        for i in range(self.delay):
            self.stateStore.append(state)
            self.commandStore.append([0] * self.dimCommand)
        #print ("InitStore:", self.stateStore)
    
    def storeInfo(self, state, command):
        '''
    	Stores the current state and returns the delayed state
    
    	Input:		-state: the state to store
    	'''
        for i in range (self.delay-1):
           self.stateStore[self.delay-i-1]=self.stateStore[self.delay-i-2]
           self.commandStore[self.delay-i-1]=self.commandStore[self.delay-i-2]
        self.stateStore[0]=state
        self.commandStore[0]=command
        #print ("After store:", self.stateStore)
        #print ("retour:", self.stateStore[self.delay-1])
        return self.stateStore[self.delay-1]
    
    def getEstimState(self, state, command):
        '''
    	Function used to compute the next state approximation with the filter
    
    	Inputs:		-state: the state to feed the filter, numpy array of dimension (x, 1), here x = 4
                        -command: the noiseless muscular activation vector U, numpy array of dimension (x, 1), here x = 6
    
    	Output:		-stateApprox: the next state approximation, numpy array of dimension (x, 1), here x = 4
    	'''
        #store the state of the arm to feed the filter with a delay on the observation
        estimState = self.storeInfo(state, command)
        for i in range (self.delay-1):
            U = self.commandStore[self.delay-i-1]
            if isNull(U):
                return state
            estimState = self.arm.computeNextState(U,estimState)
            for i in range(2,4):#len(estimState)):
                estimState[i] = estimState[i]*(1+ np.random.normal(0,0.001))

        return estimState
    
    def debugStore(self):
        state = np.array([1,2,3,4])
        self.initObsStore(state)
        for i in range(5):
            tmpS = [rd.random() for x in range(4)]
            tmpU = [rd.random() for x in range(6)]
            self.storeInfo(tmpS,tmpU)
    
