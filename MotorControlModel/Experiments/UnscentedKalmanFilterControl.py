#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: UnscentedKalmanFilterControl

Description: Class with some functions to use the unscented kalman filter to generate trajectories and reproduced the human control motor delay.
Uses the librairy pykalman.
'''
import numpy as np
import random as rd
from pykalman import UnscentedKalmanFilter

from ArmModel.MuscularActivation import getNoisyCommand

class UnscentedKalmanFilterControl:
    
    def __init__(self, dimState, delay, arm, knoiseU, controller):
        '''
    	Initializes parameters to uses the function implemented below
    	
    	inputs:		-dimState: dimension of the state, here the state correspond to the muscular activation vector U, int
    			-dimObs: dimension of the observation, here the observation is the position of the arm given by the model, int
    			-delay: the delay with which we give the observation to the filter, int
    			-arm, armModel, class object
                        -rs, ReadSetupFile, class object
    	'''
        self.name = "UnscentedKalmanFilter"
        self.knoiseU = knoiseU
        self.dimState = dimState
        self.delay = delay
        self.arm = arm
        self.controller = controller

        #initialization of some parameters for the filter
        transition_covariance = np.eye(self.dimState)*0.01
        initial_state_mean = np.zeros(self.dimState)
        observation_covariance = 1000*np.eye(self.dimState) 
        initial_state_covariance = np.eye(self.dimState)
        self.nextCovariance = np.eye(self.dimState)*0.0001
        self.ukf = UnscentedKalmanFilter(self.transitionFunctionUKF, self.observationFunctionUKF,
                                    transition_covariance, observation_covariance,
                                    initial_state_mean, initial_state_covariance)
    
    def initObsStore(self, state):
        '''
    	Initialization of the observation storage
    
    	Input:		-state: the state stored
    	'''
        self.obsStore = []
        for i in range(self.delay):
            self.obsStore.append(state)
        #print ("InitStore:", self.obsStore)
    
    def storeObs(self, state):
        '''
    	Stores the current state and returns the delayed state
    
    	Input:		-state: the state to store
    	'''
        for i in range (self.delay-1):
           self.obsStore[self.delay-i-1]=self.obsStore[self.delay-i-2]
        self.obsStore[0]=state
        #print ("After store:", self.obsStore)
        #print ("retour:", self.obsStore[self.delay-1])
        return self.obsStore[self.delay-1]
    
    def transitionFunctionUKF(self, state, transitionNoise = 0):
        '''
    	Transition function used by the filter, function of the state and the transition noise at time t and produces the state at time t+1
    	
    	Inputs:		-stateU: the state at time t, numpy array
    			-transitionNoise: transition noise at time t, numpy array
    
    	Output:		-nextStateUNoise: the next State with noise added, numpy array
    	'''
        #print("UKF trans state :", state)
        U = self.controller.computeOutput(state)
        nextState = self.arm.computeNextState(U, state)
        return nextState + transitionNoise
    
    def observationFunctionUKF(self, state, observationNoise = 0):
        '''
    	Observation function used by the filter, function of the state and the observation noise at time t and produces the observation at time t
    
    	Inputs:		-stateU: the state at time t, numpy array
    			-observationNoise: the observation noise at time t, numpy array
    
    	Output:		-nextObsNoise: observation at time t+1
    	'''
        U = self.controller.computeOutput(state)
        nextObs = self.arm.computeNextState(U, state)
        nextObsNoise = nextObs + observationNoise
        return nextObsNoise
    
    def runUKF(self, state):
        '''
    	Function used to compute the next state approximation with the filter
    
    	Inputs:		-Unoisy: the state to feed the filter, here its the muscular activation vector U, numpy array of dimension (x, 1), here x = 6
    			    -state: the state of the arm
    
    	Output:		-stateApprox: the next state approximation, numpy array of dimension (x, 1), here x = 4
    	'''
        #store the state of the arm to feed the filter with a delay on the observation
        oldstate = self.storeObs(state)
        #print("UKF old state :", oldstate)
        #compute the nextState approximation ie here the next muscular activation
        nextState, nextCovariance = self.ukf.filter_update(state, self.nextCovariance, oldstate)
        #compute the nextState i.e. the next position vector from the approximation of the next muscular activation vector given by the filter
        return nextState
    
    def debugStore(self):
        state = np.array([1,2,3,4])
        self.initObsStore(state)
        for i in range(5):
            tmp = [rd.random() for x in range(4)]
            self.storeObs(tmp)
    
