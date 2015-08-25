#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: CostComputation

Description: The class which computes the trajectory cost
'''
import numpy as np

class CostComputation:
    
    def __init__(self, rs):
        '''Initializes class object needed to acces to the setup variables
        
        Input:	-rs: ReadSetup, class object given acces to the setup variables
        '''
        self.name = "CostComputation"
        self.rs = rs
         
    def computeStateTransitionCost(self, cost, U, t):
        '''
		Computes the cost on one step of the trajectory
		
		Input:	-cost: cost at time t, float
				-U: muscular activation vector, numpy array (6,1)
				-t: time, float
				
		Output:		-cost: cost at time t+1, float
		'''
        #compute the square of the norm of the muscular activation vector
        mvtCost = (np.linalg.norm(U))**2
        #compute the cost following the law of the model
        cost += np.exp(-t/self.rs.gammaCF)*(-self.rs.upsCF*mvtCost)
        return cost

    
    def computeFinalCostReward(self, cost, t):
        '''
		Computes the cost on final step if the target is reached
		
		Input:		-cost: cost at the end of the trajectory, float
					-t: time, float
					
		Output:		-cost: final cost if the target is reached
		'''
        cost += np.exp(-t/self.rs.gammaCF)*self.rs.rhoCF
        return cost
    
    
    
    
    
