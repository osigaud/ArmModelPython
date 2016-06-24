#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: Cost

Description: Class to calcul reward 
'''
import numpy as np







class Cost():
    def __init__(self, rs):
        self.rs=rs


    def computeManipulabilityCost(self, arm):
        '''
        Computes the manipulability cost on one step of the trajectory
        
        Input:    -cost: cost at time t, float
                
        Output:        -cost: cost at time t+1, float
        '''
        dotq, q = arm.getDotQAndQFromStateVector(self.arm.getState())
        manip = arm.directionalManipulability(q,self.cartTarget)
        return 1-manip

    def computeStateTransitionCost(self, U):
        '''
        Computes the cost on one step of the trajectory
        
        Input:    -cost: cost at time t, float
                -U: muscular activation vector, numpy array (6,1)
                -t: time, float
                
        Output:        -cost: cost at time t+1, float
        '''
        #compute the square of the norm of the muscular activation vector
        norme = np.linalg.norm(U)
        mvtCost = norme*norme
        #compute the cost following the law of the model
        #return np.exp(-t/self.rs.gammaCF)*(-self.rs.upsCF*mvtCost)
        return -self.rs.upsCF*mvtCost
    
    def computeStateTransitionCostU12(self, U):
        '''
        Computes the cost on one step of the trajectory
        
        Input:    -cost: cost at time t, float
                -U: muscular activation vector, numpy array (6,1)
                -t: time, float
                
        Output:        -cost: cost at time t+1, float
        '''
        #compute the square of the norm of the muscular activation vector
        
        norme = np.linalg.norm(U[:2])
        mvtCost = norme*norme
        #compute the cost following the law of the model
        #return np.exp(-t/self.rs.gammaCF)*(-self.rs.upsCF*mvtCost)
        return -self.rs.upsCF*mvtCost
    
    def computePerpendCost(self, arm): 
        '''
        compute the Perpendicular cost for one trajectory
        
        Ouput :        -cost, the perpendicular cost
        ''' 
        dotq, q = arm.getDotQAndQFromStateVector(arm.getState())
        J = arm.jacobian(q)
        xi = np.dot(J,dotq)
        norm=np.linalg.norm(xi)
        if(norm!=0):
            xi = xi/norm
        return 500-1000*xi[0]*xi[0]
    
    def computeFinalReward(self, arm, t, coordHand, sizeOfTarget):
        return 0