#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: Cost

Description: Class to calcul reward 
'''
import numpy as np
from Cost import Cost






class CostDDPG(Cost):
    reduce=3000.
    

    def computeStateTransitionCost(self, U, coordHand):
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
        dst=np.sqrt((coordHand[0]-self.rs.XTarget)**2+(coordHand[1]-self.rs.YTarget)**2)
        #compute the cost following the law of the model
        #return np.exp(-t/self.rs.gammaCF)*(-self.rs.upsCF*mvtCost)
        return -self.rs.upsCF*mvtCost/self.reduce-dst/1000.
    


    def computeFinalReward(self, arm, t, coordHand, sizeOfTarget):
        cost=0
        '''
        Computes the cost on final step if the target is reached
        
        Input:        -t: time, float
                    -coordHand: coordinate of the end effector, numpy array
                    
        Output:        -cost: final cost , float
        '''
        #print coordHand[0]
        #check if the Ordinate of the target is reached and give the reward if yes
        if coordHand[1] >= self.rs.YTarget:
            cost = self.computePerpendCost(arm)
            #print "main X:", coordHand[0]
            #check if target is reached
            if coordHand[0] >= -sizeOfTarget/2 and coordHand[0] <= sizeOfTarget/2:
                cost += np.exp(-t/self.rs.gammaCF)*self.rs.rhoCF/self.reduce
            else:
                cost += (-500-30000*(coordHand[0]*coordHand[0]))/self.reduce
        return cost