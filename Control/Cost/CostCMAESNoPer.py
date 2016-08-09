#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: Cost

Description: Class to calcul reward 
'''
import numpy as np
from Cost import Cost






class CostCMAESNoPer(Cost):
    def __init__(self, rs):
        self.rs=rs


   

    def computeFinalReward(self, arm, t, coordHand, sizeOfTarget):
        '''
        Computes the cost on final step if the target is reached
        
        Input:        -t: time, float
                    -coordHand: coordinate of the end effector, numpy array
                    
        Output:        -cost: final cost , float
        '''
        cost=0
        
        #print coordHand[0]
        #check if the Ordinate of the target is reached and give the reward if yes
        if coordHand[1] >= self.rs.YTarget:
            #print "main X:", coordHand[0]
            #check if target is reached
            if coordHand[0] >= -sizeOfTarget/2 and coordHand[0] <= sizeOfTarget/2:
                cost += np.exp(-t/self.rs.gammaCF)*self.rs.rhoCF
            else:
                #cost += -500-500000*(coordHand[0]*coordHand[0])
                cost+= -50000+50000*(1-coordHand[0]**2)
        else:
            cost += -10000+10000*(coordHand[1]*coordHand[1])
        return cost