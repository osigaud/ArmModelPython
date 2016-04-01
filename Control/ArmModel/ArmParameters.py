#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: ArmParameters

Description:    -We find here all arm parameters
                -we use a model of arm with two joints and six muscles
'''
import numpy as np
import os

class ArmParameters:
    '''
    class ArmParameters
    '''
    
    def __init__(self):
        '''
        Intializes the class
        '''
        self.pathSetupFile = os.getcwd() + "/ArmModel/Setup/Arm.params"
        with open(self.pathSetupFile, "r") as setupFile:
            alls = setupFile.read()
        #Split to read line by line
        allsByLign = alls.split("\n")
        self.readSetupFile(allsByLign)
        self.massMatrix()
        self.AMatrix(allsByLign)
        self.BMatrix(allsByLign)
        self.readStops(allsByLign)
        
    def readSetupFile(self,allsByLign):
        '''
        Reads the setup file
        '''
        
        #line 1, Arm length
        self.l1 = float((allsByLign[0].split(":"))[1])
        #line 2, ForeArm length
        self.l2 = float((allsByLign[1].split(":"))[1])
        #line 3, Arm mass
        self.m1 = float((allsByLign[2].split(":"))[1])
        #line 4, ForeArm mass
        self.m2 = float((allsByLign[3].split(":"))[1])
        #line 5, Arm inertia
        self.I1 = float((allsByLign[4].split(":"))[1])
        #line 6, ForeArm inertia
        self.I2 = float((allsByLign[5].split(":"))[1])
        #line 7, Distance from the center of segment 1 to its center of mass
        self.s1 = float((allsByLign[6].split(":"))[1])
        #line 8, Distance from the center of segment 2 to its center of mass
        self.s2 = float((allsByLign[7].split(":"))[1])
        
    def massMatrix(self):
        '''
        Initialization of parameters used for the inertia matrix
        '''
        self.k1 = self.I1 + self.I2 + self.m2*(self.l1**2)
        self.k2 = self.m2*self.l1*self.s2
        self.k3 = self.I2
    
    def BMatrix(self,allsByLign):
        '''
        Defines the damping matrix B
        '''
        #line 9, Damping term k6
        b1 = float((allsByLign[8].split(":"))[1])
        #line 10, Damping term k7
        b2 = float((allsByLign[9].split(":"))[1])
        #line 11, Damping term k8
        b3 = float((allsByLign[10].split(":"))[1])
        #line 12, Damping term k9
        b4 = float((allsByLign[11].split(":"))[1])
        #matrix definition
        self.B = np.array([[b1,b2],[b3,b4]])
    
    def AMatrix(self,allsByLign):
        '''
        Defines the moment arm matrix A
        '''
        #line 13, Moment arm matrix, a1
        a1 = float((allsByLign[12].split(":"))[1])
        #line 14, Moment arm matrix, a2
        a2 = float((allsByLign[13].split(":"))[1])
        #line 15, Moment arm matrix, a3
        a3 = float((allsByLign[14].split(":"))[1])
        #line 16, Moment arm matrix, a4
        a4 = float((allsByLign[15].split(":"))[1])
        #line 17, Moment arm matrix, a5
        a5 = float((allsByLign[16].split(":"))[1])
        #line 18, Moment arm matrix, a6
        a6 = float((allsByLign[17].split(":"))[1])
        #line 19, Moment arm matrix, a7
        a7 = float((allsByLign[18].split(":"))[1])
        #line 20, Moment arm matrix, a8
        a8 = float((allsByLign[19].split(":"))[1])
        #line 21, Moment arm matrix, a9
        a9 = float((allsByLign[20].split(":"))[1])
        #line 22, Moment arm matrix, a10
        a10 = float((allsByLign[21].split(":"))[1])
        #line 23, Moment arm matrix, a11
        a11 = float((allsByLign[22].split(":"))[1])
        #line 24, Moment arm matrix, a12
        a12 = float((allsByLign[23].split(":"))[1])
        #matrix definition
        self.At = np.array([[a1,a2,a3,a4,a5,a6], [a7,a8,a9,a10,a11,a12]])

    def readStops(self,allsByLign):
        #line 25, Shoulder upper bound
        self.sub = float((allsByLign[24].split(":"))[1])
        #line 26, Shoulder lower bound
        self.slb = float((allsByLign[25].split(":"))[1])
        #line 27, Elbow upper bound
        self.eub = float((allsByLign[26].split(":"))[1])
        #line 28, Elbow lower bound
        self.elb = float((allsByLign[27].split(":"))[1])

    
    
    
