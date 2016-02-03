#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher
Cythonisation: Remi Cambuzat

Module: MusclesParameters

Description:    -We find here all muscles parameters
                -we use a model of arm with two joints and six muscles
'''
# cimport MusclesParameters
#from MusclesParameters cimport MusclesParameters

import numpy as np
cimport numpy as np # import special compile-time information about the numpy module for cython

import math

from GlobalVariables import pathWorkingDirectory

cdef class MusclesParameters:
    '''
    class MusclesParameters
    '''

    # ATTRIBUTE DECLARATIONS in MusclesParameters.pxd

    def __init__(self):
        '''
        class parameters initialization
        '''
        print("init MusclesParameters")

        pathSetupFile = pathWorkingDirectory + "/ArmModel/Setup/Muscles.params"
        #pathSetupFile = "ArmModel/Setup/Muscles.params"
        self.pathSetupFile = pathSetupFile;
        #print(self.pathSetupFile)

        self.activationVectorInit()
        self.fmaxMatrix()

        ###############################Annexe parameters########################
        #Hogan parameters
        self.GammaMax = 2
        self.K = (2*self.GammaMax)/math.pi#stiffness
        self.Gamma_H = np.array([[0],[0]])#hogan torque initialization
        #stiffness matrix (null)
        self.Knulle = np.mat([(0, 0, 0, 0, 0, 0),(0, 0, 0, 0, 0, 0)])
        #stiffness matrix (low)
        self.Kp1 = 10.
        self.Kp2 = 10.
        self.KP1 = 10.
        self.KP2 = 10.
        self.Kraid = np.mat([(self.KP1,self.KP1,0,0,self.Kp1,self.Kp1),(0,0,self.Kp2,self.Kp2,self.KP2,self.KP2)])
        #stiffness matrix (high)
        self.KP22 = (80*self.GammaMax)/math.pi
        self.Kp22=(60*self.GammaMax)/math.pi
        self.KP11=(200*self.GammaMax)/math.pi
        self.Kp11=(100*self.GammaMax)/math.pi
        self.Kgrand = np.mat([(self.KP11,self.KP11,0,0,self.Kp11,self.Kp11),(0,0,self.Kp22,self.Kp22,self.KP22,self.KP22)])
        #Proportional gain
        self.Kp = 10 # Arbitrary value
        #Derivative gain
        self.Kd = 2*math.sqrt(self.Kp)

    cdef fmaxMatrix(self):
        '''
        Defines the matrix of the maximum force exerted by each muscle
        '''
        with open(self.pathSetupFile, "r") as file:
            alls = file.read()
        allsByLign = alls.split("\n")
        #line 1, fmax1
        cdef float fmax1 = float((allsByLign[0].split(":"))[1])
        #line 2, fmax2
        cdef float fmax2 = float((allsByLign[1].split(":"))[1])
        #line 3, fmax3
        cdef float fmax3 = float((allsByLign[2].split(":"))[1])
        #line 4, fmax4
        cdef float fmax4 = float((allsByLign[3].split(":"))[1])
        #line 5, fmax5
        cdef float fmax5 = float((allsByLign[4].split(":"))[1])
        #line 6, fmax6
        cdef float fmax6 = float((allsByLign[5].split(":"))[1])
        #matrix definition
        self.fmax = np.diag([fmax1, fmax2, fmax3, fmax4, fmax5, fmax6])

        #line 7, amount of motor noise on U
        self.knoiseU = float((allsByLign[6].split(":"))[1])


    cdef activationVectorInit(self):
        '''
        Initializes the muscular activation vector
        '''
        cdef float u1 = 0
        cdef float u2 = 0
        cdef float u3 = 0
        cdef float u4 = 0
        cdef float u5 = 0
        cdef float u6 = 0
        self.U0 = np.array([[u1],[u2],[u3],[u4],[u5],[u6]])

    cpdef activationVectorUse(self, u1, u2, u3, u4, u5, u6):
        '''
        Builds the muscular activation vector from its 6 components
        '''
        cdef np.ndarray U = np.array([[u1],[u2],[u3],[u4],[u5],[u6]])
        return U
