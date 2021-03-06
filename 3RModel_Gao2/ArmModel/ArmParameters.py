#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: ArmParameters

Description:    -We find here all arm parameters
                -we use a model of arm with two joints and six muscles
'''
import numpy as np
from GlobalVariables import pathWorkingDirectory

class ArmParameters:
    '''
    class ArmParameters
    '''
    def __init__(self):
        '''
        Initializes the class
        '''
        self.pathSetupFile = pathWorkingDirectory + "/ArmModel/Setup/Arm.params"
        self.readSetupFile()
        self.AMatrix()
        self.BMatrix()
        self.readStops()
    
    def readSetupFile(self):
        '''
        Reads the setup file
        '''
        with open(self.pathSetupFile, "r") as file:
            alls = file.read()
        #Split to read line by line
        allsByLign = alls.split("\n")
        #line 1, Arm length
        self.l1 = float((allsByLign[0].split(":"))[1])
        #line 2, ForeArm length
        self.l2 = float((allsByLign[1].split(":"))[1])
        #line 3, Hand length
        self.l3 = float((allsByLign[2].split(":"))[1])
        #line 4, Arm mass
        self.m1 = float((allsByLign[3].split(":"))[1])
        #line 5, ForeArm mass
        self.m2 = float((allsByLign[4].split(":"))[1])
        #line 6, Hand mass
        self.m3 = float((allsByLign[5].split(":"))[1])
        #line 7, Distance from the axis of joint 1 to its center of mass
        self.s1 = float((allsByLign[6].split(":"))[1])
        #line 8, Distance from the axis of joint 2 to its center of mass
        self.s2 = float((allsByLign[7].split(":"))[1])
        #line 9, Distance from the axis of joint 3 to its center of mass
        self.s3 = float((allsByLign[8].split(":"))[1])
        #line 10, Arm inertia
        self.i1 = float((allsByLign[9].split(":"))[1])
        #line 11, ForeArm inertia
        self.i2 = float((allsByLign[10].split(":"))[1])
        #line 12, Hand inertia
        self.i3 = float((allsByLign[11].split(":"))[1]) 
        
    def BMatrix(self):
        '''
        Defines the damping matrix B
        '''
        with open(self.pathSetupFile, "r") as file:
            alls = file.read()
        allsByLign = alls.split("\n")
        #line 13, Damping term b1
        b1 = float((allsByLign[12].split(":"))[1])
        #line 14, Damping term b2
        b2 = float((allsByLign[13].split(":"))[1])
        #line 15, Damping term b3
        b3 = float((allsByLign[14].split(":"))[1])
        #line 16, Damping term b4
        b4 = float((allsByLign[15].split(":"))[1])
        #line 17, Damping term b5
        b5 = float((allsByLign[16].split(":"))[1])
        #line 18, Damping term b6
        b6 = float((allsByLign[17].split(":"))[1])
        #line 19, Damping term b7
        b7 = float((allsByLign[18].split(":"))[1])
        #line 20, Damping term b8
        b8 = float((allsByLign[19].split(":"))[1])
        #line 21, Damping term b9
        b9 = float((allsByLign[20].split(":"))[1])
        #matrix definition
        self.B = np.array([[b1,b2,b3],[b4,b5,b6],[b7,b8,b9]])
        
    def AMatrix(self):
        '''
        Defines the moment arm matrix A
        '''
        with open(self.pathSetupFile, "r") as file:
                alls = file.read()
                allsByLign = alls.split("\n")
        #line 22, Moment arm matrix, a1
        a1 = float((allsByLign[21].split(":"))[1])
        #line 23, Moment arm matrix, a2
        a2 = float((allsByLign[22].split(":"))[1])
        #line 24, Moment arm matrix, a3
        a3 = float((allsByLign[23].split(":"))[1])
        #line 25, Moment arm matrix, a4
        a4 = float((allsByLign[24].split(":"))[1])
        #line 26, Moment arm matrix, a5
        a5 = float((allsByLign[25].split(":"))[1])
        #line 27, Moment arm matrix, a6
        a6 = float((allsByLign[26].split(":"))[1])
        #line 28, Moment arm matrix, a7
        a7 = float((allsByLign[27].split(":"))[1])
        #line 29, Moment arm matrix, a8
        a8 = float((allsByLign[28].split(":"))[1])
        #line 30, Moment arm matrix, a9
        a9 = float((allsByLign[29].split(":"))[1])
        #line 31, Moment arm matrix, a10
        a10 = float((allsByLign[30].split(":"))[1])
        #line 32, Moment arm matrix, a11
        a11 = float((allsByLign[31].split(":"))[1])
        #line 33, Moment arm matrix, a12
        a12 = float((allsByLign[32].split(":"))[1])
        #line 34, Moment arm matrix, a13
        a13 = float((allsByLign[33].split(":"))[1])
        #line 35, Moment arm matrix, a14
        a14 = float((allsByLign[34].split(":"))[1])
        #line 36, Moment arm matrix, a15
        a15 = float((allsByLign[35].split(":"))[1])
        #line 37, Moment arm matrix, a16
        a16 = float((allsByLign[36].split(":"))[1])
        #line 38, Moment arm matrix, a17
        a17 = float((allsByLign[37].split(":"))[1])
        #line 39, Moment arm matrix, a18
        a18 = float((allsByLign[38].split(":"))[1])
        #line 40, Moment arm matrix, a19
        a19 = float((allsByLign[39].split(":"))[1])
        #line 41, Moment arm matrix, a20
        a20 = float((allsByLign[40].split(":"))[1])
        #line 42, Moment arm matrix, a21
        a21 = float((allsByLign[41].split(":"))[1])
        #line 43, Moment arm matrix, a22
        a22 = float((allsByLign[42].split(":"))[1])
        #line 44, Moment arm matrix, a23
        a23 = float((allsByLign[43].split(":"))[1])
        #line 45, Moment arm matrix, a24
        a24 = float((allsByLign[44].split(":"))[1])
        #matrix definition
        self.At = np.array([[a1,a2,a3,a4,a5,a6,a7,a8], [a9,a10,a11,a12,a13,a14,a15,a16], [a17,a18,a19,a20,a21,a22,a23,a24]])

    def readStops(self):
        with open(self.pathSetupFile, "r") as file:
            alls = file.read()
        allsByLign = alls.split("\n")
        #line 46, Shoulder upper bound
        self.sub = float((allsByLign[45].split(":"))[1])
        #line 47, Shoulder lower bound
        self.slb = float((allsByLign[46].split(":"))[1])
        #line 48, Elbow upper bound
        self.eub = float((allsByLign[47].split(":"))[1])
        #line 49, Elbow lower bound
        self.elb = float((allsByLign[48].split(":"))[1])
        #line 50, Hand upper bound
        self.hub = float((allsByLign[49].split(":"))[1])
        #line 51, Hand lower bound
        self.hlb = float((allsByLign[50].split(":"))[1])
