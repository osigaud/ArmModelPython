#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: Test

Description: Functions to test experimentation
'''

import numpy as np
from Plot.plotFunctions import plotRegBrent
from Experiments.TrajMaker import TrajMaker
from Utils.FileReading import loadTrajs
from Utils.ReadXmlFile import ReadXmlFile
from ArmModel.Arm2 import Arm2
from GlobalVariables import pathDataFolder

def testRegression(setupFile, thetaFile):
    """ 
    
    """
    rs=ReadXmlFile(setupFile)
    stateAll, commandAll = loadTrajs(pathDataFolder + "Brent/", 0.01, rs.det)
    stateAll=np.array(stateAll)
    arm=Arm2()
    trajReg = np.empty_like(stateAll)
    trajMaker = TrajMaker(rs,0.02,None,rs.path+thetaFile)

    for i in range(stateAll.shape[0]):
        coordHand = arm.mgdEndEffector(stateAll[i][0][2:])
        trajReg[i]=trajMaker.runTrajectory2(coordHand[0], coordHand[1])[0]
        
    plotRegBrent(trajReg, stateAll)
    
