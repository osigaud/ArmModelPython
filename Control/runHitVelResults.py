#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher + Olivier Sigaud

Module: runScript

Description: script to run cmaes
'''




from Main.MainCMAES import lauchCMAESForListOfPoints, checkAllPoint, launchCMAESForSpecificTargetSizeAndSpecificPointMulti, generateFromCMAESonePoint, generateFromCMAESNController, launchCMAESForAllPoint, launchCMAESForAllTargetSizesMulti, generateFromCMAES, generateRichDataFromRegression, generateRichDataFromCMAES, launchCMAESForAllTargetSizes, launchCMAESForSpecificTargetSize

from Plot.plotFunctions import plotCostColorMapFor12, plotCMAESOnePointProgress, plotEstimatorPoint, trajectoriesAnimation, plotCostColorMap, plotTimeColorMap, plotTimeDistanceTarget, plotFittsLaw, plotPerfSizeDist, plotVelocityProfile, plotXYPositions, plotXYEstimError, plotXYEstimErrorOfSpeed, plotArticularPositions, plotInitPos, plotMuscularActivations, plotScattergram, plotHitDispersion, plotExperimentSetup, plotCMAESProgress, plotTrajsInRepo, plotManipulability, plotManipulability2
from GlobalVariables import pathDataFolder
import numpy as np
from Utils.Chrono import Chrono
from Utils.ReadXmlFile import ReadXmlFile


#----------------------------- main list of available actions ----------------------------------------------------------------------

def printMainMenu():
    print('    Will do the following:')
    print('        27 generate results from CMAES One point')
    print('        3 plot velocity profiles')
    print('        4 plot XY and articular positions')
    print('        6 plot cost Map')                  
    print('        9 plot Fitts Law')                  
    print('        10 plot Map Time x Trajectory')
    print('        12 plot Hit dispersion (CMAES or RBFN)')

    print('        14 plot CMAES cost progress')
    print('        26 plot CMAES One point cost progress')
    print('        28 plot XY  for one target')  
    print('        30 plot cost Map of muscles 1 and 2')
    print('        31 check CMAES progression')
    
def runChoice():
    checkL = True
    fileName = 'setupHitVelgamma6.xml'
    rs = ReadXmlFile(fileName)

    nameTheta = 'Best'
    nameF = 'results_folder'
    nbret = 5
    generateFromCMAESNController(nbret, rs, nameTheta, nameF)
    plotVelocityProfile("OPTI",rs,nameF)
    plotXYPositions("OPTI",rs, nameF,"All",False)
    plotCostColorMap("OPTI",rs, nameF)
    plotTimeDistanceTarget(nameF, rs)
    plotFittsLaw(nameF, rs)
    plotTimeColorMap("OPTI",rs, nameF)
    plotScattergram("OPTI",nameF, rs)

runChoice()
