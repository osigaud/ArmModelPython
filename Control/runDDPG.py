#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: runScript

Description: script to run ddpg
'''




from Main.MainCMAES import  generateRichDataFromRegression
from Main.MainDDPG import launchDDPGForSpecificTargetSizeAndSpeceficBeginning, generateFromDDPGNController, launchDDPGForAllPoint, generateFromDDPG, generateRichDataFromDDPG, launchDDPGForSpecificTargetSize

from Plot.plotFunctions import plotDDPGOnePointProgress, plotDDPGProgress, plotEstimator, trajectoriesAnimation, plotCostColorMap, plotTimeColorMap, plotTimeDistanceTarget, plotFittsLaw, plotPerfSizeDist, plotVelocityProfile, plotXYPositions, plotXYEstimError, plotXYEstimErrorOfSpeed, plotArticularPositions, plotInitPos, plotMuscularActivations, plotScattergram, plotHitDispersion, plotExperimentSetup, plotTrajsInRepo, plotManipulability, plotManipulability2


from Utils.Chrono import Chrono
from Utils.ReadXmlFile import ReadXmlFile


#----------------------------- main list of available actions ----------------------------------------------------------------------

def printMainMenu():
    print('    DDPG:')
    print('        1 train DDPG for all targets')
    print('        2 generate results from DDPG controllers')
    print('        3 plot velocity profiles')
    print('        4 plot XY and articular positions')
    print('        5 plot muscular activations')
    print('        6 plot cost Map')                  
    print('        7 plot Time x Distance for Targets')                  
    print('        8 plot Size x Dist')                  
    print('        9 plot Fitts Law')                  
    print('        10 plot Map Time x Trajectory')
    print('        11 show trajectory animations (all)')                 
    print('        12 plot Hit dispersion')
    print('        13 train DDPG for one target')
    print('        14 plot DDPG cost progress')
    print('        16 generate rich results from DDPG controllers')
    print('        17 plot successful trajectories in repository')
    print('        18 plot XY estimation error')
    print('        19 plot Estimation error as function of velocity')
    print('        20 plot Experimental set-up')
    print('        21 plot Directional Manipulability')
    print('        22 plot Manipulability')
    print('        23 plot Estimation')
    print('        24 train one DDPG for one point')
    print('        25 plot DDPG One point cost progress')
    print('        26 generate results from DDPG One point')
    print('        27 plot XY  for one target') 
    
    
def runChoice():
    checkL = True
    fileName = raw_input('Name of file to load setup : ')
    rs = ReadXmlFile(fileName)
    while checkL:
        printMainMenu()
        choix = input('Enter the number corresponding to the script you want to run: ')
        choix = int(choix)
        if(chooseFunction(choix, rs)==0): break

def chooseFunction(choix, rs):
    if choix == 1:
        c = Chrono()
        #TODO: DDPGForAllTargetSizes
        #launchDDPGForAllTargetSizesMulti(rs)
        c.stop()
    elif choix == 2:
        #TODO: Choose the kinematic model
        nameTheta = raw_input('Name of the controller file: ')
        name = raw_input('Folder where you want to save the results: ')
        nbret = input("Number of repeat for each trajectory (int): ")
        nbret = int(nbret)
        generateFromDDPG(nbret, rs, nameTheta, name)
    elif choix == 3:
        nameF = raw_input('Folder where the results are saved: ')
        plotVelocityProfile("OPTI",rs,nameF)
    elif choix == 4:
        nameF = raw_input('Folder where the results are saved: ')
        rorc = input("enter 1 if XY or 2 if Joint results: ")
        rorc = int(rorc)
        if rorc == 1:
            plotXYPositions("OPTI",rs, nameF,"All",False)
        else:
            plotArticularPositions("OPTI",rs, nameF)
    elif choix == 5:
        nameF = raw_input('Folder where the results are saved: ')
        tSize = raw_input('Target Size: ')
        plotMuscularActivations("OPTI",rs,nameF,tSize)
    elif choix == 6:
        nameF = raw_input('Folder where the results are saved: ')
        #tSize = raw_input('Target Size: ')
        #plotCostColorMap("OPTI",nameF,tSize)
        plotCostColorMap("OPTI",rs, nameF)
    elif choix == 7:
        nameF = raw_input('Folder where the results are saved: ')
        plotTimeDistanceTarget(nameF, rs)
    elif choix == 8:
        nameF = raw_input('Folder where the results are saved: ')
        plotPerfSizeDist(nameF, rs)
    elif choix == 9:
        nameF = raw_input('Folder where the results are saved: ')
        plotFittsLaw(nameF, rs)
    elif choix == 10:
        nameF = raw_input('Folder where the results are saved: ')
        plotTimeColorMap("OPTI",rs, nameF)
    elif choix == 11:
        rorc = input("enter 0 if Brent, 1 if Regression or 2 if DDPG results: ")
        rorc = int(rorc)
        if rorc == 0:
            trajectoriesAnimation("Brent", rs)
        elif rorc == 1:
            nameF = raw_input('Folder where the results are saved: ')
            trajectoriesAnimation("RBFN",rs, nameF)
        elif rorc == 2:
            nameF = raw_input('Folder where the results are saved: ')
            tSize = raw_input('Target Size: ')
            trajectoriesAnimation("OPTI",rs, nameF, tSize)
    elif choix == 12:
        nameF = raw_input('Folder where the results are saved: ')
        rorc = input("enter 1 if RBFN or 2 if DDPG results: ")
        #plotHitDispersion(nameF,"0.05")
        rorc = int(rorc)
        if rorc == 1:
            plotScattergram("RBFN",nameF, rs)
        elif rorc == 2:
            plotScattergram("OPTI",nameF, rs)
    elif choix == 13:
        tSize = raw_input('Target Size: ')
        c = Chrono()
        launchDDPGForSpecificTargetSize(float(tSize),rs)
        c.stop()
    elif choix == 14:
        plotDDPGProgress(rs)
    elif choix == 15:
        name = raw_input('Name of the Regression controller file: ')
        fname = raw_input('Folder where you want to save the results: ')
        nbret = input("Number of repeat for each trajectory (int): ")
        c = Chrono()
        generateRichDataFromRegression(nbret,rs, name, fname)
        c.stop()
    elif choix == 16:
        nameTheta = raw_input('Name of the controller file: ')
        name = raw_input('Folder where you want to save the results: ')
        nbret = input("Number of repeat for each trajectory (int): ")
        nbret = int(nbret)
        c = Chrono()
        generateRichDataFromDDPG(nbret,rs, nameTheta, name)
        c.stop()
    elif choix == 17:
        plotTrajsInRepo()
    elif choix == 18:
        nameF = raw_input('Folder where the results are saved: ')
        plotXYEstimError("OPTI",rs,nameF,"All")
    elif choix == 19:
        nameF = raw_input('Folder where the results are saved: ')
        plotXYEstimErrorOfSpeed("OPTI",rs,nameF,"All")
    elif choix == 20:
        plotExperimentSetup(rs)
    elif choix == 21:
        plotManipulability(rs)
    elif choix == 22:
        plotManipulability2(rs)
    elif choix == 23:
        plotEstimator(rs, 0.04, 0., 0.25)
    elif choix == 24:
        tSize = raw_input('Target Size: ')
        c = Chrono()
        launchDDPGForAllPoint(rs,float(tSize))
        #launchDDPGForSpecificTargetSizeAndSpeceficBeginning(float(tSize), rs, [0, [0, 0.2]])
        c.stop()
    elif choix == 25:
        size=raw_input('Target Size: ')

        while True:
            print("    Enter the number of the point you want sea, q for quit")
            point=raw_input('Point: ')
            if(point=="q"): 
                break
            plotDDPGOnePointProgress(rs,size, point)
    elif choix == 26:
        #TODO: Choose the kinematic model
        nameTheta = raw_input('Name of the controller file: ')
        name = raw_input('Folder where you want to save the results: ')
        nbret = input("Number of repeat for each trajectory (int): ")
        nbret = int(nbret)
        generateFromDDPGNController(nbret, rs, nameTheta, name)
    elif choix == 27:
        nameF = raw_input('Folder where the results are saved: ')
        rorc = raw_input("Target Size: ")
        plotXYPositions("OPTI",rs, nameF,rorc,False)
    else :
        return 0
    return 1


runChoice()