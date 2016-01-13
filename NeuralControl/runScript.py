#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher + Olivier Sigaud

Module: runScript

Description: main script to run what we want in the project
'''

import numpy as np
import random as rd

from Main.Main import generateFromNN, generateFromCMAES, generateRichDataFromNN, generateRichDataFromCMAES, launchCMAESForAllTargetSizes, launchCMAESForSpecificTargetSize

from Regression.RunRegressionNN import runNN, UnitTestNN, UnitTestNNController

from Plot.plotFunctions import trajectoriesAnimation, plotCostColorMap, plotTimeColorMap, plotTimeDistanceTarget, plotFittsLaw, plotPerfSizeDist, plotVelocityProfile, plotXYPositions, plotXYEstimError, plotXYEstimErrorOfSpeed, plotArticularPositions, plotInitPos, plotMuscularActivations, plotScattergram, plotHitDispersion, plotExperimentSetup, plotCMAESProgress, plotTrajsInRepo, plotManipulability, plotManipulability2

from Utils.Chrono import Chrono
from Utils.ReadSetupFile import ReadSetupFile
from GlobalVariables import pathDataFolder

#----------------------------- main list of available actions ----------------------------------------------------------------------

def printMainMenu():
    print('Available scripts:')
    print('	Brent:')
    print('		1 plot velocity profiles')
    print('		2 plot articular positions')
    print('		3 plot XY positions')
    print('		4 plot muscular activations')
    print('-------------------------------------------------')
    print('	NN:')
    print('		5 train from Brent data')
    print('		6 generate results from NN controller')
    print('		7 plot velocity profiles')
    print('		8 plot XY positions')
    print('		9 plot muscular activations')
    print('		10 plot cost Map')
    print('		28 plot XY estimation error')
    print('		29 plot Estimation error as function of velocity')
    print('-------------------------------------------------')
    print('	CMAES:')
    print('		11 train CMAES for all targets')
    print('		12 generate results from CMAES controllers')
    print('		13 plot velocity profiles')
    print('		14 plot XY and articular positions')
    print('		15 plot muscular activations')
    print('		16 plot cost Map')                  
    print('		17 plot Time x Distance for Targets')                  
    print('		18 plot Size x Dist')                  
    print('		19 plot Fitts Law')                  
    print('		20 plot Map Time x Trajectory')
    print('		21 show trajectory animations (all)')                 
    print('		22 plot Hit dispersion (CMAES or NN)')
    print('		23 train CMAES for one target')
    print('		24 plot CMAES cost progress')
    print('		25 generate rich results from NN controller')
    print('		26 generate rich results from CMAES controllers')
    print('		27 plot successful trajectories in repository')
    print('		30 plot XY estimation error')
    print('		31 plot Estimation error as function of velocity')
    print('		32 plot Experimental set-up')
    print('		33 plot Manipulability')
    print('		34 plot Manipulability2')

def runChoice():
    checkL = True
    while checkL:
        try:
            printMainMenu()
            choix = input('Enter the number corresponding to the script you want to run: ')
            choix = int(choix)
            checkL = False
        except:
            print("Enter a number.")
    chooseFunction(choix)

def runAuto():
    for choix in range(21):
        chooseFunction(choix)

def chooseFunction(choix):
    if choix == 1:
        plotVelocityProfile("Brent")
    elif choix == 2:
        plotArticularPositions("Brent")
    elif choix == 3:
        plotXYPositions("Brent")
    elif choix == 4:
        plotMuscularActivations("Brent")

#------------------------------------------- NN
    elif choix == 5:
        name = raw_input('Name of file to save the NN controller: ')
        c = Chrono()
#        runNN(name,True)
        runNN(name,False)
        c.stop()
    elif choix == 6:
        name = raw_input('Name of the NN controller file: ')
        fname = raw_input('Folder where you want to save the results: ')
        nbret = input("Number of repeat for each trajectory (int): ")
        c = Chrono()
        generateFromNN(nbret, name, fname)
        c.stop()
    elif choix == 7:
        nameF = raw_input('Folder where the results are saved: ')
        plotVelocityProfile("NN",nameF)
    elif choix == 8:
        nameF = raw_input('Folder where the results are saved: ')
        rorc = input("enter 1 if XY or 2 if Joint results: ")
        rorc = int(rorc)
        if rorc == 1:
            plotXYPositions("NN",nameF,"All",True)#False)#
        else:
            plotArticularPositions("NN",nameF)
    elif choix == 9:
        nameF = raw_input('Folder where the results are saved: ')
        plotMuscularActivations("NN",nameF)
    elif choix == 10:
        nameF = raw_input('Folder where the results are saved: ')
        plotCostColorMap("NN",nameF)
    elif choix == 28:
        nameF = raw_input('Folder where the results are saved: ')
        plotXYEstimError("NN",nameF,"All")
    elif choix == 29:
        nameF = raw_input('Folder where the results are saved: ')
        plotXYEstimErrorOfSpeed("NN",nameF,"All")

#------------------------------------------- CMAES
    elif choix == 11:
        rorc = input("enter 1 if from NN, anything if from previous CMAES: ")
        save = False
        rorc = int(rorc)
        if rorc == 1:
            save = True
        name = raw_input('Name of the controller file: ')
        c = Chrono()
        launchCMAESForAllTargetSizes(name,save)
        c.stop()
    elif choix == 12:
        nameTheta = raw_input('Name of the controller file: ')
        name = raw_input('Folder where you want to save the results: ')
        nbret = input("Number of repeat for each trajectory (int): ")
        nbret = int(nbret)
        generateFromCMAES(nbret, nameTheta, name)
    elif choix == 13:
        nameF = raw_input('Folder where the results are saved: ')
        plotVelocityProfile("CMAES",nameF)
    elif choix == 14:
        nameF = raw_input('Folder where the results are saved: ')
        rorc = input("enter 1 if XY or 2 if Joint results: ")
        rorc = int(rorc)
        if rorc == 1:
            plotXYPositions("CMAES",nameF,"All",False)
        else:
            tSize = raw_input('Target Size: ')
            plotArticularPositions("CMAES",nameF,tSize)
    elif choix == 15:
        nameF = raw_input('Folder where the results are saved: ')
        tSize = raw_input('Target Size: ')
        plotMuscularActivations("CMAES",nameF,tSize)
    elif choix == 16:
        nameF = raw_input('Folder where the results are saved: ')
        #tSize = raw_input('Target Size: ')
        #plotCostColorMap("CMAES",nameF,tSize)
        plotCostColorMap("CMAES",nameF)
    elif choix == 17:
        nameF = raw_input('Folder where the results are saved: ')
        plotTimeDistanceTarget(nameF)
    elif choix == 18:
        nameF = raw_input('Folder where the results are saved: ')
        plotPerfSizeDist(nameF)
    elif choix == 19:
        nameF = raw_input('Folder where the results are saved: ')
        plotFittsLaw(nameF)
    elif choix == 20:
        nameF = raw_input('Folder where the results are saved: ')
        plotTimeColorMap("CMAES",nameF)
    elif choix == 21:
        rorc = input("enter 0 if Brent, 1 if NN or 2 if CMAES results: ")
        rorc = int(rorc)
        if rorc == 0:
            trajectoriesAnimation("Brent")
        elif rorc == 1:
            nameF = raw_input('Folder where the results are saved: ')
            trajectoriesAnimation("NN",nameF)
        elif rorc == 2:
            nameF = raw_input('Folder where the results are saved: ')
            tSize = raw_input('Target Size: ')
            trajectoriesAnimation("CMAES",nameF, tSize)
    elif choix == 22:
        nameF = raw_input('Folder where the results are saved: ')
        rorc = input("enter 1 if NN or 2 if CMAES results: ")
        #plotHitDispersion(nameF,"0.05")
        rorc = int(rorc)
        if rorc == 1:
            plotScattergram("NN",nameF)
        elif rorc == 2:
            plotScattergram("CMAES",nameF)
    elif choix == 23:
        name = raw_input('Name of the controller file: ')
        rorc = input("enter 1 if from NN, anything if from previous CMAES: ")
        save = False
        rorc = int(rorc)
        if rorc == 1:
            save = True
        tSize = raw_input('Target Size: ')
        c = Chrono()
        launchCMAESForSpecificTargetSize(float(tSize),name,save)
        c.stop()
    elif choix == 24:
        plotCMAESProgress()
    elif choix == 25:
        name = raw_input('Name of the NN controller file: ')
        fname = raw_input('Folder where you want to save the results: ')
        nbret = input("Number of repeat for each trajectory (int): ")
        c = Chrono()
        generateRichDataFromNN(nbret, name, fname)
        c.stop()
    elif choix == 26:
        nameTheta = raw_input('Name of the controller file: ')
        name = raw_input('Folder where you want to save the results: ')
        nbret = input("Number of repeat for each trajectory (int): ")
        nbret = int(nbret)
        c = Chrono()
        generateRichDataFromCMAES(nbret, nameTheta, name)
        c.stop()
    elif choix == 27:
        plotTrajsInRepo()
    elif choix == 30:
        nameF = raw_input('Folder where the results are saved: ')
        plotXYEstimError("CMAES",nameF,"All")
    elif choix == 31:
        nameF = raw_input('Folder where the results are saved: ')
        plotXYEstimErrorOfSpeed("CMAES",nameF,"All")
    elif choix == 32:
        plotExperimentSetup()
    elif choix == 33:
        plotManipulability()
    elif choix == 34:
        plotManipulability2()

def setPosCircu15():
    rs=ReadSetupFile()
    filename = pathDataFolder + "PosCircu15"
    data = []
    for i in range(3):
        point = [rs.XTarget+ 0.0975*np.cos((-i-1)*np.pi/4), rs.YTarget+ 0.0975*np.sin((-i-1)*np.pi/4)]
        data.append(point)
    for i in range(5):
        point = [rs.XTarget+ 0.243*np.cos(-i*np.pi/8-np.pi/4), rs.YTarget+ 0.243*np.sin(-i*np.pi/8-np.pi/4)]
        data.append(point)
    for i in range(7):
        point = [rs.XTarget+ 0.39*np.cos(-i*np.pi/12-np.pi/4), rs.YTarget+ 0.39*np.sin(-i*np.pi/12-np.pi/4)]
        data.append(point)
    np.savetxt(filename,data)

    #------- **** --rayon de 0.1 à 0.6, angle de -pi/5 à -4pi/5 ---------
def setPosCircu540():
    rs=ReadSetupFile()
    filename2 = pathDataFolder + "PosCircu540"
    data2 = []
    for j in range(20):
        for i in range(27):
            point = [rs.XTarget+ (0.1+j*0.02)*np.cos(-i*np.pi/40-np.pi/5), rs.YTarget+ (0.1+j*0.02)*np.sin(-i*np.pi/40-np.pi/5)]
            data2.append(point)
    np.savetxt(filename2,data2)

def setPosCircu56():
    rs=ReadSetupFile()
    filename = pathDataFolder + "PosCircu56"
    data2 = []
    for j in range(8):
        for i in range(7):
            point = [rs.XTarget+ (0.1+j*0.05)*np.cos(-i*np.pi/12-np.pi/4), rs.YTarget+ (0.1+j*0.05)*np.sin(-i*np.pi/12-np.pi/4)]
            data2.append(point)
    np.savetxt(filename,data2)

def setPosCircu28():
    rs=ReadSetupFile()
    filename = pathDataFolder + "PosCircu28"
    data2 = []
    for j in range(4):
        for i in range(7):
            point = [rs.XTarget+ (0.1+j*0.1)*np.cos(-i*np.pi/12-np.pi/4), rs.YTarget+ (0.1+j*0.1)*np.sin(-i*np.pi/12-np.pi/4)]
            data2.append(point)
    np.savetxt(filename,data2)

def setPosSquare():
    rs=ReadSetupFile()
    filename3 = pathDataFolder + "PosSquare"
    data3 = []
    for j in range(20):
        for i in range(20):
            point = [rs.XTarget - 0.3 + i*0.03, rs.YTarget - 0.4 + j*0.01]
            data3.append(point)
    np.savetxt(filename3,data3)

def generateNNs():
     for i in range(20):
        c = Chrono()
        runNN("X"+str(i),True)
        c.stop()

def testNNs():
     for i in range(3):
        c = Chrono()
        generateFromNN(3, "R"+str(i), "SR"+str(i))
        c.stop()
'''
     for i in range(12):
        c = Chrono()
        generateFromNN(3, "X"+str(i), "SX"+str(i))
        c.stop()
'''

def plotNNs():
     for i in range(3):
        plotXYPositions("NN","SR"+str(i),"All",True)
'''
     for i in range(12):
        plotXYPositions("NN","SX"+str(i),"All",True)
'''

#plotInitPos("PosCircu540")  
#runAuto()
#generateFromNN(nbret, nameC)
#generateInitialPositions()

#generateNNs()
#testNNs()
#plotNNs()

#UnitTest()
#UnitTestNNController()
#UnitTestArmModel()

#runNN("Full",False)
#generateFromNN(3, "Full", "SFull")
#plotXYPositions("NN","SFull","All",True)

#setPosCircu28()
#plotXYPositions("CMAES", "A2", "0.04", True)

runChoice()

#UnitTestNNController()

