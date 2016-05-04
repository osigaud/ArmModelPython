#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher + Olivier Sigaud

Module: runScript

Description: script to run regression
'''


from Main.MainCMAES import generateFromRegression

from Regression.RunRegression import run


from Plot.plotFunctions import plotCostColorMap, plotVelocityProfile, plotXYPositions, plotXYEstimError, plotXYEstimErrorOfSpeed, plotArticularPositions, plotInitPos, plotMuscularActivations, plotScattergram, plotHitDispersion, plotExperimentSetup, plotCMAESProgress, plotTrajsInRepo, plotManipulability, plotManipulability2

from Utils.Chrono import Chrono
from Utils.ReadXmlFile import ReadXmlFile


#----------------------------- main list of available actions ----------------------------------------------------------------------

def printMainMenu():
    print('Available scripts:')
    print('    Brent:')
    print('        1 plot velocity profiles')
    print('        2 plot articular positions')
    print('        3 plot XY positions')
    print('        4 plot muscular activations')
    print('-------------------------------------------------')
    print('    Regression:')
    print('        5 train from Brent data')
    print('        6 generate results from Regression controller')
    print('        7 plot velocity profiles')
    print('        8 plot XY positions')
    print('        9 plot muscular activations')
    print('        10 plot cost Map')
    print('        11 plot XY estimation error')
    print('        12 plot Estimation error as function of velocity')
    print('-------------------------------------------------')
    
    
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
        plotVelocityProfile("Brent",rs)
    elif choix == 2:
        plotArticularPositions("Brent",rs)
    elif choix == 3:
        plotXYPositions("Brent",rs)
    elif choix == 4:
        plotMuscularActivations("Brent",rs)

#------------------------------------------- RBFN
    elif choix == 5:
        c = Chrono()
#        run(name,True)
        run(rs)
        c.stop()
    elif choix == 6:
        fname = raw_input('Folder where you want to save the results: ')
        nbret = input("Number of repeat for each trajectory (int): ")
        c = Chrono()
        generateFromRegression(nbret, rs, fname)
        c.stop()
    elif choix == 7:
        nameF = raw_input('Folder where the results are saved: ')
        plotVelocityProfile("Regression",rs, nameF)
    elif choix == 8:
        nameF = raw_input('Folder where the results are saved: ')
        rorc = input("enter 1 if XY or 2 if Joint results: ")
        rorc = int(rorc)
        if rorc == 1:
            plotXYPositions("Regression", rs, nameF,"All",True)#False)#
        else:
            plotArticularPositions("Regression",rs, nameF)
    elif choix == 9:
        nameF = raw_input('Folder where the results are saved: ')
        plotMuscularActivations("Regression",rs, nameF)
    elif choix == 10:
        nameF = raw_input('Folder where the results are saved: ')
        plotCostColorMap("Regression", rs, nameF)
    elif choix == 11:
        nameF = raw_input('Folder where the results are saved: ')
        plotXYEstimError("Regression", rs, nameF,"All")
    elif choix == 12:
        nameF = raw_input('Folder where the results are saved: ')
        plotXYEstimErrorOfSpeed("Regression", rs, nameF,"All")
        
        
runChoice()