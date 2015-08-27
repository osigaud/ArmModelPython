#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: Experiments

Description: Class used to generate all the trajectories of the experimental setup and also used for CMAES optimization
'''
import os

import numpy as np

from Utils.ThetaNormalization import normalization, unNormalization
from Utils.ReadSetupFile import ReadSetupFile
from Utils.FileReading import dicToArray, getInitPos
from Utils.Chrono import Chrono

from GlobalVariables import BrentTrajectoriesFolder, pathDataFolder

from TrajMaker import TrajMaker

def checkIfFolderExists(name):
    if not os.path.isdir(name):
        os.makedirs(name)

def findDataFileName(foldername, name, extension):
    i = 1
    checkIfFolderExists(foldername)
    tryName = name + "1" + extension
    while tryName in os.listdir(foldername):
        i += 1
        tryName = name + str(i) + extension
    filename = foldername + tryName
    return filename

#------------------------------------------------------------------------------

class Experiments:
    def __init__(self, rs, sizeOfTarget, saveTraj, foldername, thetafile):
        '''
    	Initializes parameters used to run functions below
    
    	Inputs:
     	'''
        self.rs = rs
        self.name = "Experiments"
        self.call = 0
        self.numfeats = rs.numfeats
        self.dimState = rs.inputDim
        self.dimOutput = rs.outputDim
        self.numberOfRepeat = rs.numberOfRepeatEachTraj
        self.foldername = foldername
        self.tm = TrajMaker(rs, sizeOfTarget, saveTraj, thetafile)
        self.posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
        self.costStore = []
        self.CMAEScostStore = []
        self.trajTimeStore = []
        self.bestCost = -10000.0
        self.lastCoord = []
        self.maxT = 1
        self.popSize = 0

    def setTheta(self, theta):
        self.tm.setTheta(theta)
    
    def initTheta(self, theta):
        '''
     	Input:		-theta: controller ie vector of parameters, numpy array
    	'''
        self.theta = np.copy(theta)
        #reshaping of the parameters vector because this function is used by the cmaes algorithm and 
        #cmaes feeds the function with a one dimension numpy array but in the rest of the algorithm the 2 dimensions numpy array is expected for the vector of parameters theta
        th = unNormalization(self.theta, self.maxT)
        self.theta = np.asarray(th).reshape((self.dimOutput, self.numfeats**self.dimState))
        #print ("theta Exp: ", self.theta)

        self.setTheta(self.theta)

    def saveCost(self):
        filename = findDataFileName(self.foldername+"Cost/","traj",".cost")
        filenameTime = findDataFileName(self.foldername+"TrajTime/","traj",".time")
        filenameX = findDataFileName(self.foldername+"finalX/","x",".last")
        np.savetxt(filename, self.costStore)
        np.savetxt(filenameTime, self.trajTimeStore)
        np.savetxt(filenameX, self.lastCoord)
         
    def runOneTrajectory(self, x, y):
        filename = findDataFileName(self.foldername+"Log/","traj",".log")
        cost, trajTime, lastX = self.tm.runTrajectory(x, y, filename)
        if lastX != -1000:
            self.lastCoord.append(lastX)
        return cost, trajTime
            
    def runTrajectoriesForCostMap(self, repeat):
        globCost = []
        xy = np.loadtxt(pathDataFolder + "PosCircu210")
        for el in xy:
            costAll, trajTimeAll = np.zeros(repeat), np.zeros(repeat)
            for i in range(repeat):
                costAll[i], trajTimeAll[i]  = self.runOneTrajectory(el[0], el[1]) 
            meanCost = np.mean(costAll)
            meanTrajTime = np.mean(trajTimeAll)
            self.costStore.append([el[0], el[1], meanCost])
            self.trajTimeStore.append([el[0], el[1], meanTrajTime])
            globCost.append(meanCost)
        return np.mean(globCost)
            
    def runTrajectoriesForResultsGeneration(self, repeat):
        globCost = []
        for xy in self.posIni:
            costAll, trajTimeAll = np.zeros(repeat), np.zeros(repeat)
            for i in range(repeat):
                costAll[i], trajTimeAll[i]  = self.runOneTrajectory(xy[0], xy[1]) 
            meanCost = np.mean(costAll)
            meanTrajTime = np.mean(trajTimeAll)
            self.costStore.append([xy[0], xy[1], meanCost])
            self.trajTimeStore.append([xy[0], xy[1], meanTrajTime])
            globCost.append(meanCost)
        return np.mean(globCost)
    
    def runTrajectoriesCMAES(self, theta):
        '''
    	Generates all the trajectories of the experimental setup and return the mean cost. This function is used by cmaes to optimize the controller.
    
    	Input:		-theta: vector of parameters, one dimension normalized numpy array
    
    	Ouput:		-meanAll: the mean of the cost of all trajectories generated, float
    	'''
        if (self.call==0):
            self.localBestCost = -10000.0
        c = Chrono()
        self.initTheta(theta)
        #compute all the trajectories x times each, x = numberOfRepeat
        meanCost = self.runTrajectoriesForResultsGeneration(self.numberOfRepeat)
        c.stop()

        print("Indiv #: ", self.call, "\n Cost: ", meanCost)

        if meanCost>self.localBestCost:
            self.localBestCost = meanCost

        if meanCost>self.bestCost:
            self.bestCost = meanCost
            extension = ".save" + str(meanCost)
            filename = findDataFileName(self.foldername+"Theta/", "theta", extension)
            np.savetxt(filename, self.theta)

        self.call += 1
        self.call = self.call%self.popSize

        if (self.call==0):
            self.CMAEScostStore.append(self.localBestCost)
            costfoldername = self.foldername+"Cost/"
            checkIfFolderExists(costfoldername)
            np.savetxt(costfoldername+"cmaesCost.log",self.CMAEScostStore) #Note: inefficient, should rather add to the file

        return (300.0-meanCost)/20.0
    
    
    
    
