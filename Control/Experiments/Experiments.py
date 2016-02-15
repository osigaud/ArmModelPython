#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: Experiments

Description: Class used to generate all the trajectories of the experimental setup and also used for CMAES optimization
'''


import numpy as np

#from Utils.ThetaNormalization import normalization, unNormalization
from Utils.Chrono import Chrono

from GlobalVariables import pathDataFolder

from TrajMaker import TrajMaker
from Utils.FileWritting import checkIfFolderExists, findDataFilename

#------------------------------------------------------------------------------

class Experiments:
    def __init__(self, rs, sizeOfTarget, saveTraj, foldername, thetafile, popSize, period):
        '''
    	Initializes parameters used to run functions below
    
    	Inputs:
     	'''
        self.rs = rs
        self.name = "Experiments"
        self.call = 0
        self.dimState = rs.inputDim
        self.dimOutput = rs.outputDim
        self.numberOfRepeat = rs.numberOfRepeatEachTraj
        self.foldername = foldername
        self.tm = TrajMaker(rs, sizeOfTarget, saveTraj, thetafile)
        self.posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
        self.costStore = []
        self.CMAESCostStore = []
        self.CMAESTimeStore = []
        self.trajTimeStore = []
        self.bestCost = -10000.0
        self.lastCoord = []
        self.popSize = popSize
        self.period = period
        
    def printLastCoordInfo(self):
        vec = np.array(self.lastCoord)
        print ("moyenne : "+ str(np.mean(vec)))
        print ("min : " + str(np.min(vec)))
        print ("max :" + str(np.max(vec)))
    

    def initTheta(self, theta):
        '''
     	Input:		-theta: controller ie vector of parameters, numpy array
    	'''
        self.theta=theta
        self.tm.setTheta(self.theta)

    def saveCost(self): 
        filename = findDataFilename(self.foldername+"Cost/","traj",".cost")
        filenameTime = findDataFilename(self.foldername+"TrajTime/","traj",".time")
        filenameX = findDataFilename(self.foldername+"finalX/","x",".last")
        np.savetxt(filename, self.costStore)
        np.savetxt(filenameTime, self.trajTimeStore)
        np.savetxt(filenameX, self.lastCoord)
         
    def runOneTrajectory(self, x, y):
        #self.tm.saveTraj = True
        cost, trajTime, lastX = self.tm.runTrajectory(x, y, self.foldername)
        #print "Exp local x y cost : ", x, y, cost
        if lastX != -1000:
            self.lastCoord.append(lastX)
        return cost, trajTime
            
    def runRichTrajectories(self, repeat):
        globCost = []
        xy = np.loadtxt(pathDataFolder + "PosCircu540")
        #xy = np.loadtxt(pathDataFolder + "PosSquare")
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
        globTime = []
        for xy in self.posIni:
            costAll, trajTimeAll = np.zeros(repeat), np.zeros(repeat)
            for i in range(repeat):
                costAll[i], trajTimeAll[i]  = self.runOneTrajectory(xy[0], xy[1]) 
            meanCost = np.mean(costAll)
            meanTrajTime = np.mean(trajTimeAll)
            self.costStore.append([xy[0], xy[1], meanCost])
            self.trajTimeStore.append([xy[0], xy[1], meanTrajTime])
            globCost.append(meanCost)
            globTime.append(meanTrajTime)
        #self.printLastCoordInfo()
        return np.mean(globCost), np.mean(globTime)
    
    def runTrajectoriesCMAES(self, theta):
        '''
    	Generates all the trajectories of the experimental setup and return the mean cost. This function is used by cmaes to optimize the controller.
    
    	Input:		-theta: vector of parameters, one dimension normalized numpy array
    
    	Ouput:		-meanAll: the mean of the cost of all trajectories generated, float
    	'''
        if (self.call==0):
            self.localBestCost = -1000000.0
            self.localWorstCost = 1000000.0
            self.localBestTime = -1000000.0
            self.localWorstTime = 1000000.0
            self.periodMeanCost = 0.0
            self.periodMeanTime = 0.0
        c = Chrono()
        self.initTheta(theta)
        #print "theta avant appel :", theta
        #compute all the trajectories x times each, x = numberOfRepeat
        meanCost, meanTime = self.runTrajectoriesForResultsGeneration(self.numberOfRepeat)
        #cma.plot()
        #opt = cma.CMAOptions()
        #print "CMAES options :", opt
        c.stop()

        print("Indiv #: ", self.call, "\n Cost: ", meanCost)

        if meanCost>self.localBestCost:
            self.localBestCost = meanCost

        if meanTime>self.localBestTime:
            self.localBestTime = meanTime

        if meanCost<self.localWorstCost:
            self.localWorstCost = meanCost

        if meanTime<self.localWorstTime:
            self.localWorstTime = meanTime

        if meanCost>self.bestCost:
            self.bestCost = meanCost
            if meanCost>0:
                extension = ".save" + str(meanCost)
                filename = findDataFilename(self.foldername+"Theta/", "theta", extension)
                np.savetxt(filename, self.theta)
                filename2 = self.foldername + "Best.theta"
                np.savetxt(filename2, self.theta)
        
        self.periodMeanCost += meanCost
        self.periodMeanTime += meanTime

        self.call += 1
        self.call = self.call%self.period

        if (self.call==0):
            self.periodMeanCost = self.periodMeanCost/self.period
            self.periodMeanTime = self.periodMeanTime/self.period
            self.CMAESCostStore.append((self.localWorstCost,self.periodMeanCost,self.localBestCost))
            self.CMAESTimeStore.append((self.localWorstTime,self.periodMeanTime,self.localBestTime))
            costfoldername = self.foldername+"Cost/"
            checkIfFolderExists(costfoldername)
            np.savetxt(costfoldername+"cmaesCost.log",self.CMAESCostStore) #Note: inefficient, should rather add to the file
            np.savetxt(costfoldername+"cmaesTime.log",self.CMAESTimeStore) #Note: inefficient, should rather add to the file

        return 10.0*(self.rs.rhoCF-meanCost)/self.rs.rhoCF
    
    
    
    
