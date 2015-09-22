#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: Main

Description: useful functions to run cmaes and some scripts to run trajectories
'''
import os
import cma
import numpy as np
from shutil import copyfile

from multiprocessing.pool import Pool

from Utils.ReadSetupFile import ReadSetupFile
from Utils.ThetaNormalization import normalization, unNormalization

from ArmModel.Arm import Arm
from Experiments.Experiments import Experiments, checkIfFolderExists

def copyRBFNtoCMAES(rs, name, size):
    savenametheta = rs.RBFNpath + name + ".theta"
    savenamestruct = rs.RBFNpath + name + ".struct"
    cmaname =  rs.CMAESpath + str(size) + "/"
    checkIfFolderExists(cmaname)
    copyfile(savenametheta, cmaname + name + ".theta")
    copyfile(savenamestruct, cmaname + name + ".struct")

def GenerateDataFromTheta(rs, sizeOfTarget, foldername, thetaFile, repeat, save):
    exp = Experiments(rs, sizeOfTarget, save, foldername,thetaFile)
    cost, time = exp.runTrajectoriesForResultsGeneration(repeat)
    print("Average cost: ", cost)
    print("Average time: ", time)
    print("foldername : ", foldername)
    if (save):
        exp.saveCost()

def GenerateRichDataFromTheta(rs, sizeOfTarget, foldername, thetaFile, repeat, save):
    exp = Experiments(rs, sizeOfTarget, save, foldername,thetaFile)
    cost = exp.runRichTrajectories(repeat)
    print("Average cost: ", cost)
    print("foldername : ", foldername)
    if (save):
        exp.saveCost()

def generateFromCMAES(repeat, thetaFile, saveDir = 'Data'):
    rs = ReadSetupFile()
    for el in rs.sizeOfTarget:
        thetaName = rs.CMAESpath + str(el) + "/" + thetaFile
        saveName = rs.CMAESpath + str(el) + "/" + saveDir + "/"
        GenerateDataFromTheta(rs,el,saveName,thetaName,repeat,True)
    print("CMAES:End of generation")

def generateRichDataFromCMAES(repeat, thetaFile, saveDir = 'Data'):
    rs = ReadSetupFile()
    for el in rs.sizeOfTarget:
        thetaName = rs.CMAESpath + str(el) + "/" + thetaFile
        saveName = rs.CMAESpath + str(el) + "/" + saveDir + "/"
        GenerateRichDataFromTheta(rs,el,saveName,thetaName,repeat,True)
    print("CMAES:End of generation")

def generateFromRBFN(repeat, thetaFile, saveDir):
    rs = ReadSetupFile()
    thetaName = rs.RBFNpath + thetaFile
    saveName = rs.RBFNpath + saveDir + "/"
    GenerateDataFromTheta(rs,0.05,saveName,thetaName,repeat,True)
    print("RBFN:End of generation")

def generateRichDataFromRBFN(repeat, thetaFile, saveDir):
    rs = ReadSetupFile()
    thetaName = rs.RBFNpath + thetaFile
    saveName = rs.RBFNpath + saveDir + "/"
    GenerateRichDataFromTheta(rs,0.05,saveName,thetaName,repeat,True)
    print("RBFN:End of generation")

def launchCMAESForSpecificTargetSize(sizeOfTarget, thetaFile, save):
    '''
    Run cmaes for a specific target size

    Input:	-sizeOfTarget, size of the target, float
    '''
    print("Starting the CMAES Optimization for target " + str(sizeOfTarget) + " !")
    rs = ReadSetupFile()
    foldername = rs.CMAESpath + str(sizeOfTarget) + "/"
    thetaname = foldername + thetaFile
    if save:
        copyRBFNtoCMAES(rs, thetaFile, sizeOfTarget)

    #Initializes all the class used to generate trajectory
    exp = Experiments(rs, sizeOfTarget, False, foldername, thetaname)
    exp.popSize = rs.popsizeCmaes
    theta = exp.tm.controller.theta
    thetaIn = theta.flatten()
    thetaCMA, max = normalization(thetaIn)
    exp.maxT = max
    print ("max normalisation :", max)
    #print ("theta CMA : ", thetaCMA)

    #run the optimization (cmaes)
    resCma = cma.fmin(exp.runTrajectoriesCMAES, thetaCMA, rs.sigmaCmaes, options={'maxiter':rs.maxIterCmaes, 'popsize':rs.popsizeCmaes})
    print("End of optimization for target " + str(sizeOfTarget) + " !")
    
def launchCMAESForAllTargetSizes(thetaname, save):
    rs = ReadSetupFile()
    for el in rs.sizeOfTarget:
        launchCMAESForSpecificTargetSize(el, thetaname,save)

#--------------------------- multiprocessing -------------------------------------------------------
    
def launchCMAESForAllTargetSizesMulti():
    '''
    Launch in parallel (on differents processor) the cmaes optimization for each target size
    '''
    #initializes setup variables
    rs = ReadSetupFile()
    #initializes a pool of worker, ie multiprocessing
    p = Pool()
    #run cmaes on each targets size on separate processor
    p.map(launchCMAESForSpecificTargetSize, rs.sizeOfTarget, "theta")
