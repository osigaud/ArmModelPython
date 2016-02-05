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
from Utils.Chrono import Chrono

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
    exp = Experiments(rs, sizeOfTarget, save, foldername,thetaFile,rs.popsizeCmaes,rs.period)
    cost, time = exp.runTrajectoriesForResultsGeneration(repeat)
    print("Average cost: ", cost)
    print("Average time: ", time)
    print("foldername : ", foldername)
    if (save):
        exp.saveCost()

def GenerateRichDataFromTheta(rs, sizeOfTarget, foldername, thetaFile, repeat, save):
    exp = Experiments(rs, sizeOfTarget, save, foldername,thetaFile,rs.popsizeCmaes,rs.period)
    cost = exp.runRichTrajectories(repeat)
    print("Average cost: ", cost)
    print("foldername : ", foldername)
    if (save):
        exp.saveCost()

def generateFromCMAES(repeat, setupFile, thetaFile, saveDir = 'Data'):
    rs = ReadSetupFile(setupFile)
    for el in rs.sizeOfTarget:
        c = Chrono()
        thetaName = rs.CMAESpath + str(el) + "/" + thetaFile
        saveName = rs.CMAESpath + str(el) + "/" + saveDir + "/"
        GenerateDataFromTheta(rs,el,saveName,thetaName,repeat,True)
        c.stop()
    print("CMAES:End of generation")

def generateRichDataFromCMAES(repeat, setupFile, thetaFile, saveDir = 'Data'):
    rs = ReadSetupFile(setupFile)
    for el in rs.sizeOfTarget:
        thetaName = rs.CMAESpath + str(el) + "/" + thetaFile
        saveName = rs.CMAESpath + str(el) + "/" + saveDir + "/"
        GenerateRichDataFromTheta(rs,el,saveName,thetaName,repeat,True)
    print("CMAES:End of generation")

def generateFromRegression(repeat, setupFile, thetaFile, saveDir):
    rs = ReadSetupFile(setupFile)
    thetaName = rs.path + thetaFile
    saveName = rs.path + saveDir + "/"
    GenerateDataFromTheta(rs,0.05,saveName,thetaName,repeat,True)
    print("Regression:End of generation")

def generateRichDataFromRegression(repeat, setupFile,thetaFile, saveDir):
    rs = ReadSetupFile(setupFile)
    thetaName = rs.path + thetaFile
    saveName = rs.path + saveDir + "/"        
    GenerateRichDataFromTheta(rs,0.05,saveName,thetaName,repeat,True)
    print("Regression:End of generation")

def launchCMAESForSpecificTargetSize(sizeOfTarget, setupFile, thetaFile, save):
    '''
    Run cmaes for a specific target size

    Input:	-sizeOfTarget, size of the target, float
    '''
    print("Starting the CMAES Optimization for target " + str(sizeOfTarget) + " !")
    rs = ReadSetupFile(setupFile)
    foldername = rs.CMAESpath + str(sizeOfTarget) + "/"
    thetaname = foldername + thetaFile
    if save:
        copyRBFNtoCMAES(rs, thetaFile, sizeOfTarget)

    #Initializes all the class used to generate trajectory
    exp = Experiments(rs, sizeOfTarget, False, foldername, thetaname,rs.popsizeCmaes,rs.period)
    theta = exp.tm.controller.theta
    thetaCMA = theta.flatten()

    #run the optimization (cmaes)
    resCma = cma.fmin(exp.runTrajectoriesCMAES, thetaCMA, rs.sigmaCmaes, options={'maxiter':rs.maxIterCmaes, 'popsize':rs.popsizeCmaes, 'CMA_diagonal':True, 'verb_log':50, 'verb_disp':1,'termination_callback':term()})
    print("End of optimization for target " + str(sizeOfTarget) + " !")
    
def launchCMAESForAllTargetSizes(setupFile, thetaName, save):
    rs = ReadSetupFile(setupFile)
    for el in rs.sizeOfTarget:
        launchCMAESForSpecificTargetSize(el, thetaName,save)

def term():
    return False

#--------------------------- multiprocessing -------------------------------------------------------
    
def launchCMAESForAllTargetSizesMulti(setupFile):
    '''
    Launch in parallel (on differents processor) the cmaes optimization for each target size
    '''
    #initializes setup variables
    rs = ReadSetupFile(setupFile)
    #initializes a pool of worker, ie multiprocessing
    p = Pool()
    #run cmaes on each targets size on separate processor
    p.map(launchCMAESForSpecificTargetSize, rs.sizeOfTarget, "theta")
