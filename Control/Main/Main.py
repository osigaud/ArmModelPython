#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: Main

Description: useful functions to run cmaes and some scripts to run trajectories
'''
#TODO: Make a optimisation and CMAES class
import os
import cma
import numpy as np
from shutil import copyfile

from multiprocessing.pool import Pool

from Utils.ReadXmlFile import ReadXmlFile
from Utils.Chrono import Chrono

from ArmModel.Arm import Arm
from Experiments.Experiments import Experiments, checkIfFolderExists

def copyRegressiontoCMAES(rs, name, size):
    cmaname =  rs.CMAESpath + str(size) + "/"
    checkIfFolderExists(cmaname)
    savenametheta = rs.path + name + ".theta"
    copyfile(savenametheta, cmaname + name + ".theta")
    
    if(rs.regression=="RBFN"):
        savenamestruct = rs.path + name + ".struct"
        copyfile(savenamestruct, cmaname + name + ".struct")

def GenerateDataFromTheta(rs, sizeOfTarget, foldername, thetaFile, repeat, save):
    os.system("rm "+foldername+"Log/*.log")
    exp = Experiments(rs, sizeOfTarget, save, foldername,thetaFile,rs.popsizeCmaes,rs.period)
    cost, time = exp.runTrajectoriesForResultsGeneration(repeat)
    print("Average cost: ", cost)
    print("Average time: ", time)
    print("foldername : ", foldername)
    if (save):
        exp.saveCost()

def GenerateRichDataFromTheta(rs, sizeOfTarget, foldername, thetaFile, repeat, save):
    os.system("rm "+foldername+"Log/*.log")
    exp = Experiments(rs, sizeOfTarget, save, foldername,thetaFile,rs.popsizeCmaes,rs.period)
    cost = exp.runRichTrajectories(repeat)
    print("Average cost: ", cost)
    print("foldername : ", foldername)
    if (save):
        exp.saveCost()

def generateFromCMAES(repeat, setupFile, thetaFile, saveDir = 'Data'):
    rs = ReadXmlFile(setupFile)
    for el in rs.sizeOfTarget:
        c = Chrono()
        thetaName = rs.CMAESpath + str(el) + "/" + thetaFile
        saveName = rs.CMAESpath + str(el) + "/" + saveDir + "/"
        GenerateDataFromTheta(rs,el,saveName,thetaName,repeat,True)
        c.stop()
    print("CMAES:End of generation")

def generateRichDataFromCMAES(repeat, setupFile, thetaFile, saveDir = 'Data'):
    rs = ReadXmlFile(setupFile)
    for el in rs.sizeOfTarget:
        thetaName = rs.CMAESpath + str(el) + "/" + thetaFile
        saveName = rs.CMAESpath + str(el) + "/" + saveDir + "/"
        GenerateRichDataFromTheta(rs,el,saveName,thetaName,repeat,True)
    print("CMAES:End of generation")

def generateFromRegression(repeat, setupFile, thetaFile, saveDir):
    rs = ReadXmlFile(setupFile)
    thetaName = rs.path + rs.thetaFile
    saveName = rs.path + saveDir + "/"
    GenerateDataFromTheta(rs,0.05,saveName,thetaName,repeat,True)
    print("Regression:End of generation")

def generateRichDataFromRegression(repeat, setupFile,thetaFile, saveDir):
    rs = ReadXmlFile(setupFile)
    thetaName = rs.path + thetaFile
    saveName = rs.path + saveDir + "/"        
    GenerateRichDataFromTheta(rs,0.05,saveName,thetaName,repeat,True)
    print("Regression:End of generation")

def launchCMAESForSpecificTargetSize(sizeOfTarget, setupFile, save):
    '''
    Run cmaes for a specific target size

    Input:	-sizeOfTarget, size of the target, float
            -setuFile, file of setup, string
            -save, for saving result, bool
    '''
    print("Starting the CMAES Optimization for target " + str(sizeOfTarget) + " !")
    rs = ReadXmlFile(setupFile)
    foldername = rs.CMAESpath + str(sizeOfTarget) + "/"
    thetaname = foldername + rs.thetaFile
    if save:
        copyRegressiontoCMAES(rs, rs.thetaFile, sizeOfTarget)

    #Initializes all the class used to generate trajectory
    exp = Experiments(rs, sizeOfTarget, False, foldername, thetaname,rs.popsizeCmaes,rs.period)
    theta = exp.tm.controller.getTheta()
    thetaCMA = theta.flatten()

    #run the optimization (cmaes)
    resCma = cma.fmin(exp.runTrajectoriesCMAES, thetaCMA, rs.sigmaCmaes, options={'maxiter':rs.maxIterCmaes, 'popsize':rs.popsizeCmaes, 'CMA_diagonal':True, 'verb_log':50, 'verb_disp':1,'termination_callback':term()})
    print("End of optimization for target " + str(sizeOfTarget) + " !")
    
def launchCMAESForAllTargetSizes(setupFile, thetaName, save):
    rs = ReadXmlFile(setupFile)
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
    rs = ReadXmlFile(setupFile)
    #initializes a pool of worker, ie multiprocessing
    p = Pool()
    #run cmaes on each targets size on separate processor
    p.map(launchCMAESForSpecificTargetSize, rs.sizeOfTarget, "theta")
