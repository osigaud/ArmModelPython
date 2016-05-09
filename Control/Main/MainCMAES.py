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
from shutil import copyfile
from functools import partial
from multiprocess.pool import ThreadPool

from Utils.Chrono import Chrono

from Experiments.Experiments import Experiments
from Utils.FileWritting import checkIfFolderExists

def copyRegressiontoCMAES(rs, name, size):
    cmaname =  rs.CMAESpath + str(size) + "/"
    checkIfFolderExists(cmaname)
    savenametheta = rs.path + name + ".theta"
    copyfile(savenametheta, cmaname + name + ".theta")
    
    if(rs.regression=="RBFN"):
        savenamestruct = rs.path + name + ".struct"
        copyfile(savenamestruct, cmaname + name + ".struct")

def GenerateDataFromTheta(rs, sizeOfTarget, foldername, thetaFile, repeat, save):
    os.system("rm "+foldername+"Log/*.log 2>/dev/null")
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

def generateFromCMAES(repeat, rs, thetaFile, saveDir = 'Data'):
    for el in rs.sizeOfTarget:
        c = Chrono()
        thetaName = rs.CMAESpath + str(el) + "/" + thetaFile
        saveName = rs.CMAESpath + str(el) + "/" + saveDir + "/"
        GenerateDataFromTheta(rs,el,saveName,thetaName,repeat,True)
        c.stop()
    print("CMAES:End of generation")

def generateRichDataFromCMAES(repeat, rs, thetaFile, saveDir = 'Data'):
    for el in rs.sizeOfTarget:
        thetaName = rs.CMAESpath + str(el) + "/" + thetaFile
        saveName = rs.CMAESpath + str(el) + "/" + saveDir + "/"
        GenerateRichDataFromTheta(rs,el,saveName,thetaName,repeat,True)
    print("CMAES:End of generation")

def generateFromRegression(repeat, rs, saveDir):
    thetaName = rs.path + rs.thetaFile
    saveName = rs.path + saveDir + "/"
    GenerateDataFromTheta(rs,0.05,saveName,thetaName,repeat,True)
    print("Regression:End of generation")

def generateRichDataFromRegression(repeat, rs,thetaFile, saveDir):
    thetaName = rs.path + thetaFile
    saveName = rs.path + saveDir + "/"        
    GenerateRichDataFromTheta(rs,0.05,saveName,thetaName,repeat,True)
    print("Regression:End of generation")

def launchCMAESForSpecificTargetSize(sizeOfTarget, rs, save):
    '''
    Run cmaes for a specific target size

    Input:	-sizeOfTarget, size of the target, float
            -setuFile, file of setup, string
            -save, for saving result, bool
    '''
    print("Starting the CMAES Optimization for target " + str(sizeOfTarget) + " !")
    foldername = rs.CMAESpath + str(sizeOfTarget) + "/"
    if save:
        thetaname = foldername + rs.thetaFile
        copyRegressiontoCMAES(rs, rs.thetaFile, sizeOfTarget)
    else:
        thetaname = foldername + "Best"

    #Initializes all the class used to generate trajectory
    exp = Experiments(rs, sizeOfTarget, False, foldername, thetaname,rs.popsizeCmaes,rs.period)
    theta = exp.tm.controller.getTheta()
    thetaCMA = theta.flatten()

    #run the optimization (cmaes)
    cma.fmin(exp.runTrajectoriesCMAES, thetaCMA, rs.sigmaCmaes, options={'maxiter':rs.maxIterCmaes, 'popsize':rs.popsizeCmaes, 'CMA_diagonal':True, 'verb_log':50, 'verb_disp':1,'termination_callback':term()})
    print("End of optimization for target " + str(sizeOfTarget) + " !")
    
def launchCMAESForAllTargetSizes(rs, save):
    for el in rs.sizeOfTarget:
        launchCMAESForSpecificTargetSize(el, rs,save)

def term():
    return False

#--------------------------- multiprocessing -------------------------------------------------------
    
def launchCMAESForAllTargetSizesMulti(rs):
    '''
    Launch in parallel (on differents processor) the cmaes optimization for each target size
    '''
    #initializes setup variables
    #initializes a pool of worker, ie multiprocessing
    p = ThreadPool(processes=4)
    #run cmaes on each targets size on separate processor
    p.map(partial(launchCMAESForSpecificTargetSize, rs=rs, save=False), rs.sizeOfTarget)
    p.close()
    p.join()