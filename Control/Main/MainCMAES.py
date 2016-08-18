#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud + Olivier Sigaud

Module: Main

Description: useful functions to run cmaes and some scripts to run trajectories
'''
#TODO: Make a optimisation and CMAES class
import os
import cma
from shutil import copyfile
from functools import partial
from multiprocess.pool import ThreadPool
import numpy as np
from GlobalVariables import pathDataFolder
from Utils.Chrono import Chrono

from Experiments.Experiments import Experiments
from Utils.FileWriting import checkIfFolderExists

def copyRegressiontoCMAES(rs, name, size):
    cmaname =  rs.OPTIpath + str(size) + "/"
    checkIfFolderExists(cmaname)
    savenametheta = rs.path + name + ".theta"
    copyfile(savenametheta, cmaname + name + ".theta")
    
    if(rs.regression=="RBFN"):
        savenamestruct = rs.path + name + ".struct"
        copyfile(savenamestruct, cmaname + name + ".struct")

def GenerateDataFromTheta(rs, sizeOfTarget, foldername, thetaFile, repeat, save):
    '''
    Generate Data from a given theta file

    Input:    -sizeOfTarget, size of the target, float
            -save: do we save the results (false when running CMAES)? True = Yes, False = No
    '''
    os.system("rm "+foldername+"Log/*.log 2>/dev/null")
    exp = Experiments(rs, sizeOfTarget, save, foldername,thetaFile,rs.popsizeCmaes,rs.period)
    cost, time = exp.runTrajectoriesForResultsGeneration(repeat)
    print("Average cost: ", cost)
    print("Average time: ", time)
    print("foldername : ", foldername)
    if (save):
        exp.saveCost()
        
def GenerateDataFromThetaNController(rs, sizeOfTarget, foldername, thetaFile, repeat, save,noise=None):
    '''
    Generate Data from a given theta file

    Input:    -sizeOfTarget, size of the target, float
            -save: do we save the results (false when running CMAES)? True = Yes, False = No
    '''
    os.system("rm "+foldername+"/Log/*.log 2>/dev/null")
    exp = Experiments(rs, sizeOfTarget, save, foldername,None,rs.popsizeCmaes,rs.period)
    if(noise!=None): exp.setNoise(noise)
    cost, time = exp.runTrajectoriesForResultsGenerationNController(repeat,thetaFile)
    print("Average cost: ", cost)
    print("Average time: ", time)
    print("foldername : ", foldername)
    if (save):
        exp.saveCost()
        
def GenerateDataFromThetaOnePoint(rs, sizeOfTarget, foldername, thetaFile, repeat, point):
    '''
    Generate Data from a given theta file

    Input:    -sizeOfTarget, size of the target, float
            -save: do we save the results (false when running CMAES)? True = Yes, False = No
    '''
    os.system("rm "+foldername+"/Log/*.log 2>/dev/null")
    exp = Experiments(rs, sizeOfTarget, True, foldername,thetaFile,rs.popsizeCmaes,rs.period)
    cost, time = exp.runTrajectoriesForResultsGenerationOnePoint(repeat,point)
    print("Average cost: ", cost)
    print("Average time: ", time)
    print("foldername : ", foldername)


def GenerateRichDataFromTheta(rs, sizeOfTarget, foldername, thetaFile, repeat, save):
    '''
    Generate Data from a given theta file

    Input:    -sizeOfTarget, size of the target, float
            -save: do we save the results (false when running CMAES)? True = Yes, False = No
    '''
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
        thetaName = rs.OPTIpath + str(el) + "/" + thetaFile
        saveName = rs.OPTIpath + str(el) + "/" + saveDir + "/"
        GenerateDataFromTheta(rs,el,saveName,thetaName,repeat,True)
        c.stop()
    print("CMAES:End of generation")
    
def generateFromCMAESNController(repeat, rs, thetaFile, saveDir = 'Data', noise=None):
    for el in rs.sizeOfTarget:
        c = Chrono()
        thetaName = rs.OPTIpath + str(el)+"/*/"+thetaFile
        saveName = rs.OPTIpath + str(el) + "/" + saveDir + "/"
        GenerateDataFromThetaNController(rs,el, saveName,thetaName,repeat,True,noise)
        c.stop()
    print("CMAES:End of generation")
    
def generateFromCMAESonePoint(repeat, rs, thetaFile, saveDir = 'Data', size=0.04, point=0):
    c = Chrono()
    thetaName = rs.OPTIpath + str(size)+"/"+str(point)+"/"+thetaFile
    saveName = rs.OPTIpath + str(size) + "/" + saveDir + "/"
    GenerateDataFromThetaOnePoint(rs,size, saveName,thetaName,repeat, point)
    c.stop()
    print("CMAES:End of generation")

def generateRichDataFromCMAES(repeat, rs, thetaFile, saveDir = 'Data'):
    for el in rs.sizeOfTarget:
        thetaName = rs.OPTIpath + str(el) + "/" + thetaFile
        saveName = rs.OPTIpath + str(el) + "/" + saveDir + "/"
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

    Input:	
            -sizeOfTarget, size of the target, float
            -setuFile, file of setup, string
            -save: do we use a previous Best.theta file? True = Yes, False = use current controller, None = random controller
    '''
    print("Starting the CMAES Optimization for target " + str(sizeOfTarget) + " !")
    foldername = rs.OPTIpath + str(sizeOfTarget) + "/"
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
    
def launchCMAESForSpecificTargetSizeAndSpecificPoint(sizeOfTarget, rs, save, point, noise=None):
    '''
    Run cmaes for a specific target size

    Input:    -sizeOfTarget, size of the target, float
            -setuFile, file of setup, string
            -save: do we use a previous Best.theta file? True = Yes, False = use current controller, None = random controller
            noise: noise on muscle, if None, defalt noise from muscle setup, float
    '''
    pos=point[0]
    x=point[1][0]
    y=point[1][1]
    print("Starting the CMAES Optimization for target " + str(sizeOfTarget) + " for point "+ str(pos)+" !")
    foldername = rs.OPTIpath + str(sizeOfTarget)+"/"+str(pos)+"/"
    

    thetaname = foldername + "Best"
    if save:
        checkIfFolderExists(foldername)
        copyfile(rs.OPTIpath + str(sizeOfTarget)+"/" + "Best.theta",foldername + "Best.theta")
    elif save==None:
        thetaname=None




    #Initializes all the class used to generate trajectory
    exp = Experiments(rs, sizeOfTarget, False, foldername, thetaname,rs.popsizeCmaes,rs.period)
    if(noise!=None):
        exp.setNoise(noise)
    theta = exp.tm.controller.getTheta()
    thetaCMA = theta.flatten()

    #run the optimization (cmaes)
    cma.fmin(partial(exp.runTrajectoriesCMAESOnePoint, x, y), thetaCMA, rs.sigmaCmaes, options={'maxiter':rs.maxIterCmaes, 'popsize':rs.popsizeCmaes, 'CMA_diagonal':True, 'verb_log':0, 'verb_disp':0,'termination_callback':term()})
    print("End of optimization for target " + str(sizeOfTarget) +  " for point "+ str(pos)+" !")
    
    
def launchCMAESForAllTargetSizes(rs, save):
    for el in rs.sizeOfTarget:
        launchCMAESForSpecificTargetSize(el, rs,save)

def term():
    return False

def checkAllPoint(rs, sizeTarget):
    posIni= np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
    print("check CMAES progression for target " + str(sizeTarget) + " : ")
    print("Point    |    max    |    gap between max and min ")
    maxR = np.zeros(posIni.shape[0])
    gap= np.zeros(posIni.shape[0])
    for i in range(posIni.shape[0]):
        name = rs.OPTIpath + str(sizeTarget)+"/"+str(i)+ "/Cost/cmaesCost.log"
        data = np.loadtxt(name)
        maxR[i]=data[:,-1].max()
        gap[i]=data[-1][-1]-data[-1][0]
        print(str(i)+"        "+str(maxR[i])+"        "+str(gap[i]))

    


        
        

#--------------------------- multiprocessing -------------------------------------------------------

def lauchCMAESForListOfPoints(sizeOfTarget, rs, save, points):
    p = ThreadPool(processes=len(points))
    posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
    p.map(partial(launchCMAESForSpecificTargetSizeAndSpecificPointMulti, sizeOfTarget, rs, save), [[i, posIni[i]] for i in points])
    p.close()
    p.join()
    
    
def launchCMAESForSpecificTargetSizeAndSpecificPointMulti(sizeOfTarget, rs, save, point):
    '''
    Run cmaes for a specific target size

    Input:    -sizeOfTarget, size of the target, float
            -setuFile, file of setup, string
            -save: do we use a previous Best.theta file? True = Yes, False = use current controller, None = random controller
    '''
    pos=point[0]
    x=point[1][0]
    y=point[1][1]
    print("Starting the CMAES Optimization for target " + str(sizeOfTarget) + " for point "+ str(pos)+" !")
    foldername = rs.OPTIpath + str(sizeOfTarget)+"/"+str(pos)+"/"
    

    thetaname = foldername + "Best"
    if save:
        checkIfFolderExists(foldername)
        copyfile(rs.OPTIpath + str(sizeOfTarget)+"/" + "Best.theta",foldername + "Best.theta")
    elif save==None:
        thetaname=None




    #Initializes all the class used to generate trajectory
    exp = Experiments(rs, sizeOfTarget, False, foldername, thetaname,rs.popsizeCmaes,rs.period)
    theta = exp.tm.controller.getTheta()
    thetaCMA = theta.flatten()

    #run the optimization (cmaes)
    cma.fmin(partial(exp.runTrajectoriesCMAESOnePointMulti, x, y), thetaCMA, rs.sigmaCmaes, options={'maxiter':rs.maxIterCmaes, 'popsize':rs.popsizeCmaes, 'CMA_diagonal':True, 'verb_log':0, 'verb_disp':0,'termination_callback':term()})
    print("End of optimization for target " + str(sizeOfTarget) +  " for point "+ str(pos)+" !")


def launchCMAESForAllPoint(rs, sizeTarget, save, noise=None):
    """
        Launch in parallel (on differents processor) the cmaes optimization for each point
        input:
                    rs: setup file
                    sizeTarget: size of the target
                    save: for save experience log
                    noise: noise on muscle, if None, defalt noise from muscle setup
    
    """
    p = ThreadPool(processes=15)
    #run cmaes on each targets size on separate processor
    posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
    p.map(partial(launchCMAESForSpecificTargetSizeAndSpecificPoint, sizeTarget, rs, save, noise=noise), enumerate(posIni))
    p.close()
    p.join()
    
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
