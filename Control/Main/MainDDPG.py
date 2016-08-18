#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: MainDDPG

Description: useful functions to run ddpg
'''

from Utils.Chrono import Chrono
from MainCMAES import GenerateDataFromTheta, GenerateRichDataFromTheta
from Optimizer.DDPGEnv import DDPGEnv
from DDPG.core.DDPG_core import DDPG
from DDPG.core.networks.simple_actor_network import simple_actor_network
from Utils.FileWriting import writeArray, checkIfFolderExists

from shutil import copyfile
import numpy as np
import os
from multiprocess.pool import ThreadPool
from functools import partial
from GlobalVariables import pathDataFolder
import tensorflow as tf
#TODO: change CMAESpath name


def copyRegressiontoCMAES(rs, name, size):
    cmaname =  rs.CMAESpath + str(size) + "/"
    checkIfFolderExists(cmaname)
    savenametheta = rs.path + name + ".theta"
    copyfile(savenametheta, cmaname + name + ".theta")
    
        
        

def generateFromDDPG(repeat, rs, thetaFile, saveDir = 'Data'):
    for el in rs.sizeOfTarget:
        c = Chrono()
        actor=simple_actor_network(rs.inputDim, rs.outputDim, l1_size = rs.hiddenLayers[0][1], l2_size = rs.hiddenLayers[1][1], learning_rate = rs.learningRate)
        env = DDPGEnv(rs, el, rs.thetaFile, actor = actor,saveDir=saveDir)
        thetaName = rs.OPTIpath + str(el) + "/" + thetaFile + ".theta"
        saveName = rs.OPTIpath + str(el) + "/" + saveDir + "/"
        os.system("rm "+saveName+"Log/*.log 2>/dev/null")
        parameters=np.loadtxt(thetaName)
        actor.load_parameters(parameters)
        cost, time = env.saveAllTraj(repeat)
        c.stop()
        print("Average cost: ", cost)
        print("Average time: ", time)
        print("foldername : ", saveName)
    print("DDPG:End of generation")
    
    
def generateRichDataFromDDPG(repeat, rs, thetaFile, saveDir = 'Data'):
    for el in rs.sizeOfTarget:
        thetaName = rs.OPTIpath + str(el) + "/" + thetaFile
        saveName = rs.OPTIpath + str(el) + "/" + saveDir + "/"
        GenerateRichDataFromTheta(rs,el,saveName,thetaName,repeat,True)
    print("DDPG:End of generation")

# TODO: that
def generateFromDDPGNController(repeat, rs, thetaFile, saveDir = 'Data'):
    for el in rs.sizeOfTarget:
        c = Chrono()
        thetaName = rs.OPTIpath + str(el)+"/*/"+thetaFile
        saveName = rs.OPTIpath + str(el) + "/" + saveDir + "/"
        #GenerateDataFromThetaNController(rs,el, saveName,thetaName,repeat,True)
        c.stop()
    print("DDPG:End of generation")   

    
def launchDDPGForSpecificTargetSize(sizeOfTarget, rs):
    actor=simple_actor_network(rs.inputDim, rs.outputDim, l1_size = rs.hiddenLayers[0][1], l2_size = rs.hiddenLayers[1][1], learning_rate = rs.learningRate)
    env = DDPGEnv(rs, sizeOfTarget, rs.thetaFile, actor=actor)
    ddpg = DDPG(env, actor = actor)
    ddpg.M_episodes(rs.maxIterDDPG, train=False)
    
    
def launchDDPGForSpecificTargetSizeAndSpeceficBeginning(sizeOfTarget, rs, point):
    '''
    Run cmaes for a specific target sizeCMAES

    Input:    -sizeOfTarget, size of the target, float
            -setuFile, file of setup, string
            -save, for saving result, bool
    '''
    pos=point[0]
    x=point[1][0]
    y=point[1][1]
    print("Starting the DDPGor target " + str(sizeOfTarget) + " for point "+ str(pos)+" !")
    foldername = rs.OPTIpath + str(sizeOfTarget)+"/"+str(pos)+"/"
    



    actor=simple_actor_network(rs.inputDim, rs.outputDim, l1_size = rs.hiddenLayers[0][1], l2_size = rs.hiddenLayers[1][1], learning_rate = rs.learningRate)
    env = DDPGEnv(rs, sizeOfTarget, "Best.theta", actor=actor, saveDir=foldername)
    env.setOnePointController(x,y)
    ddpg = DDPG(env, actor = actor)
    ddpg.M_episodes(rs.maxIterDDPG, train=False)

    print("End of optimization for target " + str(sizeOfTarget) +  " for point "+ str(pos)+" !")
    
def launchDDPGForAllPoint(rs, sizeTarget):
    """
    p = ThreadPool(processes=15)
    #run cmaes on each targets size on separate processor
    posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
    p.map(partial(launchDDPGForSpecificTargetSizeAndSpeceficBeginning, sizeTarget, rs), enumerate(posIni))
    p.close()
    p.join()
    """
    coord=tf.train.Coordinator()
    posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
    # Create  threads t
    threads = [tf.train.threading.Thread(target=launchDDPGForSpecificTargetSizeAndSpeceficBeginning, args=(sizeTarget, rs, point)) for point in enumerate(posIni)]

    # Start the threads and wait for all of them to stop.
    for t in threads: t.start()
    coord.join(threads)

    
