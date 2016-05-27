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
from Utils.FileWritting import writeArray 
from Utils.FileWritting import checkIfFolderExists
from shutil import copyfile
import numpy as np
import os
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
    

    
def launchDDPGForSpecificTargetSize(sizeOfTarget, rs):
    actor=simple_actor_network(rs.inputDim, rs.outputDim, l1_size = rs.hiddenLayers[0][1], l2_size = rs.hiddenLayers[1][1], learning_rate = rs.learningRate)
    env = DDPGEnv(rs, sizeOfTarget, rs.thetaFile, actor=actor)
    ddpg = DDPG(env, actor = actor)
    ddpg.M_episodes(rs.maxIterDDPG, train=False)
    