#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author: Corentin Arnaud

Module: ReadXmlFile

Description: reads a global xml setup file
'''

from lxml import etree
from os import path
import math
from Cost.CostType import CostType
from Experiments.StateEstimatorType import StateEstimatorType

class ReadXmlFile(object):
    def __init__(self, xmlFile):
        self.parse(xmlFile)


    def parse(self, xmlFile):
        tree = etree.parse(xmlFile).getroot()
        self.dataParse(tree[0])
        self.regressionParse(tree[1])
        self.costFunctionParse(tree[2])
        self.optimisationParse(tree[3])
        self.targetParse(tree[4])
        self.trajectoryParse(tree[5])
        self.kalmanParse(tree[6])
        self.plotParse(tree[7])
    
    def dataParse(self, dataElement):
        self.inputDim=int(dataElement[0].text)
        self.outputDim=int(dataElement[1].text)
        self.det="no"==dataElement[2].text
        if(not self.det and dataElement[2].text!= 'yes'):
            self.noise = float(dataElement[2].text)
        else :
            self.noise=None
        self.arm="Arm"+str(self.inputDim/2)+str(self.outputDim)
           
    def regressionParse(self, reg):
        self.regression=reg[0].tag
        if(self.regression=="NeuralNet"):
            self.neuralNetParse(reg[0])
        elif(self.regression == "RBFN"):
            self.rbfnParse(reg[0])
        self.thetaFile=reg[1].text 
        self.path=path.abspath(path.expanduser(reg[2].text))+"/"

            
    def neuralNetParse(self, reg):
        inputLayer   = reg[0]
        hiddenLayers = reg[1]
        outputLayer  = reg[2]
        
        self.bias=reg[3].text=="yes"
        
        self.learningRate =float(reg[4].text)
        self.momentum     =float(reg[5].text)

        
        self.inputLayer  = inputLayer[0] .text
        self.outputLayer = outputLayer[0].text
        self.hiddenLayers=[]
        for layer in hiddenLayers :
            self.hiddenLayers.append((layer[0].text, int(layer[1].text)))
            
    def rbfnParse(self, rbfnElement):
        self.numfeats   = int  (rbfnElement[0].text)
        self.lamb       = float(rbfnElement[1].text)
        self.fromStruct = rbfnElement[2].text == "yes"
        
    def costFunctionParse(self, cf):
        self.costClass=cf.get('type')
        if(self.costClass!=None):self.costClass=CostType[self.costClass]
        self.gammaCF=float(cf[0].text)
        self.rhoCF  =float(cf[1].text)
        self.upsCF  =float(cf[2].text)
        
    def optimisationParse(self, optiElem):
        if(optiElem.get("type")=="CMAES"):
            self.CMAESParse(optiElem)
        elif(optiElem.get("type") == "DDPG"):
            self.DDPGParse(optiElem)
        else :
            raise TypeError()
        
    def CMAESParse(self, cmaesElement):
        self.maxIterCmaes           =int  (cmaesElement[0].text)
        self.numberOfRepeatEachTraj =int  (cmaesElement[1].text)
        self.OPTIpath=path.abspath(path.expanduser(cmaesElement[2].text))
        self.sigmaCmaes             =float(cmaesElement[3].text)
        self.popsizeCmaes           =int  (cmaesElement[4].text)
        
    def DDPGParse(self, ddpgElement):
        self.maxIterDDPG         =int  (ddpgElement[0].text)
        self.numberOfRepeatEachTraj =int  (ddpgElement[1].text)
        self.OPTIpath=path.abspath(path.expanduser(ddpgElement[2].text))+"/"
        
    def targetParse(self, targetElement):
        self.sizeOfTarget = []
        for size in targetElement[0]:
            self.sizeOfTarget.append(float(size.text))
        self.XTarget = float(targetElement[1][0].text)
        self.YTarget = float(targetElement[1][1].text)
        
    def trajectoryParse(self, trajectoryElement):
        self.experimentFilePosIni=trajectoryElement[0].text
        self.maxSteps=int(trajectoryElement[1].text)
        self.dt=float(trajectoryElement[2].text)
        
    def kalmanParse(self, kalmanElement):
        self.estimationType=kalmanElement.get("type")
        if(self.estimationType==None):
            self.estimationType=StateEstimatorType['Inv']
        else :
            self.estimationType=StateEstimatorType[self.estimationType]
        self.delayUKF=int(kalmanElement[0].text)
        
    def plotParse(self, plotElement):
        self.period=float(plotElement[0].text)
      
    def getDistanceToTarget(self, x, y):
        return math.sqrt((x - self.XTarget)**2 + (y - self.YTarget)**2) 
    
