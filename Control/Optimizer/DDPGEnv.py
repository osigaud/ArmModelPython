#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Created on 28 avril. 2016

@author: corentin arnaud
'''

from DDPG.environement.env import Env
from Utils.CreateVectorUtil import createVector
from ArmModel.ArmType import ArmType
from ArmModel.MuscularActivation import getNoisyCommand, muscleFilter
from Utils.FileWritting import checkIfFolderExists, findDataFilename, writeArray

from Experiments.StateEstimator import StateEstimator
from Experiments.StateEstimatorRegression import StateEstimatorRegression

from Experiments.StateEstimatorHyb import StateEstimatorHyb
from Experiments.StateEstimatorNoFeedBack import StateEstimatorNoFeedBack
from GlobalVariables import pathDataFolder
import random as rd
import numpy as np
from Experiments.CostDDPG import CostDDPG
from Experiments import CostCMAES.Cost
from multiprocess.pool import Pool
from functools import partial

class DDPGEnv(Env):
    
    print_interval = 20
    
    def __init__(self, rs, sizeOfTarget, thetafile, arm="Arm26", estim="Inv", actor=None, logDir="Best", saveDir=None):
        self.rs=rs
        self.posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
        self.arm=ArmType[arm]()
        self.arm.setDT(rs.dt)
        self.trajCost=CostDDPG(rs)
        if(len(self.posIni.shape)==1):
            self.posIni=self.posIni.reshape((1,self.posIni.shape[0]))
        if estim=="Inv" :
            self.stateEstimator = StateEstimator(rs.inputDim,rs.outputDim, rs.delayUKF, self.arm)
        elif estim=="Reg":
            self.stateEstimator = StateEstimatorRegression(rs.inputDim,rs.outputDim, rs.delayUKF, self.arm)
        elif estim == "Hyb" :
            self.stateEstimator = StateEstimatorHyb(rs.inputDim,rs.outputDim, rs.delayUKF, self.arm)
        elif estim =="No" :
            self.stateEstimator = StateEstimatorNoFeedBack(rs.inputDim,rs.outputDim, rs.delayUKF, self.arm)
        else :
            raise TypeError("This Estimator do not exist")
        
        if(saveDir==None):
            self.foldername=self.rs.OPTIpath + str(self.sizeOfTarget) + "/"
        else :
            self.foldername=saveDir
            
        self.costStore = []
        self.trajTimeStore = []
        self.lastCoord = []
        
        self.sizeOfTarget = sizeOfTarget
        self.thetafile = thetafile
        self.actor=actor
        self.saveName = self.foldername + logDir + "/"
        self.nbReset=0
        self.cost=0
        self.max=0
        self.progressTab=[0.1,0.25,0.5,0.75,1.]
        self.progress=1
        self.reset()

        
    def getActionSize(self):
        return self.rs.outputDim
        
       
    def getStateSize(self):
        return self.rs.inputDim
        
        
    def getActionBounds(self):
        return [[1]*self.rs.outputDim, [0]*self.rs.outputDim]
    
    def saveCost(self):
        writeArray(self.costStore,self.foldername+"CostCMAES/","traj",".cost")
        writeArray(self.trajTimeStore, self.foldername+"TrajTime/","traj",".time")
        writeArray(self.lastCoord, self.foldername+"finalX/","x",".last")
        
    def act(self, action):
        action=np.array(action[0])
        action = action + np.random.normal(0.,0.2,action.shape)
        return self.__act__(action)
        
    
    def __act__(self, action):
        if self.rs.det:
            realU = muscleFilter(action)
            #computation of the arm state
            realNextState = self.arm.computeNextState(realU, self.arm.getState())
 
            #computation of the approximated state
            tmpState = self.arm.getState()
                
            estimNextState = realNextState
        else:
            #realU = getNoisyCommand(U,self.arm.getMusclesParameters().getKnoiseU())
            realU = getNoisyCommand(action,self.arm.musclesP.knoiseU)
            realU = muscleFilter(realU)


            #computation of the arm state
            realNextState = self.arm.computeNextState(realU, self.arm.getState())
     
            #computation of the approximated state
            tmpState = self.arm.getState()
                
            action = muscleFilter(action)
            estimNextState = self.stateEstimator.getEstimState(tmpState,action)


            
        #print estimNextState

        self.arm.setState(realNextState)

        #computation of the cost

        #get dotq and q from the state vector
        _, q = self.arm.getDotQAndQFromStateVector(tmpState)
        self.coordHand = self.arm.mgdEndEffector(q)
        costAct = self.trajCost.computeStateTransitionCost(realU, self.coordHand)
        #print ("dotq :",dotq)
        #computation of the coordinates to check if the target is reach or not
        self.i+=1
        self.t += self.rs.dt
        self.estimState = estimNextState
        if(self.coordHand[1] >= self.rs.YTarget or self.i >= self.rs.maxSteps):
            costAct += self.trajCost.computeFinalReward(self.arm,self.t, self.coordHand, self.sizeOfTarget)
        self.cost+=costAct
        return [realU], [costAct]
    
    
    
    def actAndStore(self, action):
        if self.rs.det:
            realU = muscleFilter(action)
            #computation of the arm state
            realNextState = self.arm.computeNextState(realU, self.arm.getState())
 
            #computation of the approximated state
            tmpState = self.arm.getState()
                
            estimNextState = realNextState
        else:
            #realU = getNoisyCommand(U,self.arm.getMusclesParameters().getKnoiseU())
            realU = getNoisyCommand(action,self.arm.musclesP.knoiseU)
            realU = muscleFilter(realU)


            #computation of the arm state
            realNextState = self.arm.computeNextState(realU, self.arm.getState())
     
            #computation of the approximated state
            tmpState = self.arm.getState()
                
            action = muscleFilter(action)
            estimNextState = self.stateEstimator.getEstimState(tmpState,action)


            
        #print estimNextState

        self.arm.setState(realNextState)

        #computation of the cost

        #get dotq and q from the state vector
        _, q = self.arm.getDotQAndQFromStateVector(tmpState)
        coordElbow, self.coordHand = self.arm.mgdFull(q)
        cost = self.trajCost.computeStateTransitionCost(realU, self.coordHand)
        
        stepStore=[]
        stepStore.append(self.vectarget)
        stepStore.append(self.estimState)
        stepStore.append(tmpState)
        stepStore.append(realU)
        stepStore.append(action)
        stepStore.append(estimNextState)
        stepStore.append(realNextState)
        stepStore.append([coordElbow[0], coordElbow[1]])
        stepStore.append([self.coordHand[0], self.coordHand[1]])
        #print ("before",stepStore)
        tmpstore = np.array(stepStore).flatten()
        row = [item for sub in tmpstore for item in sub]
        #print ("store",row)
        self.dataStore.append(row)
        
        self.i+=1
        self.t += self.rs.dt
        self.estimState = estimNextState
        return [realU], [cost]
    
    def state(self):
        return [self.arm.getState()]
    
    def reset(self, noise=True):
        #print("Episode : "+str(self.nbReset)+ ", Progression :"+str(self.progress))
        #self.nbReset+=1
        #print(self.cost)
        #Discrete begining
        """
        i = rd.randint(0,len(self.posIni)-1)
        #i=0
        #computes the articular position q1, q2 from the initial coordinates (x, y)
        q1, q2 = self.arm.mgi(self.posIni[i][0], self.posIni[i][1])
        """
        #Progress
        """
        i = ((rd.random()*6*np.pi - 3*np.pi)*self.progressTab[self.progress]-6*np.pi)/12
        j= (rd.random()*0.3-0.15)*self.progressTab[self.progress]+0.25
        #i=0
        #computes the articular position q1, q2 from the initial coordinates (x, y)
        q1, q2 = self.arm.mgi(self.rs.XTarget+ j*np.cos(i), self.rs.YTarget+ j*np.sin(i))
        """


        


        """
        i =  - 6*np.pi/12
        j=rd.randint(1,4)/10.
        #j=0.1

        q1, q2 = self.arm.mgi(self.rs.XTarget+ j*np.cos(i), self.rs.YTarget+ j*np.sin(i))
        """

        x, y = self.begin()
        q1, q2 = self.arm.mgi(x,y)
        #creates the state vector [dotq1, dotq2, q1, q2]
        q = createVector(q1,q2)
        state = np.array([0., 0., q1, q2])
        #print("start state --------------: ",state)

        #computes the coordinates of the hand and the elbow from the position vector
        self.coordHand = self.arm.mgdEndEffector(q)
        #assert(coordHand[0]==x and coordHand[1]==y), "Erreur de MGD" does not work because of rounding effects

        #initializes parameters for the trajectory
        self.i, self.t, self.cost = 0, 0., 0.
        self.stateEstimator.initStore(state)
        self.arm.setState(state)
        self.estimState = state

    def begin(self):
        return self.beginDiscret()

    def beginOnePoint(self):
        return self.x, self.y
        
    def beginArcProgress(self):
        i = (rd.random()*6*np.pi - 9*np.pi)/12.
        j=rd.randint(1,self.progress)/10.
        return self.rs.XTarget+ j*np.cos(i), self.rs.YTarget+ j*np.sin(i)
    
    def beginDiscret(self):
        i = rd.randint(0,len(self.posIni)-1)
        return self.posIni[i][0], self.posIni[i][1]

    def isFinished(self):
        if(self.coordHand[1] >= self.rs.YTarget or self.i >= self.rs.maxSteps):
            return True
        return False
    
    def OneTraj(self, x, y):
        #computes the articular position q1, q2 from the initial coordinates (x, y)
        q1, q2 = self.arm.mgi(x, y)
        #creates the state vector [dotq1, dotq2, q1, q2]
        q = createVector(q1,q2)
        state = np.array([0., 0., q1, q2])
        #print("start state --------------: ",state)

        #computes the coordinates of the hand and the elbow from the position vector
        self.coordHand = self.arm.mgdEndEffector(q)
        #assert(coordHand[0]==x and coordHand[1]==y), "Erreur de MGD" does not work because of rounding effects

        #initializes parameters for the trajectory
        self.i, self.t, self.cost = 0, 0, 0
        self.stateEstimator.initStore(state)
        self.arm.setState(state)
        self.estimState = state
        
        self.vectarget=[0.0, 0.0, self.rs.XTarget, self.rs.YTarget]
        totalCost=0
        while(not self.isFinished()):
            action=self.actor.action(self.estimState)
            _,cost = self.__act__(action)
            totalCost+= cost[0]
        totalCost += self.trajCost.computeFinalReward(self.arm,self.t,self.coordHand,self.sizeOfTarget)
        return totalCost, self.t
    
    def saveOneTraj(self, x, y):
        #computes the articular position q1, q2 from the initial coordinates (x, y)
        q1, q2 = self.arm.mgi(x, y)
        #creates the state vector [dotq1, dotq2, q1, q2]
        q = createVector(q1,q2)
        state = np.array([0., 0., q1, q2])
        #print("start state --------------: ",state)

        #computes the coordinates of the hand and the elbow from the position vector
        self.coordHand = self.arm.mgdEndEffector(q)
        #assert(coordHand[0]==x and coordHand[1]==y), "Erreur de MGD" does not work because of rounding effects

        #initializes parameters for the trajectory
        self.i, self.t, self.cost = 0, 0, 0
        self.stateEstimator.initStore(state)
        self.arm.setState(state)
        self.estimState = state
        
        self.dataStore=[]
        self.vectarget=[0.0, 0.0, self.rs.XTarget, self.rs.YTarget]
        totalCost=0
        while(not self.isFinished()):
            action=self.actor.action(self.estimState)
            _,cost = self.actAndStore(action)
            totalCost+= cost[0]
        totalCost += self.trajCost.computeFinalReward(self.arm,self.t,self.coordHand,self.sizeOfTarget)
        filename = findDataFilename(self.saveName+"Log/","traj"+str(x)+"-"+str(y),".log")
        np.savetxt(filename,self.dataStore)
        
        #used to ignore dispersion when the target line is not crossed
        if self.coordHand[1] >= self.rs.YTarget:
            self.lastCoord.append(self.coordHand[0])
            
        return totalCost, self.t
    
    def nTraj(self, (x, y), repeat):
        costAll, trajTimeAll = np.zeros(repeat), np.zeros(repeat)
        for i in range(repeat):
            costAll[i], trajTimeAll[i]  = self.OneTraj(x, y) 
        meanCost = np.mean(costAll)
        meanTrajTime = np.mean(trajTimeAll)
        return meanCost, meanTrajTime
    
    def allTraj(self, repeat):
        globMeanCost=0.
        globTimeCost=0.
        for x,y in self.posIni:
            meanCost, meanTrajTime = self.nTraj((x, y), repeat)
            globMeanCost+=meanCost
            globTimeCost+=meanTrajTime
        size=len(self.posIni)
        return globMeanCost/size, globTimeCost/size
    
    def progressArcTraj(self, repeat):
        globMeanCost=0.
        globTimeCost=0.
        for j in range(self.progress):
            for i in range(7):
                x,y = self.rs.XTarget+ (0.1+j*0.1)*np.cos(-i*np.pi/12-np.pi/4), self.rs.YTarget+ (0.1+j*0.1)*np.sin(-i*np.pi/12-np.pi/4)
                meanCost, meanTrajTime = self.nTraj((x, y), repeat)
                globMeanCost+=meanCost
                globTimeCost+=meanTrajTime
        size=self.progress*7
        return globMeanCost/size, globTimeCost/size
    
    def ligneTraj(self, repeat):
        globMeanCost=0.
        globTimeCost=0.
        for j in range(4):
            x,y = self.rs.XTarget+ (0.1+j*0.1)*np.cos(-np.pi/2), self.rs.YTarget+ (0.1+j*0.1)*np.sin(-np.pi/2)
            meanCost, meanTrajTime = self.nTraj((x, y), repeat)
            globMeanCost+=meanCost
            globTimeCost+=meanTrajTime
        size=4
        return globMeanCost/size, globTimeCost/size
    
    def progress(self, repeat):
        return self.allTraj(repeat)
      
    def saveAllTraj(self, repeat):
        globMeanCost=0.
        globTimeCost=0.
        costAll, trajTimeAll = np.zeros(repeat), np.zeros(repeat)
        for x,y in self.posIni:
            for i in range(repeat):
                costAll[i], trajTimeAll[i]=self.saveOneTraj(x,y)
            meanCost = np.mean(costAll)
            meanTrajTime = np.mean(trajTimeAll)
            self.costStore.append([x, y, meanCost])
            self.trajTimeStore.append([x, y, meanTrajTime])
            globMeanCost+=meanCost
            globTimeCost+=meanTrajTime
        size=len(self.posIni)
        return globMeanCost/size, globTimeCost/size
    
    def saveAllTrajNController(self, repeat, thetaName):
        globMeanCost=0.
        globTimeCost=0.
        costAll, trajTimeAll = np.zeros(repeat), np.zeros(repeat)

        for enum,xy in enumerate(self.posIni):
            controllerFileName = thetaName.replace("*",str(enum))
            parameters=np.loadtxt(controllerFileName)
            self.actor.load_parameters(parameters)
            for i in range(repeat):
                costAll[i], trajTimeAll[i]=self.saveOneTraj(xy[0],xy[1])
            meanCost = np.mean(costAll)
            meanTrajTime = np.mean(trajTimeAll)
            self.costStore.append([xy[0], xy[1], meanCost])
            self.trajTimeStore.append([xy[0], xy[1], meanTrajTime])
            globMeanCost+=meanCost
            globTimeCost+=meanTrajTime
        size=len(self.posIni)
        return globMeanCost/size, globTimeCost/size
 
 
    def runMultiProcessTrajectories(self, repeat):
        pool=Pool(processes=len(self.posIni))
        result = pool.map(partial(self.nTraj, repeat=repeat) , [(x, y) for x, y in self.posIni])
        pool.close()
        pool.join()
        meanCost, meanTraj=0, 0
        for CostCMAES, traj in result:
            meanCost+=CostCMAES
            meanTraj+=traj
        size = len(result)
        return meanCost/size, meanTraj/size
    
            
    def draw(self):
        pass
    
    def printEpisode(self):
        #cost, time = self.runMultiProcessTrajectories(self.rs.numberOfRepeatEachTraj)
        #cost, time = self.allTraj(self.rs.numberOfRepeatEachTraj)
        #print("Progression :"+str(self.progress))
        cost, time = self.progress(self.rs.numberOfRepeatEachTraj)
        #cost, time = self.ligneTraj(self.rs.numberOfRepeatEachTraj)
        #cost, time = self.OneTraj(self.rs.XTarget+ 0.1*np.cos(-np.pi/2), self.rs.YTarget+ 0.1*np.sin(-np.pi/2))
        
        costfoldername = self.foldername+"CostCMAES/"
        checkIfFolderExists(costfoldername)
        costFile = open(costfoldername+"ddpgCost.log","a")
        timeFile = open(costfoldername+"ddpgTime.log","a")
        costFile.write(str(cost)+"\n")
        timeFile.write(str(time)+"\n")
        costFile.close()
        timeFile.close()
        """
        if(cost>0.8 and self.progress <4):
            self.progress+=1
            self.max=0
        """
        if(cost>self.max):
            self.max = cost
            writeArray(self.actor.linear_parameters(),self.foldername, "Best", ".theta")
        #TODO: versatil version
        #print("CostCMAES : "+str(cost)+" time : "+str(time))
            
    
    def setOnePointController(self,x,y):
        self.x=x
        self.y=y
        self.begin = self.beginOnePoint
        self.progress=partial(self.nTraj,(self.x,self.y))