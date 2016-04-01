#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Olivier Sigaud

Module: TrajMaker

Description: Class to generate a trajectory
'''
import numpy as np


from Utils.CreateVectorUtil import createVector
from ArmModel.Arm26 import getDotQAndQFromStateVector
from ArmModel.Arm26 import Arm26
from ArmModel.MuscularActivation import getNoisyCommand, muscleFilter
from Utils.FileWritting import findDataFilename

from Regression.RunRegression import initController

from StateEstimator import StateEstimator
from StateEstimatorRegression import StateEstimatorRegression

from StateEstimatorHyb import StateEstimatorHyb




class TrajMaker:
    
    def __init__(self, rs, sizeOfTarget, saveTraj, thetaFile, estim="Inv"):
        '''
    	Initializes the parameters used to run the functions below
    
    	Inputs:		
    			-arm, armModel, class object
                        -rs, readSetup, class object
    			-sizeOfTarget, size of the target, float
    			-Ukf, unscented kalman filter, class object
    			-saveTraj, Boolean: true = Data are saved, false = data are not saved
    	'''
        self.arm = Arm26()
        self.arm.setDT(rs.dt)

        self.controller = initController(rs,thetaFile)

        #put theta to a one dimension numpy array, ie row vector form
        #theta = matrixToVector(theta)
 
        self.rs = rs
        self.sizeOfTarget = sizeOfTarget
        #6 is the dimension of the state for the filter, 4 is the dimension of the observation for the filter, 25 is the delay used
        if estim=="Inv" :
            self.stateEstimator = StateEstimator(rs.inputDim,rs.outputDim, rs.delayUKF, self.arm)
        elif estim=="Reg":
            self.stateEstimator = StateEstimatorRegression(rs.inputDim,rs.outputDim, rs.delayUKF, self.arm)
        else :
            self.stateEstimator = StateEstimatorHyb(rs.inputDim,rs.outputDim, rs.delayUKF, self.arm)
        self.saveTraj = saveTraj
        #Initializes variables used to save trajectory
 
    def setTheta(self, theta):
        self.controller.setTheta(theta)

    def computeManipulabilityCost(self):
        '''
        Computes the manipulability cost on one step of the trajectory
		
        Input:	-cost: cost at time t, float
				
        Output:		-cost: cost at time t+1, float
        '''
        dotq, q = getDotQAndQFromStateVector(self.arm.getState())
        manip = self.arm.directionalManipulability(q,self.cartTarget)
        return 1-manip

    def computeStateTransitionCost(self, U):
        '''
		Computes the cost on one step of the trajectory
		
		Input:	-cost: cost at time t, float
				-U: muscular activation vector, numpy array (6,1)
				-t: time, float
				
		Output:		-cost: cost at time t+1, float
		'''
        #compute the square of the norm of the muscular activation vector
        norme = np.linalg.norm(U)
        mvtCost = norme*norme
        #compute the cost following the law of the model
        #return np.exp(-t/self.rs.gammaCF)*(-self.rs.upsCF*mvtCost)
        return -self.rs.upsCF*mvtCost
    
    def computePerpendCost(self): 
        '''
        compute the Perpendicular cost for one trajectory
        
        Ouput :        -cost, the perpendicular cost
        ''' 
        dotq, q = getDotQAndQFromStateVector(self.arm.getState())
        J = self.arm.jacobian(q)
        xi = np.dot(J,dotq)
        norm=np.linalg.norm(xi)
        if(norm!=0):
            xi = xi/norm
        return 500-1000*xi[0]*xi[0]

    def computeFinalReward(self, t, coordHand):
        cost = self.computePerpendCost()
        '''
		Computes the cost on final step if the target is reached
		
		Input:		-t: time, float
					-coordHand: coordinate of the end effector, numpy array
					
		Output:		-cost: final cost , float
		'''
        #check if the Ordinate of the target is reached and give the reward if yes
        if coordHand[1] >= self.rs.YTarget:
            #print "main X:", coordHand[0]
            #check if target is reached
            if coordHand[0] >= -self.sizeOfTarget/2 and coordHand[0] <= self.sizeOfTarget/2:
                cost += np.exp(-t/self.rs.gammaCF)*self.rs.rhoCF
            else:
                cost += -500-500000*(coordHand[0]*coordHand[0])
        else:
            cost += -4000
        return cost

        
    def runTrajectory(self, x, y, foldername):
        '''
    	Generates trajectory from the initial position (x, y)
    
    	Inputs:		-x: abscissa of the initial position, float
    			-y: ordinate of the initial position, float
    
    	Output:		-cost: the cost of the trajectory, float
    	'''
        #computes the articular position q1, q2 from the initial coordinates (x, y)
        q1, q2 = self.arm.mgi(x, y)
        #creates the state vector [dotq1, dotq2, q1, q2]
        q = createVector(q1,q2)
        state = np.array([0., 0., q1, q2])
        #print("start state --------------: ",state)

        #computes the coordinates of the hand and the elbow from the position vector
        coordElbow, coordHand = self.arm.mgdFull(q)
        #assert(coordHand[0]==x and coordHand[1]==y), "Erreur de MGD" does not work because of rounding effects

        #initializes parameters for the trajectory
        i, t, cost = 0, 0, 0
        self.stateEstimator.initStore(state)
        self.arm.setState(state)
        estimState = state
        dataStore = []
        qtarget1, qtarget2 = self.arm.mgi(self.rs.XTarget, self.rs.YTarget)
        vectarget = [0.0, 0.0, qtarget1, qtarget2]

        #loop to generate next position until the target is reached 
        while coordHand[1] < self.rs.YTarget and i < self.rs.maxSteps:
            stepStore = []
            #computation of the next muscular activation vector using the controller theta
            #print ("state :",self.arm.getState())
            U = self.controller.computeOutput(estimState)

            if self.rs.det:
                Unoisy = muscleFilter(U)
            else:
                #Unoisy = getNoisyCommand(U,self.arm.getMusclesParameters().getKnoiseU())
                Unoisy = getNoisyCommand(U,self.arm.musclesP.knoiseU)
                Unoisy = muscleFilter(Unoisy)
            #computation of the arm state
            realNextState = self.arm.computeNextState(Unoisy, self.arm.getState())
 
            #computation of the approximated state
            tmpState = self.arm.getState()

            if self.rs.det:
                estimNextState = realNextState
            else:
                U = muscleFilter(U)
                estimNextState = self.stateEstimator.getEstimState(tmpState,U)
            
            #print estimNextState

            self.arm.setState(realNextState)

            #computation of the cost
            cost += self.computeStateTransitionCost(Unoisy)

            '''
            print "U =", U
            print "Unoisy =", Unoisy
            print "estimstate =", estimState
            #print "theta =", self.controller.theta
            if math.isnan(cost):
                print "NAN : U =", U
                print "NAN : Unoisy =", Unoisy
                print "NAN : estimstate =", estimState
                #print "NAN : theta =", self.controller.theta
                sys.exit()
            '''

            #get dotq and q from the state vector
            dotq, q = getDotQAndQFromStateVector(tmpState)
            coordElbow, coordHand = self.arm.mgdFull(q)
            #print ("dotq :",dotq)
            #computation of the coordinates to check if the target is reach or not
            #code to save data of the trajectory

            #Note : these structures might be much improved
            if self.saveTraj == True: 
                stepStore.append(vectarget)
                stepStore.append(estimState)
                stepStore.append(tmpState)
                stepStore.append(Unoisy)
                stepStore.append(np.array(U))
                stepStore.append(estimNextState)
                stepStore.append(realNextState)
                stepStore.append([coordElbow[0], coordElbow[1]])
                stepStore.append([coordHand[0], coordHand[1]])
                #print ("before",stepStore)
                tmpstore = np.array(stepStore).flatten()
                row = [item for sub in tmpstore for item in sub]
                #print ("store",row)
                dataStore.append(row)

            estimState = estimNextState
            i += 1
            t += self.rs.dt

        cost += self.computeFinalReward(t,coordHand)

        if self.saveTraj == True:
            filename = findDataFilename(foldername+"Log/","traj",".log")
            np.savetxt(filename,dataStore)
            '''
            if coordHand[0] >= -self.sizeOfTarget/2 and coordHand[0] <= self.sizeOfTarget/2 and coordHand[1] >= self.rs.YTarget and i<230:
                foldername = pathDataFolder + "TrajRepository/"
                name = findFilename(foldername,"Traj",".traj")
                np.savetxt(name,dataStore)
            '''

        lastX = -1000 #used to ignore dispersion when the target line is not crossed
        if coordHand[1] >= self.rs.YTarget:
            lastX = coordHand[0]
        #print "end of trajectory"
        return cost, t, lastX


    def runTrajectory2(self, x, y):
        '''
    	Generates trajectory from the initial position (x, y) use for plot trajectory wihtout save them 
    
    	Inputs:		-x: abscissa of the initial position, float
    			    -y: ordinate of the initial position, float
    
    	Output:		    -trajState: state of the  trajectory generated, np-array
                        -trajActivity: activity of the trajectory generated, np-array
                        -cost: the cost of the trajectory, float
                        -t:    time of the trajectory, float
    	'''
        #computes the articular position q1, q2 from the initial coordinates (x, y)
        q1, q2 = self.arm.mgi(x, y)
        #creates the state vector [dotq1, dotq2, q1, q2]
        q = createVector(q1,q2)
        state = np.array([0., 0., q1, q2])
        #print("start state --------------: ",state)

        #computes the coordinates of the hand and the elbow from the position vector
        coordHand = self.arm.mgdEndEffector(q)
        #assert(coordHand[0]==x and coordHand[1]==y), "Erreur de MGD" does not work because of rounding effects

        #initializes parameters for the trajectory
        i, t, cost = 0, 0, 0
        self.stateEstimator.initStore(state)
        self.arm.setState(state)
        estimState = state
        trajState = [state]
        trajActivity = []

        #loop to generate next position until the target is reached 
        while coordHand[1] < self.rs.YTarget and i < self.rs.maxSteps:
            #computation of the next muscular activation vector using the controller theta
            #print ("state :",self.arm.getState())

            U = self.controller.computeOutput(estimState)

            if self.rs.det:
                realU = muscleFilter(U)
            else:
                realU = getNoisyCommand(U,self.arm.musclesP.knoiseU)
                realU = muscleFilter(realU)


            #computation of the arm state
            realNextState = self.arm.computeNextState(realU, self.arm.getState())
 
            #computation of the approximated state
            tmpState = self.arm.getState()

            if self.rs.det:
                estimNextState = realNextState
            else:
                U = muscleFilter(U)
                estimNextState = self.stateEstimator.getEstimState(tmpState,U)
            
            #print estimNextState

            self.arm.setState(realNextState)

            #computation of the cost
            cost += self.computeStateTransitionCost(realU)

            #get dotq and q from the state vector
            _, q = getDotQAndQFromStateVector(realNextState)
            coordHand = self.arm.mgdEndEffector(q)
            #print ("dotq :",dotq)
            #computation of the coordinates to check if the target is reach or not

            trajActivity.append(realU)
            trajState.append(realNextState)
            estimState = estimNextState
            i += 1
            t += self.rs.dt

        trajActivity.append(np.zeros((4)))
        cost += self.computeFinalReward(t,coordHand)
        return np.array(trajState), np.array(trajActivity), cost, t
    
    
    def runTrajectoryOpti(self, x, y):
        '''
        Generates trajectory from the initial position (x, y) use for plot trajectory wihtout save them 
    
        Inputs:        -x: abscissa of the initial position, float
                    -y: ordinate of the initial position, float
    
        Output:
                        -cost: the cost of the trajectory, float
                        -t:    time of the trajectory, float
                        -lastX: Last X
        '''
        #computes the articular position q1, q2 from the initial coordinates (x, y)
        q1, q2 = self.arm.mgi(x, y)
        #creates the state vector [dotq1, dotq2, q1, q2]
        q = createVector(q1,q2)
        state = np.array([0., 0., q1, q2])
        #print("start state --------------: ",state)

        #computes the coordinates of the hand and the elbow from the position vector
        coordHand = self.arm.mgdEndEffector(q)
        #assert(coordHand[0]==x and coordHand[1]==y), "Erreur de MGD" does not work because of rounding effects

        #initializes parameters for the trajectory
        i, t, cost = 0, 0, 0
        self.stateEstimator.initStore(state)
        self.arm.setState(state)
        estimState = state


        #loop to generate next position until the target is reached 
        while coordHand[1] < self.rs.YTarget and i < self.rs.maxSteps:
            #computation of the next muscular activation vector using the controller theta
            #print ("state :",self.arm.getState())
            U = self.controller.computeOutput(estimState)

            if self.rs.det:
                realU = muscleFilter(U)
                #computation of the arm state
                realNextState = self.arm.computeNextState(realU, self.arm.getState())
     
                #computation of the approximated state
                tmpState = self.arm.getState()
                
                estimNextState = realNextState
            else:
                #realU = getNoisyCommand(U,self.arm.getMusclesParameters().getKnoiseU())
                realU = getNoisyCommand(U,self.arm.musclesP.knoiseU)
                realU = muscleFilter(realU)


                #computation of the arm state
                realNextState = self.arm.computeNextState(realU, self.arm.getState())
     
                #computation of the approximated state
                tmpState = self.arm.getState()
                
                U = muscleFilter(U)
                estimNextState = self.stateEstimator.getEstimState(tmpState,U)


            
            #print estimNextState

            self.arm.setState(realNextState)

            #computation of the cost
            cost += self.computeStateTransitionCost(realU)
            #get dotq and q from the state vector
            _, q = getDotQAndQFromStateVector(tmpState)
            coordHand = self.arm.mgdEndEffector(q)
            #print ("dotq :",dotq)
            #computation of the coordinates to check if the target is reach or not


            estimState = estimNextState
            i += 1
            t += self.rs.dt


        cost += self.computeFinalReward(t,coordHand)
        lastX = -1000 #used to ignore dispersion when the target line is not crossed
        if coordHand[1] >= self.rs.YTarget:
            lastX = coordHand[0]
        return cost, t, lastX
    

    
    
