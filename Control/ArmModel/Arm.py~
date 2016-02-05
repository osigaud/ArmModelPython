#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: Arm

Description:    
-Models an arm with two joints and six muscles
-Computes its dynamics
'''

import numpy as np
import math

from ArmModel.ArmParameters import ArmParameters
from ArmModel.MusclesParameters import MusclesParameters

def getDotQAndQFromStateVector(state):
      '''
      Returns dotq and q from the state vector state
    
      Input:    -state: numpy array, state vector
    
      Outputs:    -dotq: numpy array
      -q: numpy array
      '''
      dotq = np.array([state[0], state[1]])
      q = np.array([state[2], state[3]])
      return dotq, q

#-----------------------------------------------------------------------------

class Arm:

  def __init__(self):
      self.__dotq0 = np.array([0.,0.])
      self.armP = ArmParameters()
      self.musclesP = MusclesParameters()
      
  def setState(self, state):
      self.state = state
      
  def getState(self):
      return self.state
      
  def setDT(self, dt):
      self.dt = dt
      
  def get_dotq_0(self):
      return np.array(self.__dotq0)

  def set_dotq_0(self, value):
      self.__dotq0 = value

  def computeNextState(self, U, state):
    '''
    Computes the next state resulting from the direct dynamic model of the arm given the muscles activation vector U
    
    Inputs:     -U: (6,1) numpy array
    -state: (4,1) numpy array (used for Kalman, not based on the current system state)

    Output:    -state: (4,1) numpy array, the resulting state
    '''
    #print ("state:", state)
    dotq, q = getDotQAndQFromStateVector(state)
    #print ("U :",U)
    #print ("dotq:",dotq)
    M = np.array([[self.armP.k1+2*self.armP.k2*math.cos(q[1]),self.armP.k3+self.armP.k2*math.cos(q[1])],[self.armP.k3+self.armP.k2*math.cos(q[1]),self.armP.k3]])
    #print ("M:",M)
    Minv = np.linalg.inv(M)
    #print ("Minv:",Minv)
    C = np.array([-dotq[1]*(2*dotq[0]+dotq[1])*self.armP.k2*math.sin(q[1]),(dotq[0]**2)*self.armP.k2*math.sin(q[1])])
    #print ("C:",C)
    #the commented version uses a non null stiffness for the muscles
    #beware of dot product Kraid times q: q may not be the correct vector/matrix
    #Gamma = np.dot((np.dot(armP.At, musclesP.fmax)-np.dot(musclesP.Kraid, q)), U)
    #Gamma = np.dot((np.dot(self.armP.At, self.musclesP.fmax)-np.dot(self.musclesP.Knulle, Q)), U)
    #above Knulle is null, so it can be simplified

    Gamma = np.dot(np.dot(self.armP.At, self.musclesP.fmax), U)
    #print ("Gamma:",Gamma)

    #Gamma = np.dot(armP.At, np.dot(musclesP.fmax,U))
    #computes the acceleration ddotq and integrates
    
    b = np.dot(self.armP.B, dotq)
    #print ("b:",b)

    ddotq = np.dot(Minv,Gamma - C - b)
    #print ("ddotq",ddotq)

    dotq += ddotq*self.dt
    q += dotq*self.dt
    #save the real state to compute the state at the next step with the real previous state
    q = self.jointStop(q)
    nextState = np.array([dotq[0], dotq[1], q[0], q[1]])
    return nextState

  def jointStop(self,q):
      '''
      Articular stop for the human arm
      The stops are included in the arm parameters file
      Shoulder: -0.6 <= q1 <= 2.6
      Elbow: -0.2 <= q2 <= 3.0
    
      Inputs:    -q: (2,1) numpy array
    
      Outputs:    -q: (2,1) numpy array
      '''
      if q[0] < self.armP.slb:
        q[0] = self.armP.slb
      elif q[0] > self.armP.sub:
        q[0] = self.armP.sub
      if q[1] < self.armP.elb:
        q[1] = self.armP.elb
      elif q[1] > self.armP.eub:
        q[1] = self.armP.eub
      return q
    
  def mgdFull(self, q):
      '''
      Direct geometric model of the arm
    
      Inputs:     -q: (2,1) numpy array, the joint coordinates
    
      Outputs:
      -coordElbow: elbow coordinate
      -coordHand: hand coordinate
      '''
      coordElbow = [self.armP.l1*np.cos(q[0]), self.armP.l1*np.sin(q[0])]
      coordHand = [self.armP.l1*np.cos(q[0])+self.armP.l2*np.cos(q[0] + q[1]), self.armP.l1*np.sin(q[0]) + self.armP.l2*np.sin(q[0] + q[1])]
      return coordElbow, coordHand
    
  def jacobian(self, q):
        J = np.array([
                    [-self.armP.l1*np.sin(q[0]) - self.armP.l2*np.sin(q[0] + q[1]),
                     -self.armP.l2*np.sin(q[0] + q[1])],
                    [self.armP.l1*np.cos(q[0]) + self.armP.l2*np.cos(q[0] + q[1]),
                     self.armP.l2*np.cos(q[0] + q[1])]])
        return J


  def estimError(self,state, estimState):
        '''
        Computes a state estimation error as the cartesian distance between the estimated state and the current state
        '''
        qdot,q = getDotQAndQFromStateVector(state)
        qdotEstim,qEstim = getDotQAndQFromStateVector(estimState)
        hand = self.mgdEndEffector(q)
        handEstim = self.mgdEndEffector(qEstim)
        dx = hand[0] - handEstim[0]
        dy = hand[1] - handEstim[1]
        return math.sqrt(dx**2 + dy**2)

  def estimErrorReduced(self,q,qEstim):
        '''
        Computes a state estimation error as the cartesian distance between the estimated state and the current state
        but only taking the position part of the state as input
        '''
        hand = self.mgdEndEffector(q)
        handEstim = self.mgdEndEffector(qEstim)
        dx = hand[0] - handEstim[0]
        dy = hand[1] - handEstim[1]
        return math.sqrt(dx**2 + dy**2)

  def cartesianSpeed(self,state):
        qdot,q = getDotQAndQFromStateVector(state)
        J = self.jacobian(q)
        return np.linalg.norm(np.dot(J,qdot))

  def mgdEndEffector(self, q):
      '''
      Direct geometric model of the arm
    
      Inputs:     -q: (2,1) numpy array, the joint coordinates
    
      Outputs:
      -coordHand: hand coordinate
      '''
      coordHand = [self.armP.l1*np.cos(q[0])+self.armP.l2*np.cos(q[0] + q[1]), self.armP.l1*np.sin(q[0]) + self.armP.l2*np.sin(q[0] + q[1])]
      return coordHand

  def mgi(self, xi, yi):
      '''
      Inverse geometric model of the arm
    
      Inputs:     -xi: abscissa of the end-effector point
              -yi: ordinate of the end-effectior point

      Outputs:
              -q1: arm angle
              -q2: foreArm angle
      '''
      a = ((xi**2)+(yi**2)-(self.armP.l1**2)-(self.armP.l2**2))/(2*self.armP.l1*self.armP.l2)
      try:
        q2 = math.acos(a)
        c = self.armP.l1 + self.armP.l2*(math.cos(q2))
        d = self.armP.l2*(math.sin(q2))
        q1 = math.atan2(yi,xi) - math.atan2(d,c)
        return q1, q2
      except ValueError:
        print("forbidden value")
        print xi,yi
        return "None"    

  def directionalManipulability(self, q, target):
       J = self.jacobian(q)
       #print "J", J
       K = np.transpose(J)
       #print "K", K
       M = np.dot(J,K)
       Minv= np.linalg.inv(M)

       coordHand = self.mgdEndEffector(q)

       vdir =  np.array([target[0]-coordHand[0],target[1]-coordHand[1]])
       vdir = vdir/np.linalg.norm(vdir)
       vdirt = np.transpose(vdir)

       root = np.dot(vdirt,np.dot(Minv,vdir))
       
       manip = 1/math.sqrt(root)
       return manip

  def manipulability(self, q, target):
       J = self.jacobian(q)
       K = np.transpose(J)
       M = np.dot(J,K)
       det = np.linalg.det(M)

       return math.sqrt(det)


    








