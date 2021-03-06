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
      dotq = np.array([state[0], state[1], state[2]])
      q = np.array([state[3], state[4], state[5]])
      return dotq, q

#-----------------------------------------------------------------------------

class Arm:

  def __init__(self):
      self.__dotq0 = np.array([0.,0.,0.])
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
    
    Inputs:     -U: (8,1) numpy array
    -state: (6,1) numpy array (used for Kalman, not based on the current system state)

    Output:    -state: (6,1) numpy array, the resulting state
    '''
    #print ("state:", state)
    dotq, q = getDotQAndQFromStateVector(state)
    #print ("U :",U)
    #print ("dotq:",dotq)
    M = np.array([[self.armP.i1+self.armP.m1*(self.armP.s1**2)+self.armP.i2+self.armP.m2*(self.armP.s2**2)+self.armP.i3+self.armP.m3*(self.armP.s3**2)+self.armP.m2*(self.armP.l1**2)+2*self.armP.m2*self.armP.l1*self.armP.s2*math.cos(q[1])+self.armP.m3*(self.armP.l1**2)+self.armP.m3*(self.armP.l2**2)+2*self.armP.m3*self.armP.l1*self.armP.l2*math.cos(q[1])+2*self.armP.m3*self.armP.l1*self.armP.s3*math.cos(q[1]+q[2])+2*self.armP.m3*self.armP.l2*self.armP.s3*math.cos(q[2]),self.armP.i2+self.armP.m2*(self.armP.s2**2)+self.armP.i3+self.armP.m3*(self.armP.s3**2)+self.armP.m2*self.armP.s2*self.armP.l1*math.cos(q[1])+self.armP.m3*self.armP.l1*self.armP.l2*math.cos(q[1])+self.armP.m3*self.armP.l1*self.armP.s3*math.cos(q[1]+q[2])+2*self.armP.m3*self.armP.l2*self.armP.s3*math.cos(q[2]),self.armP.i3+self.armP.m3*(self.armP.s3**2)+self.armP.m3*self.armP.l1*self.armP.s3*math.cos(q[1]+q[2])+self.armP.m3*self.armP.l2*self.armP.s3*math.cos(q[2])],[self.armP.i2+self.armP.m2*(self.armP.s2**2)+self.armP.i3+self.armP.m3*(self.armP.s3**2)+self.armP.m2*self.armP.s2*self.armP.l1*math.cos(q[1])+self.armP.m3*self.armP.l1*self.armP.l2*math.cos(q[1])+self.armP.m3*self.armP.l1*self.armP.s3*math.cos(q[1]+q[2])+2*self.armP.m3*self.armP.l2*self.armP.s3*math.cos(q[2]),self.armP.i2+self.armP.m2*(self.armP.s2**2)+self.armP.i3+self.armP.m3*(self.armP.s3**2)+self.armP.m3*(self.armP.l2**2)+2*self.armP.m3*self.armP.l2*self.armP.s3*math.cos(q[2]),self.armP.i3+self.armP.m3*(self.armP.s3**2)+self.armP.m3*self.armP.l2*self.armP.s3*math.cos(q[2])],[self.armP.i3+self.armP.m3*(self.armP.s3**2)+self.armP.m3*self.armP.l1*self.armP.s3*math.cos(q[1]+q[2])+self.armP.m3*self.armP.l2*self.armP.s3*math.cos(q[2]),self.armP.i3+self.armP.m3*(self.armP.s3**2)+self.armP.m3*self.armP.l2*self.armP.s3*math.cos(q[2]),self.armP.i3+self.armP.m3*(self.armP.s3**2)]])
    #print ("M:",M)
    Minv = np.linalg.inv(M)
    #print ("Minv:",Minv)
    C = np.array([-1*(2*self.armP.m2*self.armP.l1*self.armP.s2*dotq[0]*dotq[1]*math.sin(q[1])+2*self.armP.m3*self.armP.l1*self.armP.l2*dotq[0]*dotq[1]*math.sin(q[1])+2*dotq[0]*(dotq[1]+dotq[2])*self.armP.m3*self.armP.l1*self.armP.s3*math.sin(q[1]+q[2])+2*self.armP.m3*self.armP.l2*self.armP.s3*dotq[0]*dotq[2]*math.sin(q[2])+self.armP.m2*self.armP.s2*self.armP.l1*(dotq[1]**2)*math.sin(q[1])+self.armP.m3*self.armP.l1*self.armP.l2*(dotq[1]**2)*math.sin(q[1])+self.armP.m3*self.armP.l1*self.armP.s3*dotq[1]*(dotq[1]+dotq[2])+2*self.armP.m3*self.armP.l2*self.armP.s3*dotq[1]*dotq[2]*math.sin(q[2])+self.armP.m3*self.armP.l1*self.armP.s3*dotq[2]*(dotq[1]+dotq[2])*math.sin(q[1]+q[2])+self.armP.m3*self.armP.l2*self.armP.s3*(dotq[2]**2)*math.sin(q[2])),
                  -1*(self.armP.m2*self.armP.s2*self.armP.l1*dotq[0]*dotq[1]*math.sin(q[1])+self.armP.m3*self.armP.l1*self.armP.l2*dotq[0]*dotq[1]*math.sin(q[1])+self.armP.m3*self.armP.l1*self.armP.s3*dotq[0]*(dotq[1]+dotq[2])*math.sin(q[1]+q[2])+2*self.armP.m3*self.armP.l2*self.armP.s3*dotq[0]*dotq[2]*math.sin(q[2])+2*self.armP.m3*self.armP.l2*self.armP.s3*dotq[1]*dotq[2]*math.sin(q[2])+self.armP.m3*self.armP.l2*self.armP.s3*(dotq[2]**2)*math.sin(q[2]))
                   +(self.armP.m2*self.armP.l1*self.armP.s2*math.sin(q[1])+self.armP.m3*self.armP.l1*self.armP.l2*math.sin(q[1])+self.armP.m3*self.armP.l1*self.armP.s3*math.sin(q[1]+q[2]))*(dotq[0]**2)
                   +(self.armP.m2*self.armP.s2*self.armP.l1*math.sin(q[1])+self.armP.m3*self.armP.l1*self.armP.l2*math.sin(q[1])+self.armP.m3*self.armP.l1*self.armP.s3*math.sin(q[1]+q[2]))
                  *dotq[0]*dotq[1]+self.armP.m3*self.armP.l1*self.armP.s3*math.sin(q[1]+q[2])*dotq[0]*dotq[2],
                 -1*(self.armP.m3*self.armP.l1*self.armP.s3*dotq[0]*(dotq[1]+dotq[2])*math.sin(q[1]+q[2])+self.armP.m3*self.armP.l2*self.armP.s3*dotq[0]*dotq[2]*math.sin(q[2])+self.armP.m3*self.armP.l2*self.armP.s3*dotq[1]*dotq[2]*math.sin(q[2]))+(self.armP.m3*self.armP.l1*self.armP.s3*math.sin(q[1]+q[2])+self.armP.m3*self.armP.l2*self.armP.s3*math.sin(q[2]))*(dotq[0]**2)+self.armP.m3*self.armP.l2*self.armP.s3*math.sin(q[2])*(dotq[1]**2)+(self.armP.m3*self.armP.l1*self.armP.s3*math.sin(q[1]+q[2])+2*self.armP.m3*self.armP.l2*self.armP.s3*math.sin(q[2]))*dotq[0]*dotq[1]+(self.armP.m3*self.armP.l1*self.armP.s3*math.sin(q[1]+q[2])+self.armP.m3*self.armP.l2*self.armP.s3*math.sin(q[2]))*dotq[0]*dotq[2]+self.armP.m3*self.armP.l2*self.armP.s3*math.sin(q[2])*dotq[1]*dotq[2]])
    #print ("C:",C)
    #the commented version uses a non null stiffness for the muscles
    #beware of dot product Kraid times q: q may not be the correct vector/matrix
    #Gamma = np.dot((np.dot(armP.At, musclesP.fmax)-np.dot(musclesP.Kraid, q)), U)
    #Gamma = np.dot((np.dot(self.armP.At, self.musclesP.fmax)-np.dot(self.musclesP.Knulle, Q)), U)
    #above Knulle is null, so it can be simplified

    Gamma = np.dot(np.dot(self.armP.At, self.musclesP.fmax), U)
    #print ("Gamma:",Gamma)

    #computes the acceleration ddotq and integrates
    
    b = np.dot(self.armP.B, dotq)
    #print ("b:",b)

    ddotq = np.dot(Minv,Gamma - C - b)
    #print ("ddotq",ddotq)

    dotq += ddotq*self.dt
    #print ("dotq",dotq)
    q += dotq*self.dt
    #save the real state to compute the state at the next step with the real previous state
    q = self.jointStop(q)
    nextState = np.array([dotq[0], dotq[1], dotq[2], q[0], q[1], q[2]])
    return nextState

  def jointStop(self,q):
      '''
      Articular stop for the human arm
      The stops are included in the arm parameters file
      Shoulder: -0.6 <= q1 <= 2.6
      Elbow: -0.2 <= q2 <= 3.0
      Hand: -0.2 <= q3 <= 3.0
    
      Inputs:    -q: (3,1) numpy array
    
      Outputs:    -q: (3,1) numpy array
      '''
      if q[0] < self.armP.slb:
        q[0] = self.armP.slb
      elif q[0] > self.armP.sub:
        q[0] = self.armP.sub
      if q[1] < self.armP.elb:
        q[1] = self.armP.elb
      elif q[1] > self.armP.eub:
        q[1] = self.armP.eub
      if q[2] < self.armP.hlb:
        q[2] = self.armP.hlb
      elif q[2] > self.armP.hub:
        q[2] = self.armP.hub
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
      coordWrist = [self.armP.l1*np.cos(q[0])+self.armP.l2*np.cos(q[0] + q[1]), self.armP.l1*np.sin(q[0]) + self.armP.l2*np.sin(q[0] + q[1])]
      coordHand = [self.armP.l1*np.cos(q[0])+self.armP.l2*np.cos(q[0]+q[1])+self.armP.l3*np.cos(q[0]+q[1]+q[2]), self.armP.l1*np.sin(q[0])+self.armP.l2*np.sin(q[0]+q[1])+self.armP.l3*np.sin(q[0]+q[1]+q[2])]
      return coordElbow, coordWrist, coordHand
    
  def jacobian(self, q):
        J = np.array([[-self.armP.l1*np.sin(q[0])-self.armP.l2*np.sin(q[0]+q[1])-self.armP.l3*np.sin(q[0]+q[1]+q[2]), -self.armP.l2*np.sin(q[0]+q[1])-self.armP.l3*np.sin(q[0]+q[1]+q[2]), -self.armP.l3*np.sin(q[0]+q[1]+q[2])], 
                    [self.armP.l1*np.cos(q[0])+self.armP.l2*np.cos(q[0]+q[1])+self.armP.l3*np.cos(q[0]+q[1]+q[2]), self.armP.l2*np.cos(q[0]+q[1])+self.armP.l3*np.cos(q[0]+q[1]+q[2]), self.armP.l3*np.cos(q[0]+q[1]+q[2])]])
        return J

  def mgdEndEffector(self, q):
      '''
      Direct geometric model of the arm
    
      Inputs:     -q: (3,1) numpy array, the joint coordinates
    
      Outputs:
      -coordHand: hand coordinate
      '''
      #print q
      coordHand = [self.armP.l1*np.cos(q[0])+self.armP.l2*np.cos(q[0]+q[1])+self.armP.l3*np.cos(q[0]+q[1]+q[2]), self.armP.l1*np.sin(q[0])+self.armP.l2*np.sin(q[0]+q[1])+self.armP.l3*np.sin(q[0]+q[1]+q[2])]
      return coordHand

  def mgi(self, xi, yi):
      '''
      Inverse geometric model of the arm
    
      Inputs:     -xi: abscissa of the end-effector point
              -yi: ordinate of the end-effectior point

      Outputs:
              -q1: arm angle
              -q2: foreArm angle
              -q3: hand angle 
      '''
      a = (xi**2+yi**2-self.armP.l1**2-(self.armP.l2+self.armP.l3)**2)/(2*self.armP.l1*(self.armP.l2+self.armP.l3))
      try:
        q3 = 0
        q2 = math.acos(a)
        b = (yi*(self.armP.l1+(self.armP.l2+self.armP.l3)*math.cos(q2))-xi*(self.armP.l2+self.armP.l3)*math.sin(q2))/(xi*(self.armP.l1+(self.armP.l2+self.armP.l3)*math.cos(q2))+yi*(self.armP.l2+self.armP.l3)*math.sin(q2))
        q1 = math.atan(b)
        return q1, q2, q3
      except ValueError:
        print("forbidden value")
        print xi,yi
        return "None"    
