import numpy as np
import os

import ArmModel

print("--------------Muscles---------------")

from ArmModel import MusclesParameters as MusclesParam

Muscles = MusclesParam.MusclesParameters()

print("--------------ArmParam---------------")

from ArmModel import ArmParameters as ArmParam

ArmParam = ArmParam.ArmParameters()

print("--------------Arm---------------")


U = np.array([0,1,2,3,4,5])
state = np.array([00,11,22,33])

print(U,state)

print("     ---------Arm cython---------")

from ArmModel import Arm as Arm
arm1 = Arm.Arm()

dotq1,q1 = ArmModel.Arm.getDotQAndQFromStateVector(state)
print dotq1
print q1

print ("Compute Next state : ",arm1.computeNextState(U, state))







