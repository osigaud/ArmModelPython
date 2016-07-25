#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: compare

test comparison
'''

import Utils.Compare as cp
import numpy as np
import matplotlib.pyplot as plt

experimentalFolder="/home/arnaud/stage/ArmModelPython/Data/ExperimentalHits/"

experiment1 = np.loadtxt(experimentalFolder+"hits0.005")[:,1]
experiment2 = np.loadtxt(experimentalFolder+"hits0.01")[:,1]
experiment3 = np.loadtxt(experimentalFolder+"hits0.02")[:,1]
experiment4 = np.loadtxt(experimentalFolder+"hits0.04")[:,1]

print(experiment1.shape[0])

resultPath="/home/arnaud/stage/ArmModelPython/Data/CMAESK20/"
resultName="50/finalX/x.last"

result1= np.loadtxt(resultPath+"0.005/"+resultName)
result2= np.loadtxt(resultPath+"0.01/"+resultName)
result3= np.loadtxt(resultPath+"0.02/"+resultName)
result4= np.loadtxt(resultPath+"0.04/"+resultName)


print ("chi2, taille 0.005 : " + str(cp.chi2(result1, experiment1)))
print ("KS, taille 0.005 : " + str(cp.ks(result1, experiment1)))
print ("MWWRankSum, taille 0.005 : " + str(cp.MWWRankSum(result1, experiment1)))
print ("KL, taille 0.005 : " + str(cp.kl(result1, experiment1)))

print cp.mw(np.random.normal(0,0.005,10000),np.random.normal(0,0.005,10000))
print cp.ks(np.random.normal(0,0.005,10000),np.random.normal(0,0.005,10000))
print (cp.kl(np.random.normal(0,0.005,10000),np.random.normal(0,0.005,10000)))