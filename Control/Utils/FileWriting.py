#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Created on 15 févr. 2016

@author: Corentin Arnaud 
'''
from shutil import copyfile
import os
import numpy as np

def check_if_theta_file_exists(gamma,targetsize,num):
    dir = "../Data/CMAESK10gamma" + str(gamma) + "/" + str(targetsize) + "/" + str(num) + "/Theta/"
    return os.path.isdir(dir)

def count_best_files(gamma):
    count=0
    for targetsize in [0.005, 0.01, 0.02, 0.04]:
        for i in range(15):
            if check_if_theta_file_exists(gamma,targetsize,i):
                count+=1
    return count

def watch_all_theta_files():
    for gamma in range(3,10):
        print("gamma =",gamma,"nbtheta:",count_best_files(gamma))

def checkIfFolderExists(name):
    if not os.path.isdir(name):
        os.makedirs(name)

def findDataFilename(foldername, name, extension):
    i = 1
    checkIfFolderExists(foldername)
    tryName = name + "1" + extension
    while tryName in os.listdir(foldername):
        i += 1
        tryName = name + str(i) + extension
    filename = foldername + tryName
    return filename

def copyRegressiontoCMAES(rs, name, size):
    cmaname =  rs.CMAESpath + str(size) + "/"
    checkIfFolderExists(cmaname)
    savenametheta = rs.path + name + ".theta"
    copyfile(savenametheta, cmaname + name + ".theta")
    
    if(rs.regression=="RBFN"):
        savenamestruct = rs.path + name + ".struct"
        copyfile(savenamestruct, cmaname + name + ".struct")
        
def writeArray(numpyArray, foldername, name, extension):
    checkIfFolderExists(foldername)
    np.savetxt(foldername+name+extension, numpyArray)
    
