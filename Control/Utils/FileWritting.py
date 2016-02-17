#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Created on 15 févr. 2016

@author: arnaud
'''
from shutil import copyfile
import os
import numpy as np

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
    