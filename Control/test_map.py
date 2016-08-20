#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: cluster

Description: script to run the project on cluster
'''
import sys
import os
import numpy as np
from Utils.ReadXmlFile import ReadXmlFile
from Main.MainCMAES import launchCMAESForVariousGamma
from functools import partial
from multiprocess.pool import ThreadPool
from GlobalVariables import pathDataFolder
from Utils.Chrono import Chrono

def check_if_theta_file_exists(gamma,targetsize,num):
    dir = "../Data/CMAESK10gamma" + str(gamma) + "/" + str(targetsize) + "/" + str(num) + "/Theta/"
    return os.path.isdir(dir)

def launch_missing_theta_files(rs):
    for gamma in range(3,10):
        launch(rs,gamma)

def launch(rs,gamma):
    count=0
    for targetsize in [0.005, 0.01, 0.02, 0.04]:
        points = []
        for i in range(15):
            if not check_if_theta_file_exists(gamma,targetsize,i):
                points.append(i)
        launchCMAESForListOfPoints(targetsize, rs, True, gamma, points)
    return count

def launchCMAESForListOfPoints(sizeOfTarget, rs, save, gamma, points):
    p = ThreadPool(processes=len(points))
    posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
    p.map(partial(launchCMAESForVariousGamma, sizeOfTarget, rs, save, gamma), [[i, posIni[i]] for i in points])
    p.close()
    p.join()
