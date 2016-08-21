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
    all_points = []
    for targetsize in [0.005, 0.01, 0.02, 0.04]:
        points = []
        for i in range(15):
            if not check_if_theta_file_exists(gamma,targetsize,i):
                points.append(i)
        all_points.append(points)
    launchCMAESForListOfPoints(rs, True, gamma, all_points)

def launchCMAESForListOfPoints(rs, save, gamma, all_points):
    nb_proc = 0
    for i in range(len(all_points)):
        nb_proc+=len(all_points[i])
    print('nb processes',nb_proc)
    p = ThreadPool(processes=nb_proc)
    posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
    tgt = 0
    for targetsize in [0.005, 0.01, 0.02, 0.04]:
        points = all_points[tgt]
        p.map(partial(launchCMAESMissing, targetsize, rs, save, gamma), [[i, posIni[i]] for i in points])
    p.close()
    p.join()
