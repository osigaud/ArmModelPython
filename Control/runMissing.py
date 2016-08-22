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
from Main.MainCMAES import launchCMAESMissing
from functools import partial
from multiprocess.pool import ThreadPool
from GlobalVariables import pathDataFolder
from Utils.Chrono import Chrono

def check_if_theta_file_exists(gamma,target_size,num):
    dir = "../Data/CMAESK10gamma" + str(gamma) + "/" + str(target_size) + "/" + str(num) + "/Theta/"
    return os.path.isdir(dir)

def launch(rs,gamma):
    all_points = []
    for target_size in [0.005, 0.01, 0.02, 0.04]:
        for i in range(15):
            if not check_if_theta_file_exists(gamma,target_size,i):
                all_points.append([i, target_size])

    print('nb processes',len(all_points))
    p = ThreadPool(processes=len(all_points))
    posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
    p.map(partial(launchCMAESMissing, rs, True, gamma), [[point[0], posIni[point[0]], point[1]] for point in all_points])
    p.close()
    p.join()

if __name__ == '__main__':
    rs = ReadXmlFile(sys.argv[1])
    launch(rs, float(sys.argv[2]))
