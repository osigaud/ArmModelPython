#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: cluster

Description: script to run the project on cluster
'''
import sys
from Utils.FileWriting import watch_all_theta_files
from Utils.ReadXmlFile import ReadXmlFile
from Main.MainCMAES import launchCMAESForAllPoint

watch_all_theta_files()

def run_missing_CMAES():
    rs = ReadXmlFile(sys.argv[1])
    nb_process=20
    for gamma in range(3,9):
        for targetsize in [0.005, 0.01, 0.02, 0.04]:
            for i in range(15):
                if not check_if_theta_file_exists(gamma,targetsize,i):
                    print("gamma =",gamma,"nbtheta:",count_best_files(gamma))
                p = ThreadPool(processes=15)
                #run cmaes on each targets size on separate processor
                posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
                p.map(partial(launchCMAESForSpecificTargetSizeAndSpecificPoint, targetsize, rs, True, noise), enumerate(posIni))
            p.close()
            p.join()
