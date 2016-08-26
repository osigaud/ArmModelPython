#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: cluster

Description: script to run the project on cluster
'''
import sys
from Utils.FileWriting import watch_all_theta_files
import os
import numpy as np

def check_if_theta_file_exists(targetsize,num):
    dir = "../Data/HitVelgamma6" + "/" + str(targetsize) + "/" + str(num) + "/Theta/"
    return os.path.isdir(dir)

def count_best_files():
    for targetsize in [0.005, 0.01, 0.02, 0.04]:
        count=0
        for i in range(15):
            if check_if_theta_file_exists(targetsize,i):
                count+=1
        print("target =",targetsize,"nbtheta:",count)
    return count

count_best_files()
