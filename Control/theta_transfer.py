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
from shutil import copyfile

def check_if_theta_file_exists(targetsize,num):
    dir = "../Data/HitVelgamma6" + "/" + str(targetsize) + "/" + str(num) + "/Theta/"
    return os.path.isdir(dir)

def copy_file(targetsize,num):
    sourcename = "../Data/CMAESK10gamma6" + "/" + str(targetsize) + "/" + str(num) + "/Best.theta"
    destname = "../Data/HitVelgamma6" + "/" + str(targetsize) + "/" + str(num) + "/Best.theta"
    print('cp pour ' + str(targetsize) + "/" + str(num))
    copyfile(sourcename,destname)

def transfer_best_files():
    for targetsize in [0.005, 0.01, 0.02, 0.04]:
        for i in range(15):
            if not check_if_theta_file_exists(targetsize,i):
                copy_file(targetsize,i)

transfer_best_files()
