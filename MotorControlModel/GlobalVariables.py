#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: GlobalVariables

Description: global variables used in the project
'''

import os

pathWorkingDirectory = os.getcwd()
pathListForm = pathWorkingDirectory.split("/")
pathDataFolder = pathWorkingDirectory.replace(pathListForm[len(pathListForm)-1], '') + "Data/"
BrentTrajectoriesFolder = pathDataFolder + "Brent/"
cmaesPath = "CMAES"
det = False
