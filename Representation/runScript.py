#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher + Olivier Sigaud

Module: runScript

Description: main script to run what we want in the project
'''

import numpy as np
import random as rd

from Regression.NeuralNet import NeuralNet

from Plot.plotFunctions import plotOutputMap, plotInput

#----------------------------- main list of available actions ----------------------------------------------------------------------

def printMainMenu():
    print('Available scripts:')
    print('		1 plot output')

def runChoice():
    checkL = True
    while checkL:
        try:
            printMainMenu()
            choix = input('Enter the number corresponding to the script you want to run: ')
            choix = int(choix)
            checkL = False
        except:
            print("Enter a number.")
    run()

def run():
    plotInput()
    n = NeuralNet(2,1)
    for i in range(5):
        print n.getTheta()
        plotOutputMap(n,i*2)
        n.train()
        n.train()
    plotOutputMap(n,10)

run()

