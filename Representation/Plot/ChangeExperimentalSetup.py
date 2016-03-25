#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: ChangeExperimentalSetup

Description: Script to change the initial position
'''

import numpy as np
import math
import matplotlib.pyplot as plt
from Utils.FileSaving import saveBin

def changeExperimentCircular():
    x0, y0 = 0, 0.6175
    r = [0.18, 0.20, 0.22, 0.24]
    x, y, exper = [], [], []
    #nppi = np.arange(5*math.pi/4, 7*math.pi/4, math.pi/10)
    nppi = np.linspace(5*math.pi/4, 7*math.pi/4, 12)
    for i in range(nppi.shape[0]):
        for j in range(len(r)): 
            xt = x0 + r[j] * math.cos(nppi[i])
            yt = y0 + r[j] * math.sin(nppi[i])
            x.append(xt)
            y.append(yt)
            exper.append((xt, yt))
            
    plt.figure()
    plt.scatter(x, y, c = 'b')
    plt.show()
    
    saveBin("PosIniExperimentCircular48", exper)

#changeExperimentCircular()

def changeExperimentSquare():
    x = np.linspace(-0.15,0.15,20)
    y = np.linspace(0.35, 0.5, 3)
    posx, posy, posxy = [], [], []
    for i in range(len(x)):
        for j in range(len(y)):
            posx.append(x[i])
            posy.append(y[j])
            posxy.append((x[i], y[j]))
    saveBin("PosIniExperimentSquare", posxy)
    
#changeExperimentSquare()



