#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Olivier Sigaud

Module: plotFunctions

Description: some plotting functions
'''
import os
import random as rd
import numpy as np
from scipy import stats

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.mlab import griddata
   
plt.rc("figure", facecolor="white")
fig = plt.figure(1, figsize=(16,9))

def plotInput(net):
   
    x0 = []
    y0 = []
    output = []
    for i in range(50):
        for j in range(50):
            num = 0
            x = i/50.0
            y = j/50.0
            x0.append(x)
            y0.append(y)
            if x>0.5 and y>0.5:
                num = rd.random()/2+1.5
            else:
                num = rd.random()/2
            output.append(num)
    '''
    print "x0",x0
    print "y0",y0
    print "output",output
    '''

    xi = np.linspace(0,1,100)
    yi = np.linspace(0,1,100)
    zi = griddata(x0, y0, output, xi, yi)
    
    t1 = plt.scatter(x0, y0, c=output, marker=u'o', s=5, cmap=cm.get_cmap('RdYlBu'))
    CS = plt.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
    fig.colorbar(t1, shrink=0.5, aspect=5)

    plt.xlabel("handle")
    plt.ylabel("head")
    plt.title("Input map")

    plt.savefig("ImageBank/10input.png", bbox_inches='tight')
    plt.show(block = True)

def plotOutputMap(net,step):
    '''
    Cette fonction permet d'afficher le profil de cout des trajectoires
    
    Entrees:  -what: choix des donnees a afficher
    '''
   
    x0 = []
    y0 = []
    output = []
    for i in range(50):
        for j in range(50):
            x = i/50.0
            y = j/50.0
            x0.append(x)
            y0.append(y)
            z0 = net.computeOutput(np.array([x, y]))
            output.append(z0[0])
    '''
    print "x0",x0
    print "y0",y0
    print "output",output
    '''

    xi = np.linspace(0,1,100)
    yi = np.linspace(0,1,100)
    zi = griddata(x0, y0, output, xi, yi)
    
    t1 = plt.scatter(x0, y0, c=output, marker=u'o', s=5, cmap=cm.get_cmap('RdYlBu'))
    CS = plt.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
#    fig.colorbar(t1, shrink=0.5, aspect=5)

    xdata = []
    ydata = []
    for inp, targ in net.ds:
        xdata.append(inp[0])
        ydata.append(inp[1])
    t1 = plt.scatter(xdata, ydata, c='b', marker=u'o', s=5)

    plt.title("Output map")

    plt.savefig("ImageBank/10outputmap"+str(step)+".png", bbox_inches='tight')
    plt.show(block = True)
