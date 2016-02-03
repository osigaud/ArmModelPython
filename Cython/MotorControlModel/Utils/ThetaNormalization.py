#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: Functions

Description: On retrouve dans ce fichier les fonctions pour normaliser theta
'''

import numpy as np

def normalization(theta):
    minT = np.min(theta)
    maxT = np.max(theta)
    for i in range(len(theta)):
        theta[i] = (theta[i]-minT) / (maxT-minT)
    return theta, minT, maxT

def unNormalization(theta, minT, maxT):
    for i in range(len(theta)):
        theta[i] = theta[i] * (maxT-minT) + minT
    return theta
