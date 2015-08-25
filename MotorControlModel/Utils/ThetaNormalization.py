#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: Functions

Description: On retrouve dans ce fichier les fonctions pour normaliser theta
'''

import numpy as np

def normalization(theta):
    maxT = np.max(np.abs(theta))/10.0
    for i in range(len(theta)):
        theta[i] = theta[i] / maxT
    return theta, maxT

def unNormalization(theta, maxT):
    for i in range(len(theta)):
        theta[i] = theta[i] * maxT
    return theta
