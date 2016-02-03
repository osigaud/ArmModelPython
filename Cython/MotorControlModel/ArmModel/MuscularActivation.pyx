#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher
Cythonisation: Remi Cambuzat

Module: MuscularActivation

Description: Class used to compute the muscular activation vector U with motor noise
'''

import numpy as np
cimport numpy as np # import special compile-time information about the numpy module for cython

#cpdef : fonction accessible by python and cython
cpdef np.ndarray[np.uint8_t] getNoisyCommand(list U, float knoiseU):
    '''
    Computes the next muscular activation vector U

    Input:		-state: the state of the arm, numpy array

    Output:		-Unoise: the muscular activation vector U with motor noise
    '''
    #add the motor noise
    cdef list UnoiseTmp = []
    # Py_ssize_t is a signed int provided by Python which cover the indices value of an numpy array
    cdef Py_ssize_t x,sizeU
    sizeU = len(U)
    for i in range(sizeU):
        UnoiseTmp.append(U[i]*(1+ np.random.normal(0,knoiseU)))
    #check if the muscular activation are normed, ie between 0 and 1
    UnoiseTmp = muscleFilter(UnoiseTmp)
    #put U in column vector form
    return np.array(UnoiseTmp)

#cdef : fonction only accessible by cython
cpdef np.ndarray[np.uint8_t] muscleFilter(list UnoiseTmp):
    '''
    Makes sure that the muscular activation is between 0 and 1

    Input:		-UnoiseTmp: muscular activation vector

    Output:		-UnoiseTmp: muscular activation vector
    '''
    cdef Py_ssize_t x,size
    size = len(UnoiseTmp)
    for i in range(size):
       if UnoiseTmp[i] < 0:
           #print "U unfiltered :", UnoiseTmp[i]
           UnoiseTmp[i] = 0
       elif UnoiseTmp[i] > 1:
           #print "U unfiltered :", UnoiseTmp[i]
           UnoiseTmp[i] = 1
    return np.array(UnoiseTmp)








