#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher + Olivier Sigaud

Module: install
'''

import os
import site

#------------------- install environment -----------------------------------------------------------------------------------

def checkPackages():
    a = site.getsitepackages()
    packageList = os.listdir(a[0])
    packageNeeded = {}
    listOfPackageNeeded = ['pykalman', 'cma', 'cython']
    for el in listOfPackageNeeded:
        packageNeeded[el] = 0
    for el1 in listOfPackageNeeded:
        for el2 in packageList:
            if el1 in el2:
                packageNeeded[el1] = 1
    print(packageNeeded)    
    return packageNeeded

def installMissingPackage(packageList):
    a = site.getsitepackages()
    a = a[0]
    a = a.split('/')
    for el in a:
        if 'python' in el:
            b = el.replace('python', '')
            b = int(float(b))
    os.system('sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose' 
              +  'libblas-dev liblapack-dev libatlas-base-dev gfortran')
    if b == 2:
        try:
            os.system('sudo pip2 install numpy scipy Sphinx numpydoc nose pykalman')
            os.system('sudo pip2 install cma lxml multiprocess')
            os.system('sudo pip2 install https://github.com/pybrain/pybrain/archive/0.3.3.zip')
            os.system('sudo pip2 install cython')
            os.system('sudo pip2 install distlib')
            os.system('sudo pip install --upgrade https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-0.7.1-cp27-none-linux_x86_64.whl')
            os.system('sudo pip install git+https://github.com/fmfn/BayesianOptimization.git')
        except:
            pass
    elif b == 3:
        print "This python 2.7 as default"


pk = checkPackages()
installMissingPackage(pk)
