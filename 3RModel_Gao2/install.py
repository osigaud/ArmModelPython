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
    os.system('sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose')
    if b == 2:
        try:
            os.system('sudo easy_install numpy scipy Sphinx numpydoc nose pykalman')
            os.system('sudo pip install cma')
            os.system('sudo easy_install cython')
            os.system('sudo pip install distlib')
        except:
            pass
    elif b == 3:
        try:
            os
            os.system('sudo easy_install3 numpy scipy Sphinx numpydoc nose pykalman')
            os.system('sudo pip3 install cma')
            os.system('sudo easy_install3 cython')
            os.system('sudo pip3 install distlib')
        except:
            pass
    os.system('clear')

pk = checkPackages()
installMissingPackage(pk)
