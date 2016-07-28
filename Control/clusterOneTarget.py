#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: cluster

Description: script to run the project on cluster
'''
import sys
from Utils.ReadXmlFile import ReadXmlFile
from Main.MainCMAES import launchCMAESForSpecificTargetSize

if __name__ == '__main__':
    rs = ReadXmlFile(sys.argv[1])
    launchCMAESForSpecificTargetSize(float(sys.argv[2]),rs,False)
