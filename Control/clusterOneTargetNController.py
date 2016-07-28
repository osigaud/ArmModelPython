#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud

Module: cluster

Description: script to run the project on cluster
'''
import sys
from Utils.ReadXmlFile import ReadXmlFile
from Main.MainCMAES import launchCMAESForAllPoint

if __name__ == '__main__':
    rs = ReadXmlFile(sys.argv[1])
    launchCMAESForAllPoint(rs, float(sys.argv[2]),False)
