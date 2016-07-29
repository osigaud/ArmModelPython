#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud


Description: plot data from CMAES
'''
from Utils.ReadXmlFile import ReadXmlFile
from Plot.plotFunctions import *

fileName="setupK10gamma3.xml"
folder="Best"

rs = ReadXmlFile(fileName)

plotScattergram("OPTI",folder, rs)
plotCostColorMap("OPTI",rs, folder)
plotCostColorMapFor12("OPTI",rs, folder) 
plotTimeColorMap("OPTI",rs, folder)
plotPerfSizeDist(folder, rs)
plotTimeDistanceTarget(folder, rs)
plotFittsLaw(folder, rs)
plotXYPositions("OPTI",rs, folder,"All",False)
plotXYPositions("OPTI",rs, folder,"All",False,zoom=True)
plotVelocityProfile("OPTI",rs,folder)