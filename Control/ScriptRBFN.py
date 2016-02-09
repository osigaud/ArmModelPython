import numpy as np
import random as rd

from Main.Main import generateFromRegression, generateFromCMAES, generateRichDataFromRegression, generateRichDataFromCMAES, launchCMAESForAllTargetSizes, launchCMAESForSpecificTargetSize

from Regression.RunRegression import run, UnitTestController, UnitTestArmModel
from Regression.Regression import UnitTest

from Plot.plotFunctions import trajectoriesAnimation, plotCostColorMap, plotTimeColorMap, plotTimeDistanceTarget, plotFittsLaw, plotPerfSizeDist, plotVelocityProfile, plotXYPositions, plotXYEstimError, plotXYEstimErrorOfSpeed, plotArticularPositions, plotInitPos, plotMuscularActivations, plotScattergram, plotHitDispersion, plotExperimentSetup, plotCMAESProgress, plotTrajsInRepo, plotManipulability, plotManipulability2
from Main.Test import *
from Utils.Chrono import Chrono
from Utils.ReadSetupFile import ReadSetupFile
from GlobalVariables import pathDataFolder


setupFile="setupRBFN"
thetaName="ScriptRBFN1"
folder="TrajRBFN1"

c = Chrono()
run(setupFile,thetaName,False)
c.stop()
generateFromRegression(1, setupFile, thetaName, folder)
plotXYPositions("Regression", setupFile, folder,"All",True)
testRegression(setupFile, thetaName, folder)