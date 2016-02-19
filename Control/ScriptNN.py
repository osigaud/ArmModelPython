import numpy as np
import random as rd
import os

from Main.Main import generateFromRegression, generateFromCMAES, generateRichDataFromRegression, generateRichDataFromCMAES, launchCMAESForAllTargetSizes, launchCMAESForSpecificTargetSize

from Regression.RunRegression import run, UnitTestController, UnitTestArmModel
from Regression.Regression import UnitTest

from Plot.plotFunctions import trajectoriesAnimation, plotCostColorMap, plotTimeColorMap, plotTimeDistanceTarget, plotFittsLaw, plotPerfSizeDist, plotVelocityProfile, plotXYPositions, plotXYEstimError, plotXYEstimErrorOfSpeed, plotArticularPositions, plotInitPos, plotMuscularActivations, plotScattergram, plotHitDispersion, plotExperimentSetup, plotCMAESProgress, plotTrajsInRepo, plotManipulability, plotManipulability2

from Main.Test import testRegression
from Utils.Chrono import Chrono
from Utils.ReadSetupFile import ReadSetupFile
from GlobalVariables import pathDataFolder

setupFile="setupNN1ReLu.xml"
thetaName="NN1ReLu"
folder="TrajNN1ReLu"

#c = Chrono()
#run(setupFile)
#c.stop()
generateFromRegression(1, setupFile, folder)
plotXYPositions("Regression", foldername =folder,targetSize ="All",plotEstim=True)
plotArticularPositions("Regression",setupFile, folder)
testRegression(setupFile)
