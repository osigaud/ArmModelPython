

from Main.Main import generateFromRegression
from Regression.RunRegression import run
from Plot.plotFunctions import plotXYPositions
from Main.Test import *
from Utils.Chrono import Chrono
from Utils.ReadXmlFile import ReadXmlFile

setupFile="setupRBFN.xml"
thetaName="ScriptRBFNxml"
folder="TrajRBFNxml"

rs=ReadXmlFile(setupFile)
c = Chrono()
run(rs)
c.stop()
generateFromRegression(1, rs, thetaName, folder)
plotXYPositions("Regression",rs, folder,"All",True)
testRegression(rs, thetaName, folder)
