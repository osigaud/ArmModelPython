
from Main.Main import generateFromRegression


<<<<<<< HEAD
from Regression.RunRegression import run
from Plot.plotFunctions import  plotXYPositions, plotArticularPositions
from Main.Test import testRegression
from Utils.ReadXmlFile import ReadXmlFile
=======
from Utils.Chrono import Chrono
from Utils.ReadSetupFile import ReadSetupFile
from GlobalVariables import pathDataFolder
>>>>>>> parent of d88d011... maj

setupFile="setupNN"
thetaName="ScriptNN1"
folder="TrajNN1"

<<<<<<< HEAD

rs=ReadXmlFile(setupFile)
run(rs)
generateFromRegression(1, rs, folder)
plotXYPositions("Regression",rs, foldername =folder,targetSize ="All",plotEstim=True)
plotArticularPositions("Regression",rs, folder)
testRegression(setupFile)
=======
#c = Chrono()
#run(setupFile,"ScriptNN",False)
#c.stop()
#generateFromRegression(1, setupFile, thetaName, folder)
plotXYPositions("Regression", setupFile=setupFile, foldername =folder,targetSize ="All",plotEstim=True)
#plotArticularPositions("Regression",setupFile, folder)
>>>>>>> parent of d88d011... maj
