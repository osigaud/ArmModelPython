
from Main.Main import generateFromRegression



from Regression.RunRegression import run
from Plot.plotFunctions import  plotXYPositions, plotArticularPositions
from Main.Test import testRegression
from Utils.ReadXmlFile import ReadXmlFile



setupFile="setupNN"
thetaName="ScriptNN1"
folder="TrajNN1"


rs=ReadXmlFile(setupFile)
run(rs)
generateFromRegression(1, rs, folder)
plotXYPositions("Regression",rs, foldername =folder,targetSize ="All",plotEstim=True)
plotArticularPositions("Regression",rs, folder)
testRegression(setupFile)

