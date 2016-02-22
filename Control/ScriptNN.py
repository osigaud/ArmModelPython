
from Main.Main import generateFromRegression


from Regression.RunRegression import run
from Plot.plotFunctions import  plotXYPositions, plotArticularPositions
from Main.Test import testRegression
from Utils.ReadXmlFile import ReadXmlFile

setupFile="setupNN1ReLu.xml"
thetaName="NN1ReLu"
folder="TrajNN1ReLu"


rs=ReadXmlFile(setupFile)
run(rs)
generateFromRegression(1, rs, folder)
plotXYPositions("Regression",rs, foldername =folder,targetSize ="All",plotEstim=True)
plotArticularPositions("Regression",rs, folder)
testRegression(setupFile)
