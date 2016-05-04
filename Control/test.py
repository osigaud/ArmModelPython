from Utils.Chrono import Chrono
from Utils.ReadXmlFile import ReadXmlFile
from Utils.FileReading import loadTrajForModel
from Experiments.Experiments import Experiments
from GlobalVariables import  pathDataFolder
from Regression.NeuraNetTF import NeuralNetTF
import numpy as np

rs=ReadXmlFile("setupNN1th.xml")

foldername = rs.CMAESpath + str(0.04) + "/"
thetaname = foldername + rs.thetaFile
exp = Experiments(rs, 0.04, False, foldername, thetaname,rs.popsizeCmaes,rs.period, "Inv")
#exp2 = Experiments(rs, 0.04, False, foldername, thetaname,rs.popsizeCmaes,rs.period,"Reg")
exp3 = Experiments(rs, 0.04, False, foldername, thetaname,rs.popsizeCmaes,rs.period,"Hyb")

'''
c=Chrono()
for i in range(30):
    cost, time = exp.runMultiProcessTrajectories(10)
c.stop()
print("Average cost: ", cost)
print("Average time: ", time)
'''
stateAndCommand, nextState = loadTrajForModel(pathDataFolder + "Brent/", 10)
#stateAndCommand, nextState = loadTrajForModel(pathDataFolder + "CMAEScluster/0.02/cluster/Log/", 10)
state=stateAndCommand[:,:4]
command=stateAndCommand[:,4:]

#exp2.tm.stateEstimator.initStore(state)
exp3.tm.stateEstimator.initStore(state)

nbE=0
error=np.zeros(4)
c=Chrono()
for i in range(state.shape[0]):
    if(state[i][0]==0 and state[i][1]==0):
        exp.tm.stateEstimator.initStore(state[i])
        tmp=0
    inferredState = exp.tm.stateEstimator.getEstimState(state[i], command[i][:6])
    if i<state.shape[0]-1 and not (state[i+1][0]==0 and state[i+1][1]==0):
        error += (inferredState - state[i+1])**2
        nbE+=1
    tmp+=1
c.stop()
print (np.mean(error/nbE))

'''
nbE=0
error=np.zeros(4)
c=Chrono()
for i in range(1000):
    if(state[i][0]==state[i][1]):
        exp2.tm.stateEstimator.initStore(state[i])
        tmp=0
    inferredState = exp2.tm.stateEstimator.getEstimState(state[i], command[i][:6])
    
    if tmp >10:
        error += (inferredState - nextState[i-10])**2
        nbE+=1
    tmp+=1
c.stop()
print (np.mean(error/nbE))
'''
nbE=0
error=np.zeros(4)
c=Chrono()
for i in range(state.shape[0]):
    if(state[i][0]==0 and state[i][1]==0):
        exp3.tm.stateEstimator.initStore(state[i])
        tmp=0
    inferredState = exp3.tm.stateEstimator.getEstimState(state[i], command[i][:6])
    if i<state.shape[0]-1 and not (state[i+1][0]==0 and state[i+1][1]==0):
        error += (inferredState - state[i+1])**2
        nbE+=1
    tmp+=1
c.stop()
print (np.mean(error)/nbE)

'''
reg=exp3.tm.stateEstimator.regression
reg.getTrainingData(stateAndCommand, nextState)
print reg.meanSquareError()
#print reg.meanSquareError.eval(session=reg.sess,feed_dict={reg.x: stateAndCommand, reg.y_: nextState})
'''

