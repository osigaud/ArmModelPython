from Utils.Chrono import Chrono
from Utils.ReadXmlFile import ReadXmlFile
from Experiments.Experiments import Experiments
from Regression.NeuraNetTF import NeuralNetTF
import numpy as np

rs=ReadXmlFile("setupNN1th.xml")

foldername = rs.CMAESpath + str(0.04) + "/"
thetaname = foldername + rs.thetaFile
exp = Experiments(rs, 0.04, False, foldername, thetaname,rs.popsizeCmaes,rs.period)
exp2 = Experiments(rs, 0.04, False, foldername, thetaname,rs.popsizeCmaes,rs.period,"Reg")
exp3 = Experiments(rs, 0.04, False, foldername, thetaname,rs.popsizeCmaes,rs.period,"Hyb")

'''
c=Chrono()
for i in range(30):
    cost, time = exp.runMultiProcessTrajectories(10)
c.stop()
print("Average cost: ", cost)
print("Average time: ", time)
'''

state=np.array([0,0.3,0,0.3])
command=np.array([0.1,0.2,0.6,0.2,0,0.3])
exp.tm.stateEstimator.initStore(state)
exp2.tm.stateEstimator.initStore(state)
exp3.tm.stateEstimator.initStore(state)


c=Chrono()
for i in range(1000):
    exp.tm.stateEstimator.getEstimState(state, command)
c.stop()


c=Chrono()
for i in range(1000):
    exp2.tm.stateEstimator.getEstimState(state, command)
c.stop()

c=Chrono()
for i in range(1000):
    exp3.tm.stateEstimator.getEstimState(state, command)
c.stop()



