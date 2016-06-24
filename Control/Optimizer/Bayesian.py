from bayes_opt import BayesianOptimization
import numpy as np
import Utils.Compare as cp
import os
from Main.MainCMAES import launchCMAESForAllPoint, generateFromCMAESNController

class BayesianOpti(object):
    def __init__(self, rs):
        self.rs=rs
        experimentalFolder=os.getcwd()+"../Data/ExperimentalHits/"

        self.experiment1 = np.loadtxt(experimentalFolder+"hits0.005")[:,1]
        self.experiment2 = np.loadtxt(experimentalFolder+"hits0.01")[:,1]
        self.experiment3 = np.loadtxt(experimentalFolder+"hits0.02")[:,1]
        self.experiment4 = np.loadtxt(experimentalFolder+"hits0.04")[:,1]


    def trainAndCompare(self,K,noise):
        self.rs.deltaK=K
        for i in self.rs.sizeOfTarget:
            launchCMAESForAllPoint(self.rs,i,None, noise)
        
        resultPath=os.getcwd()+"../Data/CMAESK"+str(K)+"/"
        resultName="geneForHits/finalX/x.last"
        
        nameTheta = "Best"
        name = "geneForHits"
        nbret = 50
        generateFromCMAESNController(nbret, self.rs, nameTheta, name, noise)

        result1= np.loadtxt(resultPath+"0.005/"+resultName)
        result2= np.loadtxt(resultPath+"0.01/"+resultName)
        result3= np.loadtxt(resultPath+"0.02/"+resultName)
        result4= np.loadtxt(resultPath+"0.04/"+resultName)
        
        return -(cp.kl(result1, self.experiment1,0.0025)+cp.kl(result2, self.experiment2,0.005)+cp.kl(result3, self.experiment3,0.01)+cp.kl(result4, self.experiment4,0.02))

        
    def opti(self):

        bo = BayesianOptimization(self.trainAndCompareHit,
                          {'x': (10, 50), 'y': (0.1, 1.0)})
        
        bo.explore({'x': range(10,50), 'y': [0.1,0.25,0.5,0.75,1.]})
        
        bo.initialize({-11:{'x':20, 'y' : 0.5}})
        
        bo.maximize(init_points=5, n_iter=5, kappa=3.29)
        
        print(bo.res['max'])