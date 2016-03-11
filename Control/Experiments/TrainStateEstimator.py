from Utils.FileReading import loadTrajForModel
from GlobalVariables import  pathDataFolder
import numpy as np
from Regression.RunRegression import regressionDict
import os
from Regression.NeuralNet import NeuralNet
















def run(regressionSetup, delay):
    
    
    stateAndCommand, nextState = loadTrajForModel(pathDataFolder + "Brent/", delay)
    
    
    
    np.random.seed(0)
    np.random.shuffle(stateAndCommand)
    np.random.seed(0)
    np.random.shuffle(nextState)
    
    print("nombre d'echantillons: ", len(stateAndCommand))

    fa = regressionDict[regressionSetup.regression](regressionSetup)
    fa.getTrainingData(stateAndCommand, nextState)
    fa.train()
    fa.saveTheta(regressionSetup.path+ regressionSetup.thetaFile+".theta")
    
    
class NeuraNetParameter():
        def __init__(self, delay):
            self.regression="NeuralNet"
            self.thetaFile="EstimTheta"
            self.path = os.getcwd()+"/Experiments/theta/"
            self.inputLayer="linear"
            self.outputLayer="tanh"
            self.hiddenLayers=[]
            for i in range(3):
                self.hiddenLayers.append(("tanh",10))
            self.inputDim=4+6*delay
            self.outputDim=4
            self.learningRate=0.001
            self.momentum=0.
            self.bias=True
            
            
if __name__ == "__main__" :
    delay = 25
    regressionSetup = NeuraNetParameter(delay)
    run(regressionSetup, delay)