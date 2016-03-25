from Utils.FileReading import loadTrajForModel
from GlobalVariables import  pathDataFolder
import numpy as np
from Regression.RunRegression import regressionDict
import os
from Regression.NeuralNet import NeuralNet
















def run(regressionSetup, delay):
    
    
    stateAndCommand, nextState = loadTrajForModel(pathDataFolder + "Brent/", delay)
    
    
    
    
    print("nombre d'echantillons: ", len(stateAndCommand))

    fa = regressionDict[regressionSetup.regression](regressionSetup)
    fa.getTrainingData(stateAndCommand, nextState)
    fa.train()
    
    
class NeuraNetParameter():
        def __init__(self, delay, name):
            self.regression=name
            self.thetaFile="EstimTheta"+name
            self.path = os.getcwd()+"/Experiments/theta/"
            self.inputLayer="linear"
            self.outputLayer="tanh"
            self.hiddenLayers=[]
            for i in range(1):
                self.hiddenLayers.append(("tanh",100))
            self.inputDim=4+6*delay
            self.outputDim=4
            self.learningRate=0.01
            self.momentum=0.
            self.bias=True
            
            
if __name__ == "__main__" :
    delay = 5
    regressionSetup = NeuraNetParameter(delay, "NeuralNetTF")
    run(regressionSetup, delay)