#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas beucher, Corentin Arnaud

Module: ReadSetupFile

Description: On retrouve dans ce fichier une fonction permettant de lire le fichier de configuration 
'''
import math
from GlobalVariables import pathWorkingDirectory, pathDataFolder, cmaesPath

class ReadSetupFile:
    
    def __init__(self, setupNameFile):
        self.name = "setup"
        self.readingSetupFile(setupNameFile)
    
    def readingSetupFile(self,setupNameFile):
        '''
        Reads the setup file
        '''
        print("ReadSetupFile L23")
        #Recuperation des donnees du fichier de configuration
        with open(pathWorkingDirectory + "/"+ setupNameFile, "r") as file:
            alls = file.read()
        #Split to get line to line
        allsByLign = alls.split("\n")
        #line 1, mode stocastic
        self.det = (not allsByLign[0].split(":")[1]=="Y")
        #line 2, regression type
        self.regression=allsByLign[1].split(":")[1]
        #reading line 3, nombre de features
        self.numfeats = int((allsByLign[2].split(":"))[1])
        #reading line 4, choix du Parametre gamma cost function
        self.gammaCF = float((allsByLign[3].split(":"))[1])
        #reading line 5, choix du Parametre rho cost function
        self.rhoCF = int((allsByLign[4].split(":"))[1])
        #reading line 6, choix du Parametre upsilon cost function
        self.upsCF = float((allsByLign[5].split(":"))[1])
        #reading line 7, Pour CMAES, sigma
        self.sigmaCmaes = float((allsByLign[6].split(":"))[1])
        #reading line 8, Pour CMAES, maxIteration
        self.maxIterCmaes = int((allsByLign[7].split(":"))[1])
        #reading line 9, POUR CMAES, popsize
        self.popsizeCmaes = int((allsByLign[8].split(":"))[1])
        #reading line 10, Taille de la cible pour l'experimentation
        sizes=allsByLign[9].split(":")[1].split("/")
        self.sizeOfTarget = [float(sizes[0]), float(sizes[1]), float(sizes[2]), float(sizes[3])]
        #reading line 11, abscisse de la cible
        self.XTarget = float((allsByLign[10].split(":"))[1])
        #reading line 12, ordonnee de la cible
        self.YTarget = float((allsByLign[11].split(":"))[1])
        #reading line 13, Pas de temps utilise pour l'experimentation
        self.dt = float((allsByLign[12].split(":"))[1])
        #reading line 14, positions initiales
        self.experimentFilePosIni = (allsByLign[13].split(":"))[1]
        #reading line 15, number of iteration to stop unresolved trajectory
        self.numMaxIter = int((allsByLign[14].split(":"))[1])
        #reading line 16, Delai utilise pour le filtre de kalman(int)
        self.delayUKF = int((allsByLign[15].split(":"))[1])
        #reading line 17, Nombre de repetition pour chaque trajectoire(int)
        self.numberOfRepeatEachTraj = int((allsByLign[16].split(":"))[1])
        #reading line 18, Dimension de l'entree, ici le vecteur position(int)
        self.inputDim = int((allsByLign[17].split(":"))[1])
        #reading line 19, Dimension de la sortie, ici le vecteur d'activation musculaire
        self.outputDim = int((allsByLign[18].split(":"))[1])
        #reading line 20, lamda, regularization factor in RBFNs
        self.lamb = float((allsByLign[19].split(":"))[1])
        #reading line 21, period, for plotting performance improvement
        self.period = int((allsByLign[20].split(":"))[1])

        if(self.regression=="RBFN"):
            self.path = pathDataFolder + "RBFN/" + str(self.numfeats) + "feats/"
        else :
            self.path = pathDataFolder + "NN/"
        self.CMAESpath = pathDataFolder + cmaesPath + "/ResCma"

    def getDistanceToTarget(self, x, y):
        return math.sqrt((x - self.XTarget)**2 + (y - self.YTarget)**2)

           
        

