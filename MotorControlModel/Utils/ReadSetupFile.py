#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas beucher

Module: ReadSetupFile

Description: On retrouve dans ce fichier une fonction permettant de lire le fichier de configuration 
'''
import math

from GlobalVariables import pathWorkingDirectory, pathDataFolder, cmaesPath

class ReadSetupFile:
    
    def __init__(self):
        self.name = "setup"
        self.readingSetupFile()
    
    def readingSetupFile(self):
        '''
        Reads the setup file
        '''
        #Recuperation des donnees du fichier de configuration
        with open(pathWorkingDirectory + "/setupFile", "r") as file:
            alls = file.read()
        #Split pour recuperer ligne par ligne
        allsByLign = alls.split("\n")
        #lecture ligne 1, nombre de features
        self.numfeats = int((allsByLign[0].split(":"))[1])
        #lecture ligne 2, choix d'une simulation avec ou sans bruit moteur
        self.noise = (allsByLign[1].split(":"))[1]
        #lecture ligne 3, choix de la valeur de k pour le bruit sur U
        self.knoiseU = float((allsByLign[2].split(":"))[1])
        #lecture ligne 4, choix du Parametre gamma cost function
        self.gammaCF = float((allsByLign[3].split(":"))[1])
        #lecture ligne 5, choix du Parametre rho cost function
        self.rhoCF = int((allsByLign[4].split(":"))[1])
        #lecture ligne 6, choix du Parametre upsilon cost function
        self.upsCF = float((allsByLign[5].split(":"))[1])
        #lecture ligne 7, Pour CMAES, sigma
        self.sigmaCmaes = float((allsByLign[6].split(":"))[1])
        #lecture ligne 8, Pour CMAES, maxIteration
        self.maxIterCmaes = int((allsByLign[7].split(":"))[1])
        #lecture ligne 9, POUR CMAES, popsize
        self.popsizeCmaes = int((allsByLign[8].split(":"))[1])
        #lecture ligne 10, Taille de la cible pour l'experimentation
        self.sizeOfTarget = [float(allsByLign[9].split(":")[1].split("/")[0]), float(allsByLign[9].split(":")[1].split("/")[1]), float(allsByLign[9].split(":")[1].split("/")[2]), float(allsByLign[9].split(":")[1].split("/")[3])]
        #lecture ligne 11, abscisse de la cible
        self.XTarget = float((allsByLign[10].split(":"))[1])
        #lecture ligne 12, ordonnee de la cible
        self.YTarget = float((allsByLign[11].split(":"))[1])
        #lecture ligne 13, Pas de temps utilise pour l'experimentation
        self.dt = float((allsByLign[12].split(":"))[1])
        #reading line 14, initiales positions
        self.experimentFilePosIni = (allsByLign[13].split(":"))[1]
        #reading line 15, number of iteration to stop unresolved trajectory
        self.numMaxIter = int((allsByLign[14].split(":"))[1])
        #reading line 16, final position error
        self.errorPosEnd = float((allsByLign[15].split(":"))[1])
        #reading line 17, Delai utilise pour le filtre de kalman(int)
        self.delayUKF = int((allsByLign[16].split(":"))[1])
        #reading line 18, Nombre de repetition pour chaque trajectoire(int)
        self.numberOfRepeatEachTraj = int((allsByLign[17].split(":"))[1])
        #reading line 19, Dimension de l'entree, ici le vecteur position(int)
        self.inputDim = int((allsByLign[18].split(":"))[1])
        #reading line 20, Dimension de la sortie, ici le vecteur d'activation musculaire
        self.outputDim = int((allsByLign[19].split(":"))[1])
        #reading line 21, lamda, regularization factor in RBFNs
        self.lamb = float((allsByLign[20].split(":"))[1])

        self.RBFNpath = pathDataFolder + "RBFN/" + str(self.numfeats) + "feats/"
        self.CMAESpath = pathDataFolder + cmaesPath + "/ResCma"

    def getDistanceToTarget(self, x, y):
        return math.sqrt((x - self.XTarget)**2 + (y - self.YTarget)**2)

           
        

