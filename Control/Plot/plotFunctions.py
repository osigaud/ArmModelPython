#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Thomas Beucher

Module: plotFunctions

Description: some plotting functions
'''
import random as rd
import numpy as np
from scipy import stats

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib import animation
from matplotlib.mlab import griddata
import time
 


from Utils.FileReading import getStateData, getEstimatedXYHandData, getXYHandData, getXYEstimError, getXYEstimErrorOfSpeed, getXYElbowData, getNoiselessCommandData, getInitPos, getCostData,  getTrajTimeData, getLastXData


from ArmModel.ArmType import ArmType
from GlobalVariables import BrentTrajectoriesFolder, pathDataFolder
from Experiments.TrajMaker import TrajMaker
plt.rc("figure", facecolor="white")
#TODO: remove GlobalVariables

#--------------------------- trajectory animations ---------------------------------------------------------------------------------------------

def trajectoriesAnimation(what, rs,foldername = "None", targetSize = "0.05"):
    if what == "OPTI":
        name = rs.OPTIpath + targetSize + "/" + foldername + "/Log/"
    elif what == "Brent":
        name = BrentTrajectoriesFolder
    else:
        name = rs.path + foldername + "/Log/"

    ec = getXYElbowData(name)
    hc = getXYHandData(name)
    
    posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
    
    xEl, yEl, xHa, yHa = [], [], [], []
    for key, val in ec.items():
        for el in val:
            xEl.append(el[0])
            yEl.append(el[1])
        for elhc in hc[key]:
            xHa.append(elhc[0])
            yHa.append(elhc[1])
    
    fig = plt.figure()
    upperArm, = plt.plot([],[]) 
    foreArm, = plt.plot([],[])
    plt.xlim(-0.7, 0.7)
    plt.ylim(-0.7,0.7)
    plt.plot([-0.7,0.7], [rs.YTarget, rs.YTarget])
    plt.scatter([-rs.sizeOfTarget[3]/2, rs.sizeOfTarget[3]/2], [rs.YTarget, rs.YTarget], c ='g', marker='o', s=50)
    plt.scatter([el[0] for el in posIni],[el[1] for el in posIni], c='b')
    
    def init():
        upperArm.set_data([0], [0])
        foreArm.set_data([xEl[0]], [yEl[0]])
        return upperArm, foreArm
    
    def animate(i):
        xe = (0, xEl[i])
        ye = (0, yEl[i])
        xh = (xEl[i], xHa[i])
        yh = (yEl[i], yHa[i])
        upperArm.set_data(xe, ye)
        foreArm.set_data(xh, yh)
        return upperArm, foreArm
    
    animation.FuncAnimation(fig, animate, init_func=init, frames=len(xEl), blit=True, interval=20, repeat=True)
    plt.show(block = True)

#----------------------------------------------------------------------------------------------------------------------------
#Functions related to plotting initial positions

def makeInitPlot(rs,filename):
    x0 = []
    y0 = []
    #posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
    posIni = np.loadtxt(pathDataFolder + filename)
    for el in posIni:
        x0.append(el[0])
        y0.append(el[1])
        #print "distance to target: " + str(rs.getDistanceToTarget(el[0],el[1]))

    #xy = getInitPos(BrentTrajectoriesFolder)
    xy = getInitPos(pathDataFolder+"TrajRepository/")
    x, y = [], []
    for _, el in xy.items():
        x.append(el[0])
        y.append(el[1])
        
    plt.scatter(x, y, c = "b", marker=u'o', s=10, cmap=cm.get_cmap('RdYlBu'))
    plt.scatter(rs.XTarget, rs.YTarget, c = "r", marker=u'*', s = 100)
    plt.scatter(x0, y0, c = "r", marker=u'o', s=25)  

def plotInitPos(filename, rs):
    '''
    Plots the initial position of trajectories present in the Brent directory
    '''
    plt.figure()
    makeInitPlot(rs,filename)
    
    plt.show(block = True)

#----------------------------------------------------------------------------------------------------------------------------
#Functions related to velocity profiles

def makeVelocityData(rs,arm,name,media):
    state = getStateData(name)
    factor = min(1, 100./len(state.items()))
    for _,v in state.items():
        index, speed = [], []
        if  rd.random()<factor:
            handxy = arm.mgdEndEffector([v[0][2],v[0][3]])
            distance = round(rs.getDistanceToTarget(handxy[0],handxy[1]),2)
            for j in range(len(v)):
                index.append(j*rs.dt)
                speed.append(arm.cartesianSpeed(v[j]))
            if distance<=0.15:
                media.plot(index, speed, c ='blue')
            elif distance<=0.28:
                media.plot(index, speed, c ='green')
            else:
                media.plot(index, speed, c ='red')

def plotVelocityProfile(what, rs, foldername = "None"):
    arm = ArmType[rs.arm]()
    plt.figure(1, figsize=(16,9))

    if what == "OPTI":
        for i in range(4):
            ax = plt.subplot2grid((2,2), (i/2,i%2))
            name =  rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/" + foldername + "/Log/"
            makeVelocityData(rs,arm,name,ax)
            ax.set_xlabel("time (s)")
            ax.set_ylabel("Instantaneous velocity (m/s)")
            ax.set_title(str("Velocity profiles for target " + str(rs.sizeOfTarget[i])))
    else:
        if what == "Brent":
            name = BrentTrajectoriesFolder
        else:
            name = rs.path + foldername + "/Log/"

        makeVelocityData(rs,arm,name,plt)
        plt.xlabel("time (s)")
        plt.ylabel("Instantaneous velocity (m/s)")
        plt.title("Velocity profiles for " + what)

    plt.savefig("ImageBank/"+what+'_velocity_profiles'+foldername+'.png', bbox_inches='tight')
    plt.show(block = True)


# ------------------------- positions, trajectories ---------------------------------
# We only draw 100 trajectories
def plotPos(name, media, plotEstim):

    states = getXYHandData(name, 100)
    print(len(states.items()))
    for _,v in states.items():        
        posX, posY = [], []
        for j in range(len(v)):
            posX.append(v[j][0])
            posY.append(v[j][1])
        media.plot(posX,posY, c ='b')

    if plotEstim==True:
        estimStates = getEstimatedXYHandData(name, 100)
        for _,v in estimStates.items():
            eX, eY = [], []
            for j in range(len(v)):
                eX.append(v[j][0])
                eY.append(v[j][1])
            media.plot(eX,eY, c ='r')

def plotRegBrent(trajReg, trajBrent):
    """
    plot Brent traj in blue and associated lerned traj in red
    input:            -trajReg: array of trajectory learned
                      -trajBrent: array of Brent trajectory
    """
    plt.figure(1, figsize=(16,9))
    arm = ArmType["Arm26"]()
    for i in range(trajBrent.shape[0]):
        ligneReg=np.empty((trajReg[i].shape[0],2))
        for j in range(trajReg[i].shape[0]):
            ligneReg[j] = arm.mgdEndEffector(trajReg[i][j][2:])
        plt.plot(ligneReg[:,0], ligneReg[:,1], c='r', label="regression")
        ligneBrent=np.empty((len(trajBrent[i]),2))
        for j in range(len(trajBrent[i])):
            ligneBrent[j] = arm.mgdEndEffector(trajBrent[i][j][2:])
        plt.plot(ligneBrent[:,0], ligneBrent[:,1], c='b')
    plt.show(block = True)
    

def plotEstimError(rs,name, media):
    errors = getXYEstimError(name)
    factor = min(1, 100./len(errors.items()))

    for _,v in errors.items():
        if  rd.random()<factor:
            index, er = [], []
            for j in range(len(v)):
#            for j in range(20):
#                index.append(j*rs.dt)
                index.append(j)
                er.append(v[j])
            media.plot(index,er, c ='b')

def plotEstimErrorOfSpeed(name, media):
    errors = getXYEstimErrorOfSpeed(name)
    factor = min(1, 100./len(errors.items()))

    for _,v in errors.items():
        if  rd.random()<factor:
            speed, er = [], []
            for j in range(len(v)):
                speed.append(v[j][0])
                er.append(v[j][1])
            media.plot(speed,er, c ='b')

def plotTrajsInRepo():
    plt.figure(1, figsize=(16,9))
    plotPos(pathDataFolder+"TrajRepository/", plt, False)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("XY Positions")

    plt.savefig("ImageBank/TrajRepo.png", bbox_inches='tight')
    #plt.savefig("ImageBank/"+what+'_trajectories.png')
    plt.show(block = True)

def plotXYPositions(what, rs, foldername = "None", targetSize = "All", plotEstim=False):
    #plt.ion()
    plt.figure(1, figsize=(16,9))
    if (what == "OPTI")  and targetSize == "All":
        for i in range(len(rs.sizeOfTarget)):
            ax = plt.subplot2grid((2,2), (i/2,i%2))
            name =  rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/" + foldername + "/Log/"
            ax.plot([rs.XTarget-rs.sizeOfTarget[i]/2, rs.XTarget+rs.sizeOfTarget[i]/2], [rs.YTarget, rs.YTarget], color="r", linewidth=4.0)
            plotPos(name, ax, plotEstim)

            #makeInitPlot(rs)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_title("XY Positions for target " + str(rs.sizeOfTarget[i]))

    else:
        plt.plot([rs.XTarget-float(targetSize)/2, rs.XTarget+float(targetSize)/2], [rs.YTarget, rs.YTarget], color="r", linewidth=4.0)
        if (what == "OPTI"):
            name = rs.OPTIpath + targetSize + "/" + foldername + "/Log/"
        elif what == "Brent":
            name = BrentTrajectoriesFolder
        else:
            name = rs.path + foldername + "/Log/"
            print(name)

        plotPos(name, plt, plotEstim)
        #makeInitPlot(rs)

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("XY Positions for " + what)
    plt.savefig("ImageBank/"+what+'_trajectories'+rs.thetaFile+time.strftime('%d\%m-%H:%M:%S',time.localtime())+ '.png', bbox_inches='tight')
    plt.show(block = True)

def plotXYEstimError(what, rs,foldername = "None", targetSize = "All"):
    plt.figure(1, figsize=(16,9))

    if what == "OPTI" and targetSize == "All":
        for i in range(len(rs.sizeOfTarget)):
            ax = plt.subplot2grid((2,2), (i/2,i%2))
            name =  rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/" + foldername + "/Log/"
            plotEstimError(rs,name, ax)

            #makeInitPlot(rs)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Estimation error (m)")
            ax.set_title("Estimation error for target " + str(rs.sizeOfTarget[i]))

    else:
        if what == "OPTI":
            name = rs.OPTIpath + targetSize + "/" + foldername + "/Log/"
        elif what == "Brent":
            name = BrentTrajectoriesFolder
        else:
            name = rs.path + foldername + "/Log/"

        plotEstimError(rs,name, plt)
        #makeInitPlot(rs)

        plt.xlabel("Time (s)")
        plt.ylabel("Estimation error (m)")
        plt.title("Estimation error Positions for " + what)

    plt.savefig("ImageBank/"+what+'_estimError'+foldername+'.png', bbox_inches='tight')
    plt.show(block = True)

def plotXYEstimErrorOfSpeed(what, rs,foldername = "None", targetSize = "All"):
    plt.figure(1, figsize=(16,9))

    if what == "OPTI" and targetSize == "All":
        for i in range(len(rs.sizeOfTarget)):
            ax = plt.subplot2grid((2,2), (i/2,i%2))
            name =  rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/" + foldername + "/Log/"
            plotEstimErrorOfSpeed(name, ax)

            #makeInitPlot(rs)
            ax.set_xlabel("Velocity (m/s)")
            ax.set_ylabel("Estimation error (m)")
            ax.set_title("Estimation error function of velocity for target " + str(rs.sizeOfTarget[i]))

    else:
        if what == "OPTI":
            name = rs.OPTIpath + targetSize + "/" + foldername + "/Log/"
        elif what == "Brent":
            name = BrentTrajectoriesFolder
        else:
            name = rs.path + foldername + "/Log/"

        plotEstimErrorOfSpeed(name, plt)
        #makeInitPlot(rs)

        plt.xlabel("Velocity (m/s)")
        plt.ylabel("Estimation error (m)")
        plt.title("Estimation error function of velocity for " + what)

    plt.savefig("ImageBank/"+what+'_estimError'+foldername+'.png', bbox_inches='tight')
    plt.show(block = True)

def plotArticularPositions(what, rs,foldername = "None"):
 
    if what == "OPTI":
        for i in range(len(rs.sizeOfTarget)):
            ax = plt.subplot2grid((2,2), (i/2,i%2))
            name =  rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/" + foldername + "/Log/"
            state = getStateData(name)
            for _,v in state.items():
                Q1, Q2 = [], []
                for j in range(len(v)):
                    Q1.append(v[j][2])
                    Q2.append(v[j][3])
                ax.plot(Q1,Q2, c ='b')
            ax.set_xlabel("Q1 (rad)")
            ax.set_ylabel("Q2 (rad)")
            ax.set_title("Articular positions for " + str(rs.sizeOfTarget[i]))
    else :
        if what == "Brent":
            name = BrentTrajectoriesFolder
        else:
            name = rs.path + foldername + "/Log/"
    
        state = getStateData(name)
    
        plt.figure(1, figsize=(16,9))
        for _,v in state.items():
            if rd.random()<0.06 or what != "Brent":
                Q1, Q2 = [], []
                for j in range(len(v)):
                    Q1.append(v[j][2])
                    Q2.append(v[j][3])
                plt.plot(Q1,Q2, c ='b')
        plt.xlabel("Q1 (rad)")
        plt.ylabel("Q2 (rad)")
        plt.title("Articular positions for " + what)
        
    plt.savefig("ImageBank/"+what+'_articular'+foldername+'.png', bbox_inches='tight')
    plt.show(block = True)

#------------------ muscular activations --------------------------------

def plotMuscularActivations(what, rs, foldername = "None", targetSize = "0.05"):
    '''
    plots the muscular activations from a folder
    
    input:    -foldername: the folder where the data lies
              -what: get from Brent, rbfn or from cmaes controllers

    '''
    if what == "OPTI":
        name = rs.OPTIpath + targetSize + "/" + foldername + "/Log/"
    elif what == "Brent":
        name = BrentTrajectoriesFolder
    else:
        name = rs.path + foldername + "/Log/"

    U = getNoiselessCommandData(name)

    for key, el1 in U.items():
        t = []
        u1, u2, u3, u4, u5, u6 = [], [], [], [], [], []
        if rd.random()<0.01 or what != "Brent":
            for i in range(len(el1)):
                t.append(i)
                u1.append(el1[i][0])
                u2.append(el1[i][1])
                u3.append(el1[i][2])
                u4.append(el1[i][3])
                u5.append(el1[i][4])
                u6.append(el1[i][5])

            plt.figure()
            plt.plot(t, u1, label = "U1")
            plt.plot(t, u2, label = "U2")
            plt.plot(t, u3, label = "U3")
            plt.plot(t, u4, label = "U4")
            plt.plot(t, u5, label = "U5")
            plt.plot(t, u6, label = "U6")
            plt.legend(loc = 0)
            plt.xlabel("time")
            plt.ylabel("U")
            plt.title("Muscular Activations for " + what)
            plt.savefig("ImageBank/"+what+"_muscu" + key +foldername + ".png", bbox_inches='tight')

            print key
            val = raw_input('1 to see data, anything otherwise: ')
            val = int(val)
            if val == 1:
                print el1
            #plt.clf()

    plt.show(block = True)

#-------------------------- cost maps ----------------------------------------------

def plotCostColorMap(what, rs, foldername = "None", targetSize = "All"):
    '''
    Cette fonction permet d'afficher le profil de cout des trajectoires
    
    Entrees:  -what: choix des donnees a afficher
    '''
    fig = plt.figure(1, figsize=(16,9))

    if what == "OPTI" and targetSize == "All":
        for i in range(len(rs.sizeOfTarget)):
            ax = plt.subplot2grid((2,2), (i/2,i%2))
            name =  rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/" + foldername + "/Cost/"
            costs = getCostData(name)

            x0 = []
            y0 = []
            cost = []

            for _, v in costs.items():
                for j in range(len(v)):
                    x0.append(v[j][0])
                    y0.append(v[j][1])
                    cost.append(v[j][2])

            xi = np.linspace(-0.4,0.4,100)
            yi = np.linspace(0.12,0.58,100)
            zi = griddata(x0, y0, cost, xi, yi)

            t1 = ax.scatter(x0, y0, c=cost, marker=u'o', s=5, cmap=cm.get_cmap('RdYlBu'))
            ax.scatter(rs.XTarget, rs.YTarget, c ='g', marker='v', s=200)
            ax.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
            fig.colorbar(t1, shrink=0.5, aspect=5)
            t1 = ax.scatter(x0, y0, c='b', marker=u'o', s=20)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_title(str("Cost map for target " + str(rs.sizeOfTarget[i])))

    else:
        if what == "OPTI":
            name = rs.OPTIpath + targetSize + "/" + foldername + "/Cost/"
        elif what == "Brent":
            name = BrentTrajectoriesFolder
        else:
            name = rs.path + foldername + "/Cost/"

        costs = getCostData(name)
   
        x0 = []
        y0 = []
        cost = []

        for _, v in costs.items():
            for j in range(len(v)):
                x0.append(v[j][0])
                y0.append(v[j][1])
                cost.append(v[j][2])

        xi = np.linspace(-0.4,0.4,100)
        yi = np.linspace(0.12,0.58,100)
        zi = griddata(x0, y0, cost, xi, yi)
    
        t1 = plt.scatter(x0, y0, c=cost, marker=u'o', s=5, cmap=cm.get_cmap('RdYlBu'))
        plt.scatter(rs.XTarget, rs.YTarget, c ='g', marker='v', s=200)
        plt.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
        fig.colorbar(t1, shrink=0.5, aspect=5)
        t1 = plt.scatter(x0, y0, c='b', marker=u'o', s=20)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("Cost map for " + what)

    plt.savefig("ImageBank/"+what+'_costmap'+foldername+'.png', bbox_inches='tight')
    plt.show(block = True)

#-------------------------- time maps ----------------------------------------------

def plotTimeColorMap(what, rs, foldername = "None", targetSize = "All"):
    '''
    Cette fonction permet d'afficher le profil de temps des trajectoires
    
    Entrees:      -what: choix des donnees a afficher
    '''
    fig = plt.figure(1, figsize=(16,9))

    if what == "OPTI" and targetSize == "All":
        for i in range(len(rs.sizeOfTarget)):
            ax = plt.subplot2grid((2,2), (i/2,i%2))
            name =  rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/" + foldername + "/TrajTime/"
            times = getTrajTimeData(name)

            x0 = []
            y0 = []
            time = []

            for _, v in times.items():
                for j in range(len(v)):
                    x0.append(v[j][0])
                    y0.append(v[j][1])
                    time.append(v[j][2])

            xi = np.linspace(-0.4,0.4,100)
            yi = np.linspace(0.12,0.58,100)
            zi = griddata(x0, y0, time, xi, yi)

            t1 = ax.scatter(x0, y0, c=time, marker=u'o', s=50, cmap=cm.get_cmap('RdYlBu'))
            ax.scatter(rs.XTarget, rs.YTarget, c ='g', marker='v', s=200)
            ax.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_title(str("Time map for target " + str(rs.sizeOfTarget[i])))
            fig.colorbar(t1, shrink=0.5, aspect=5)
            t1 = ax.scatter(x0, y0, c='b', marker=u'o', s=20)

    else:
        if what == "OPTI":
            name = rs.OPTIpath + targetSize + "/" + foldername + "/TrajTime/"
        elif what == "Brent":
            name = BrentTrajectoriesFolder
        else:
            name = rs.path + foldername + "/TrajTime/"

        times = getTrajTimeData(name)
   
        x0 = []
        y0 = []
        time = []

        for _, v in times.items():
            for j in range(len(v)):
                x0.append(v[j][0])
                y0.append(v[j][1])
                time.append(v[j][2])

        xi = np.linspace(-0.4,0.4,100)
        yi = np.linspace(0.12,0.58,100)
        zi = griddata(x0, y0, time, xi, yi)
    
        t1 = plt.scatter(x0, y0, c=time, marker=u'o', s=50, cmap=cm.get_cmap('RdYlBu'))
        plt.scatter(rs.XTarget, rs.YTarget, c ='g', marker='v', s=200)
        plt.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
        fig.colorbar(t1, shrink=0.5, aspect=5)
        plt.scatter(x0, y0, c='b', marker=u'o', s=20)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")

    plt.savefig("ImageBank/"+what+'_timemap'+foldername+'.png', bbox_inches='tight')
    plt.show(block = True)

#-----------------------------------------------------------------------------------------------------------
    
def plotTimeDistanceTarget(foldername,rs):

    dicoTime = {}
 
    for i in range(len(rs.sizeOfTarget)):
        name =  rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/" + foldername + "/TrajTime/"

        trajTimes = getTrajTimeData(name)

        for _, v in trajTimes.items():
            for j in range(len(v)):
                distance = round(rs.getDistanceToTarget(v[j][0],v[j][1]),2)
                if not distance in dicoTime.keys():
                    dicoTime[distance] = {}
                if not rs.sizeOfTarget[i] in dicoTime[distance].keys():
                    dicoTime[distance][rs.sizeOfTarget[i]] = []
                dicoTime[distance][rs.sizeOfTarget[i]].append(v[j][2])
 
    plotTab = []

    plt.figure(1, figsize=(16,9))
    plt.ylabel("time (s)")
    plt.xlabel("Target size (mm)")
    for key in sorted(dicoTime.keys()):
        plotTab.append(plt.plot([i for i in sorted(dicoTime[key].keys())], [np.mean(dicoTime[key][i]) for i in sorted(dicoTime[key].keys())], label = str("Distance: " + str(key))))
    plt.legend(loc = 0)
    plt.savefig("ImageBank/timedist"+foldername+'.png', bbox_inches='tight')
    plt.show(block = True)

#-----------------------------------------------------------------------------------------------------------
    
def plotPerfSizeDist(foldername, rs):
    dicoCost = {}
 
    for i in range(len(rs.sizeOfTarget)):
        name =  rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/" + foldername + "/Cost/"

        costs = getCostData(name)

        for _, v in costs.items():
            for j in range(len(v)):
                distance = round(rs.getDistanceToTarget(v[j][0],v[j][1]),2)
                if not distance in dicoCost.keys():
                    dicoCost[distance] = {}
                if not rs.sizeOfTarget[i] in dicoCost[distance].keys():
                    dicoCost[distance][rs.sizeOfTarget[i]] = []
                dicoCost[distance][rs.sizeOfTarget[i]].append(v[j][2])

    plotTab = []
    plt.figure(1, figsize=(16,9))
    plt.ylabel("performance")
    plt.xlabel("Target size (mm)")
    for key in sorted(dicoCost.keys()):
        plotTab.append(plt.plot([i for i in sorted(dicoCost[key].keys())], [np.mean(dicoCost[key][i]) for i in sorted(dicoCost[key].keys())], label = str("Distance: " + str(key))))
    plt.legend(loc = 0)
    plt.savefig("ImageBank/perfdist"+foldername+".png", bbox_inches='tight')
    plt.show(block = True)

#-----------------------------------------------------------------------------------------------------------
            
def plotFittsLaw(foldername, rs, rbfn = False):

    timeDistWidth = []
    for i in range(len(rs.sizeOfTarget)):
        name =  rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/" + foldername + "/TrajTime/"

        trajTimes = getTrajTimeData(name)

        for _, v in trajTimes.items():
            for j in range(len(v)):
                distance = rs.getDistanceToTarget(v[j][0],v[j][1])
                trajtime = v[j][2]
                size = rs.sizeOfTarget[i]
                timeDistWidth.append((distance, size, trajtime))
            
    MT, DI = [], []
    for el in timeDistWidth:
        MT.append(el[2])
        DI.append(np.log2(el[0]/el[1]))
    slope, intercept, r_value, _, _ = stats.linregress(DI,MT)
    yLR = slope * np.asarray(DI) + intercept
    plt.figure()

    for el in timeDistWidth:
            if el[0]<=0.15:
                plt.scatter(np.log2(el[0]/el[1]), el[2], c ='blue')
            elif el[0]<=0.28:
                plt.scatter(np.log2(el[0]/el[1]), el[2], c ='green')
            else:
                plt.scatter(np.log2(el[0]/el[1]), el[2], c ='red')
        
    plt.plot(DI, yLR)
    plt.title("a = " + str(slope) + " b = " + str(intercept) + " r^2 = " + str(r_value**2))
    plt.xlabel("log(D/W)/log(2)")
    plt.ylabel("Movement time (s)")
    plt.savefig("ImageBank/fitts"+foldername+".png", bbox_inches='tight')
    plt.show(block = True)
 
# ---------------- hit dispersion ---------------------------------------

def plotHitDispersion(foldername,sizeT, rs):
    name =  rs.OPTIpath + sizeT + "/" + foldername + "/finalX/"
    data = getLastXData(name)

    tabx, taby = [], []
    for el in data.values():
        for j in range(len(el)):
            tabx.append(el[j])
            taby.append(rs.YTarget)

    plt.figure(1, figsize=(16,9))
    plt.plot([-rs.sizeOfTarget[0]/2, rs.sizeOfTarget[0]/2], [rs.YTarget, rs.YTarget], c = 'r')
    plt.scatter([-rs.sizeOfTarget[0]/2, rs.sizeOfTarget[0]/2], [rs.YTarget, rs.YTarget], marker=u'|', s = 100)
    plt.scatter(tabx, taby, c = 'b')
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.savefig("ImageBank/hit" + str(sizeT) +foldername + ".png", bbox_inches='tight')
    plt.show(block = True)

def plotScattergram(what,foldername,rs):
    data = {}

    if what=="OPTI":
        for i in range(len(rs.sizeOfTarget)):
            name =  rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/" + foldername + "/finalX/"
            tmp = getLastXData(name)
            tabx = []
            for el in tmp.values():
                for j in range(len(el)):
                    tabx.append(el[j])

                    data[rs.sizeOfTarget[i]] = tabx

        plt.figure(1, figsize=(16,9))

        for i in range(len(rs.sizeOfTarget)):
            ax = plt.subplot2grid((2,2), (i/2,i%2))
            ax.hist(data[rs.sizeOfTarget[i]], 20)
            ax.plot([-rs.sizeOfTarget[i]/2, -rs.sizeOfTarget[i]/2], [0, 500], c = 'r', linewidth = 3)
            ax.plot([rs.sizeOfTarget[i]/2, rs.sizeOfTarget[i]/2], [0, 500], c = 'r', linewidth = 3)
            ax.set_title(str("Hit Dispersion for Target " + str(rs.sizeOfTarget[i])))

    elif what=="RBFN":
            name =  rs.path + foldername + "/finalX/"
            tmp = getLastXData(name)
            tabx = []
            for el in tmp.values():
                for j in range(len(el)):
                    tabx.append(el[j])
            plt.hist(tabx, 20)
            for i in range(len(rs.sizeOfTarget)):
                plt.plot([-rs.sizeOfTarget[i]/2, -rs.sizeOfTarget[i]]/2, [0, 20], c = 'r', linewidth = 3)
                plt.plot([rs.sizeOfTarget[i]/2, rs.sizeOfTarget[i]]/2, [0, 20], c = 'r', linewidth = 3)
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.title("Hit Dispersion for "+rs.regression)
    
    plt.savefig("ImageBank/"+what+"_hitdisp"+foldername+".png", bbox_inches='tight')
    plt.show(block = True)
        
# ---------------- end of hit dispersion ---------------------------------------

def plotCMAESProgress(rs):
    plotCMAESCostProgress(rs)
    plotCMAESTimeProgress(rs)

def plotCMAESCostProgress(rs):
    plt.figure(1, figsize=(16,9))

    for i in range(len(rs.sizeOfTarget)):
        ax = plt.subplot2grid((2,2), (i/2,i%2))
        name = rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/Cost/cmaesCost.log"
        data = np.loadtxt(name)

        x,w,m,b = [],[],[],[]
        for j in range(len(data)):
            x.append(j)
            w.append(data[j][0])
            m.append(data[j][1])
            b.append(data[j][2])
        ax.plot(x, w, c = 'b')
        ax.plot(x, m, c = 'g')
        ax.plot(x, b, c = 'r')

        ax.set_title(str("Cost Target " + str(rs.sizeOfTarget[i])))

    plt.savefig("ImageBank/"+rs.thetaFile+"costProgress.png")
    plt.show(block = True)

def plotCMAESTimeProgress(rs):
    plt.figure(1, figsize=(16,9))

    for i in range(len(rs.sizeOfTarget)):
        ax = plt.subplot2grid((2,2), (i/2,i%2))

        name = rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/Cost/cmaesTime.log"
        data = np.loadtxt(name)

        x,w,m,b = [],[],[],[]
        for j in range(len(data)):
            x.append(j)
            w.append(data[j][0])
            m.append(data[j][1])
            b.append(data[j][2])
        ax.plot(x, w, c = 'b')
        ax.plot(x, m, c = 'g')
        ax.plot(x, b, c = 'r')

        ax.set_title(str("Time Target " + str(rs.sizeOfTarget[i])))

    plt.savefig("ImageBank/timeProgress.png")
    plt.show(block = True)
    
    
def plotCMAESOnePointProgress(rs, ts, point):
    plotCMAESOnePointCostProgress(rs, ts, point)
    plotCMAESOnePointTimeProgress(rs, ts, point)

def plotCMAESOnePointCostProgress(rs, ts, point):
    plt.figure(1, figsize=(16,9))


    name = rs.OPTIpath + str(ts)+"/"+str(point)+ "/Cost/cmaesCost.log"
    data = np.loadtxt(name)

    x,w,m,b = [],[],[],[]
    for j in range(len(data)):
        x.append(j)
        w.append(data[j][0])
        m.append(data[j][1])
        b.append(data[j][2])
    plt.plot(x, w, c = 'b')
    plt.plot(x, m, c = 'g')
    plt.plot(x, b, c = 'r')



    plt.show(block = True)

def plotCMAESOnePointTimeProgress(rs, ts, point):
    plt.figure(1, figsize=(16,9))

    name = rs.OPTIpath + str(ts)+"/"+str(point) + "/Cost/cmaesTime.log"
    data = np.loadtxt(name)

    x,w,m,b = [],[],[],[]
    for j in range(len(data)):
        x.append(j)
        w.append(data[j][0])
        m.append(data[j][1])
        b.append(data[j][2])
    plt.plot(x, w, c = 'b')
    plt.plot(x, m, c = 'g')
    plt.plot(x, b, c = 'r')

    
    plt.show(block = True)
    
    
    
def plotDDPGProgress(rs):
    plotDDPGCostProgress(rs)
    plotDDPGTimeProgress(rs)

def plotDDPGCostProgress(rs):
    plt.figure(1, figsize=(16,9))

    for i in range(len(rs.sizeOfTarget)):
        ax = plt.subplot2grid((2,2), (i/2,i%2))
        name = rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/Cost/ddpgCost.log"
        nameProgress = rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/Cost/ddpgProg.log"
        data = np.loadtxt(name)
        #progress = np.loadtxt(nameProgress)

        x,w = [],[]
        for j in range(len(data)):
            x.append(j)
            w.append(data[j])
        ax.plot(x, w, c = 'b')
        """
        for j in progress.shape[0]:
            ax.plot(progress[j],np.random.uniform(-0.8,1.2,100),'r')
        """
        ax.set_title(str("Cost Target " + str(rs.sizeOfTarget[i])))

    plt.savefig("ImageBank/"+rs.thetaFile+"DDPGcostProgress.png")
    plt.show(block = True)

def plotDDPGTimeProgress(rs):
    plt.figure(1, figsize=(16,9))

    for i in range(len(rs.sizeOfTarget)):
        ax = plt.subplot2grid((2,2), (i/2,i%2))

        name = rs.OPTIpath + str(rs.sizeOfTarget[i]) + "/Cost/ddpgTime.log"
        data = np.loadtxt(name)

        x,w = [],[]
        for j in range(len(data)):
            x.append(j)
            w.append(data[j])
        ax.plot(x, w, c = 'b')

        ax.set_title(str("DDPG Time Target " + str(rs.sizeOfTarget[i])))

    plt.savefig("ImageBank/DDPGTimeProgress.png")
    plt.show(block = True)

    

def plotExperimentSetup(rs):
    plt.figure(1, figsize=(16,9))
    arm = ArmType[rs.arm]()
    q1 = np.linspace(-0.6, 2.6, 100, True)
    q2 = np.linspace(-0.2, 3, 100, True)
    posIni = np.loadtxt(pathDataFolder + rs.experimentFilePosIni)
    xi, yi = [], []
    xb, yb = [0], [0]
    t = 0
    for el in posIni:
        if el[1] == np.min(posIni, axis = 0)[1] and t == 0:
            t += 1
            a, b = arm.mgi(el[0], el[1])
            a1, b1 = arm.mgdFull(np.array([[a], [b]]))
            xb.append(a1[0])
            xb.append(b1[0])
            yb.append(a1[1])
            yb.append(b1[1])
        xi.append(el[0])
        yi.append(el[1])
    pos = []
    for i in range(len(q1)):
        for j in range(len(q2)):
            coordHa = arm.mgdEndEffector(np.array([[q1[i]], [q2[j]]]))
            pos.append(coordHa)
    x, y = [], []
    for el in pos:
        x.append(el[0])
        y.append(el[1])

    plt.scatter(x, y)
    plt.scatter(xi, yi, c = 'r')
    plt.scatter(0, 0.6175, c = "r", marker=u'*', s = 200)
    plt.plot(xb, yb, c = 'r')
    plt.plot([-0.3,0.3], [0.6175, 0.6175], c = 'g')
    plt.savefig("ImageBank/setup.png", bbox_inches='tight')
    plt.show(block = True)

#TODO: both functions below can be much improved

def plotManipulability(rs):
    fig = plt.figure(1, figsize=(16,9))
    arm = ArmType[rs.arm]()
    q1 = np.linspace(-0.6, 2.6, 100, True)
    q2 = np.linspace(-0.2, 3, 100, True)
    target = [rs.XTarget, rs.YTarget]

    pos = []
    for i in range(len(q1)):
        for j in range(len(q2)):
            config = np.array([q1[i], q2[j]])
            coordHa = arm.mgdEndEffector(config)
            pos.append(coordHa)

    x, y, cost = [], [], []
    for el in pos:
        x.append(el[0])
        y.append(el[1])
        config = arm.mgi(el[0],el[1])
        manip = arm.directionalManipulability(config,target)
        cost.append(manip)

    xi = np.linspace(-0.7,0.8,100)
    yi = np.linspace(-0.5,0.8,100)
    zi = griddata(x, y, cost, xi, yi)

    #t1 = plt.scatter(x, y, c=cost, marker=u'o', s=5, cmap=cm.get_cmap('RdYlBu'))
    #CS = plt.contourf(xi, xi, zi, 15, cmap=cm.get_cmap('RdYlBu'))

    t1 = plt.scatter(x, y, c=cost, s=5, cmap=cm.get_cmap('RdYlBu'))
    plt.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
    fig.colorbar(t1, shrink=0.5, aspect=5)
    plt.scatter(rs.XTarget, rs.YTarget, c = "g", marker=u'*', s = 200)
    #plt.plot([-0.3,0.3], [rs.YTarget, rs.YTarget], c = 'g')
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title(str("Manipulability map"))
    plt.savefig("ImageBank/manipulability.png", bbox_inches='tight')
    plt.show(block = True)


def plotManipulability2(rs):
    fig = plt.figure(1, figsize=(16,9))
    arm = ArmType[rs.arm]()
    q1 = np.linspace(-0.6, 2.6, 100, True)
    q2 = np.linspace(-0.2, 3, 100, True)
    target = [rs.XTarget, rs.YTarget]

    pos = []
    for i in range(len(q1)):
        for j in range(len(q2)):
            config = np.array([q1[i], q2[j]])
            coordHa = arm.mgdEndEffector(config)
            pos.append(coordHa)

    x, y, cost = [], [], []
    for el in pos:
        x.append(el[0])
        y.append(el[1])
        config = arm.mgi(el[0],el[1])
        manip = arm.manipulability(config,target)
        cost.append(manip)

    xi = np.linspace(-0.7,0.8,100)
    yi = np.linspace(-0.5,0.8,100)
    zi = griddata(x, y, cost, xi, yi)

    #t1 = plt.scatter(x, y, c=cost, marker=u'o', s=5, cmap=cm.get_cmap('RdYlBu'))
    #CS = plt.contourf(xi, xi, zi, 15, cmap=cm.get_cmap('RdYlBu'))

    t1 = plt.scatter(x, y, c=cost, s=5, cmap=cm.get_cmap('RdYlBu'))
    plt.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
    fig.colorbar(t1, shrink=0.5, aspect=5)
    plt.scatter(rs.XTarget, rs.YTarget, c = "g", marker=u'*', s = 200)
    #plt.plot([-0.3,0.3], [rs.YTarget, rs.YTarget], c = 'g')
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title(str("Manipulability map"))
    plt.savefig("ImageBank/manipulability2.png", bbox_inches='tight')
    plt.show(block = True)


def plotEstimator(setupFile, sizeOfTarget, x, y):
    thetaName = setupFile.CMAESpath + str(sizeOfTarget) + "/" + "Best"
    arm = ArmType["Arm26"]()
    
    setupFile.det = True
    tm=TrajMaker(setupFile, sizeOfTarget, False, thetaName, "Inv")
    state1,_,_,_=tm.runTrajectoryForPlot(x, y)
    
    setupFile.det = False
    state2,_,_,_=tm.runTrajectoryForPlot(x, y)
    
    tm=TrajMaker(setupFile, sizeOfTarget, False, thetaName, "No")
    state3,_,_,_=tm.runTrajectoryForPlot(x, y)
    
    plt.figure(1, figsize=(16,9))
    
    hand=[]
    for state in state1 :
        hand.append(arm.mgdEndEffector(state[2:]))
    hand=np.array(hand)
    plt.plot(hand[:,0],hand[:,1], color="blue")
    
    hand=[]
    for state in state2 :
        hand.append(arm.mgdEndEffector(state[2:]))
    hand=np.array(hand)
    plt.plot(hand[:,0],hand[:,1], color="red")
    
    hand=[]
    for state in state3 :
        hand.append(arm.mgdEndEffector(state[2:]))
    hand=np.array(hand)
    plt.plot(hand[:,0],hand[:,1], color="green")
    
    plt.show(block = True)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    