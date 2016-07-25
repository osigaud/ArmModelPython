#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Author: Corentin Arnaud


Description: plot data from Babich
'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.mlab import griddata
from os import path
from scipy import stats
from Utils.FileReading import loadExpeTrajs

plt.rc("figure", facecolor="white")
experimentalFolder=path.abspath("../Data/Experimental/")+"/"
imageFolder=path.abspath("../Data/GoodResult/experimental/")+"/"

'''
#---------------------------------------------------------Hit Dispersion -------------------------------------------------------
hits = [np.loadtxt(experimentalFolder+"Hits/hits0.005")[:,1],
        np.loadtxt(experimentalFolder+"Hits/hits0.01")[:,1],
        np.loadtxt(experimentalFolder+"Hits/hits0.02")[:,1],
        np.loadtxt(experimentalFolder+"Hits/hits0.04")[:,1]]


plt.figure(1, figsize=(16,9))
target =0.005
for i in range(4):
    ax = plt.subplot2grid((4,1), (i,0))
    ax.hist(hits[i], 100)
    ax.axvline(x=target/2, c = 'r', linewidth = 3)
    ax.axvline(x=-target/2, c = 'r', linewidth = 3)
    ax.set_title(str("Hit Dispersion for Target " + str(target)))
    ax.set_xlim([-0.03, 0.03])
    target*=2
plt.savefig(imageFolder+"hitDispersion.svg", bbox_inches='tight')      
    


#--------------------------------------------------------Cost Map ---------------------------------------------------------------
posIni = np.loadtxt(path.abspath("../Data/") + "/PosCircu15")
posIni[:3]=posIni[:3][::-1]
posIni[3:8]=posIni[3:8][::-1]
posIni[8:]=posIni[8:][::-1]
x0=posIni[:,0]
y0=posIni[:,1]

fig=plt.figure(2, figsize=(16,9))
cost_emg = np.loadtxt(experimentalFolder+"Cost/D_cost_emg.txt")
target =0.005
for i in range(4):
    ax = plt.subplot2grid((2,2), (i/2,i%2))


    xi = np.linspace(-0.4,0.4,100)
    yi = np.linspace(0.12,0.58,100)

    zi = griddata(x0, y0, cost_emg[:,i], xi, yi)

    t1 = ax.scatter(x0, y0, c=cost_emg[:,i], marker=u'o', s=5, cmap=cm.get_cmap('RdYlBu'))
    ax.scatter(0.0, 0.6175, c ='g', marker='v', s=200)
    ax.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
    fig.colorbar(t1, shrink=0.5, aspect=5)
    t1 = ax.scatter(x0, y0, c='b', marker=u'o', s=20)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(str("Cost calculated from total EMG for target " + str(target)))
    target*=2
plt.savefig(imageFolder+"totalEMG.svg", bbox_inches='tight')  

fig=plt.figure(3, figsize=(16,9))
cost_kin1 = np.loadtxt(experimentalFolder+"Cost/D_cost_kin1.txt")
target =0.005
for i in range(4):
    ax = plt.subplot2grid((2,2), (i/2,i%2))


    xi = np.linspace(-0.4,0.4,100)
    yi = np.linspace(0.12,0.58,100)

    zi = griddata(x0, y0, cost_kin1[:,i], xi, yi)

    t1 = ax.scatter(x0, y0, c=cost_kin1[:,i], marker=u'o', s=5, cmap=cm.get_cmap('RdYlBu'))
    ax.scatter(0.0, 0.6175, c ='g', marker='v', s=200)
    ax.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
    fig.colorbar(t1, shrink=0.5, aspect=5)
    t1 = ax.scatter(x0, y0, c='b', marker=u'o', s=20)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(str("Cost calculated from dynamic model of arm for target " + str(target)))
    target*=2
plt.savefig(imageFolder+"dynamicModel.svg", bbox_inches='tight')   
    
fig=plt.figure(4, figsize=(16,9))
cost_kin2 = np.loadtxt(experimentalFolder+"Cost/D_cost_kin2.txt")
target =0.005
for i in range(4):
    ax = plt.subplot2grid((2,2), (i/2,i%2))


    xi = np.linspace(-0.4,0.4,100)
    yi = np.linspace(0.12,0.58,100)

    zi = griddata(x0, y0, cost_kin2[:,i], xi, yi)

    t1 = ax.scatter(x0, y0, c=cost_kin2[:,i], marker=u'o', s=5, cmap=cm.get_cmap('RdYlBu'))
    ax.scatter(0.0, 0.6175, c ='g', marker='v', s=200)
    ax.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
    fig.colorbar(t1, shrink=0.5, aspect=5)
    t1 = ax.scatter(x0, y0, c='b', marker=u'o', s=20)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(str("Cost calculated from dynamic model of arm and HM for target " + str(target)))
    target*=2
plt.savefig(imageFolder+"dynamicModelAndHM.svg", bbox_inches='tight')    
    
fig=plt.figure(5, figsize=(16,9))
target =0.005
cost=cost_emg-(cost_kin2-cost_kin1)/cost_kin2*cost_emg
for i in range(4):
    ax = plt.subplot2grid((2,2), (i/2,i%2))


    xi = np.linspace(-0.4,0.4,100)
    yi = np.linspace(0.12,0.58,100)

    zi = griddata(x0, y0, cost[:,i], xi, yi)

    t1 = ax.scatter(x0, y0, c=cost[:,i], marker=u'o', s=5, cmap=cm.get_cmap('RdYlBu'))
    ax.scatter(0.0, 0.6175, c ='g', marker='v', s=200)
    ax.contourf(xi, yi, zi, 15, cmap=cm.get_cmap('RdYlBu'))
    fig.colorbar(t1, shrink=0.5, aspect=5)
    t1 = ax.scatter(x0, y0, c='b', marker=u'o', s=20)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(str("Cost calculated EMG with excluded damping for target " + str(target)))
    target*=2
plt.savefig(imageFolder+"EMGExcluededDamping.svg", bbox_inches='tight')

#-------------------------------------------------------Perf/size target plot -----------------------------------------------------    
fig=plt.figure(6, figsize=(16,9))
perf = np.loadtxt(experimentalFolder+"Perf/D_perf.txt")
perf_std = np.loadtxt(experimentalFolder+"Perf/D_perf_std.txt")

for i in range(3):
    plt.errorbar([0.005,0.01,0.02,0.04],perf[i,:],yerr=perf_std[i,:]/np.sqrt(10))
plt.xlabel('target size [m]')
plt.ylabel('average performance [s^{-1}]')
plt.legend(['short', 'mid', 'long'])
plt.savefig(imageFolder+"performance.svg", bbox_inches='tight')



#------------------------------------------------------time/size Target plot--------------------------------------------------------
fig=plt.figure(7, figsize=(16,9))
time = np.loadtxt(experimentalFolder+"Perf/D_time.txt")
time_std = np.loadtxt(experimentalFolder+"Perf/D_time_std.txt")

for i in range(3):
    plt.errorbar([0.005,0.01,0.02,0.04],time[i,:],yerr=time_std[i,:]/np.sqrt(10))
plt.xlabel('target size [m]')
plt.ylabel('average reaching time [s]')
plt.legend(['short', 'mid', 'long'])
plt.savefig(imageFolder+"reaching.svg", bbox_inches='tight')


#------------------------------------------------------Fitts ------------------------------------------------------------------------   
fig=plt.figure(8, figsize=(16,9))
fitts = np.loadtxt(experimentalFolder+"reachingTimeAvg")
dist= np.loadtxt(experimentalFolder+"DistanceToTarget").mean(0)
dist_pos=np.empty(15)
DI=np.zeros(15*4)
MI=np.zeros(15*4)
dist_pos[:3]=dist[0]
dist_pos[3:8]=dist[1]
dist_pos[8:]=dist[2]
target=0.005
for j in range(4):
    for i in range(15):
        DI[15*j+i]=np.log2(dist_pos[i]/target)
        MI[15*j+i]=fitts[i][j]
    target*=2
    

slope, intercept, r_value, _, _ = stats.linregress(DI,MI)
yLR = slope * np.asarray(DI) + intercept
for i in range(15):
    for j in range(4):
        if i<3 :
            plt.scatter(DI[j*15+i],MI[j*15+i],c ='blue')
        elif i<8 :
            plt.scatter(DI[j*15+i],MI[j*15+i],c ='green')
        else :
            plt.scatter(DI[j*15+i],MI[j*15+i],c ='red')
plt.plot(DI,yLR,"b")
plt.title("a = " + str(slope) + " b = " + str(intercept) + " r^2 = " + str(r_value**2))
plt.xlabel("log(D/W)/log(2)")
plt.ylabel("Movement time (s)")
plt.savefig(imageFolder+"Fitts.svg", bbox_inches='tight')
'''
#--------------------------------------------------Traj---------------------------------------------------------------------------------
trajplot=plt.figure(9, figsize=(16,9))
veloplot=plt.figure(11, figsize=(16,9))
zoomplot=plt.figure(10, figsize=(16,9))
target=0.005
who="4"
if who=="*" : num=0.1
else : 
    num=1
    
yCoordinate=np.loadtxt(experimentalFolder+"Traj/targetYCoordinate")
for j in range(4):
    
    time, coor, velo, pos=loadExpeTrajs(experimentalFolder+"Traj/"+str(target)+"/"+who+"/",num)
    trajax = trajplot.add_subplot(2,2,j+1)
    veloax = veloplot.add_subplot(2,2,j+1)
    zoomax = zoomplot.add_subplot(2,2,j+1)
    scale=target*18/16
    zoomax.set_xlim([-target,target])
    if(who=="*"):
        zoomax.set_ylim([yCoordinate.mean()-3*scale/4,yCoordinate.mean()+scale/4])
    else :
        zoomax.set_ylim([yCoordinate[int(who)-1]-3*scale/4,yCoordinate[int(who)-1]+scale/4])
        trajax.plot([-target/2,target/2],[yCoordinate[int(who)-1],yCoordinate[int(who)-1]],c='black',linewidth = 4)
        zoomax.plot([-target/2,target/2],[yCoordinate[int(who)-1],yCoordinate[int(who)-1]],c='black',linewidth = 4)
    for i in range(coor.shape[0]):
        if pos[i] == 1:
            continue
            trajax.plot(coor[i][:,0],coor[i][:,1],c="blue")
            zoomax.plot(coor[i][:,0],coor[i][:,1],c="blue")
            veloax.plot(time[i],np.sqrt(velo[i][:,0]**2+velo[i][:,1]**2),c="blue")
        elif pos[i] ==2:
            continue
            trajax.plot(coor[i][:,0],coor[i][:,1],c="green")
            zoomax.plot(coor[i][:,0],coor[i][:,1],c="green")
            veloax.plot(time[i],np.sqrt(velo[i][:,0]**2+velo[i][:,1]**2),c="green")
        else :
            trajax.plot(coor[i][coor[i][:,1]<0.,0],coor[i][coor[i][:,1]<0.,1],c="red")
            zoomax.plot(coor[i][:,0],coor[i][:,1],c="red")
            veloax.plot(time[i][coor[i][:,1]<0.],np.sqrt(velo[i][:,0]**2+velo[i][:,1]**2)[coor[i][:,1]<0.],c="red")
        #veloax.plot(time[i][1:],np.sqrt((coor[i][1:,1]-coor[i][:-1,1])**2+(coor[i][1:,0]-coor[i][:-1,0])**2)/(time[i][1:]-time[i][:-1]))
        #veloax.plot(range(1,coor[i].shape[0]),np.sqrt((coor[i][1:,1]-coor[i][:-1,1])**2+(coor[i][1:,0]-coor[i][:-1,0])**2))
    
    veloax.set_xlabel("time (s)")
    veloax.set_ylabel("Instantaneous velocity (m/s)")
    veloax.set_title(str("Velocity profiles for target " + str(target)))
    trajax.set_xlabel("X (m)")
    trajax.set_ylabel("Y (m)")
    trajax.set_title("XY Positions for target " + str(target))
    zoomax.set_xlabel("X (m)")
    zoomax.set_ylabel("Y (m)")
    zoomax.set_title("XY Positions for target " + str(target))
    target*=2
if who=="*":who=""
#trajplot.savefig(imageFolder+"trajectories"+who+"_Luka.svg", bbox_inches='tight')
#zoomplot.savefig(imageFolder+"trajectorieszoom"+who+"_Luka.pdf", bbox_inches='tight')
#veloplot.savefig(imageFolder+"velo"+who+".svg", bbox_inches='tight')
plt.show()