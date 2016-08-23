#!/bin/sh
#PBS -N missing$GAMMA
#PBS -o missing$GAMMA.out
#PBS -b missing$GAMMA.err
#PBS -l walltime=48:00:00
#PBS -l nodes=1:ppn=16
#PBS -V
#PBS -d /home/sigaud/ArmModelPython/Control/
python runMissing.py setupMissing.xml $GAMMA
