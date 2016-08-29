#!/bin/sh
#PBS -N missingHitVel
#PBS -o missingHitVel.out
#PBS -b missingHitVel.err
#PBS -l walltime=24:00:00
#PBS -l nodes=1:ppn=16
#PBS -V
#PBS -d /home/sigaud/ArmModelPython/Control/
python runMissingHitVel.py setupMissingHitVel.xml
