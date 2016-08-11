#!/bin/sh
#PBS -N arm_model$TARGET
#PBS -o arm_model$TARGET.out
#PBS -b arm_model$TARGET.err
#PBS -l walltime=48:00:00
#PBS -l ncpus=15
#PBS -V
#PBS -d /home/sigaud/ArmModelPython/Control/
python clusterOneTargetNController.py setupK10gamma$GAMMA.xml $TARGET
