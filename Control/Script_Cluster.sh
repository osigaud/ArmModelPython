#!/bin/sh
#PBS -N arm_model
#PBS -o arm_model.out
#PBS -b arm_model.err
#PBS -l walltime=00:04:00
#PBS -l ncpus=1
#PBS -V
dir=/home/corentin.arnaud/stage/ArmModelPython/Control
python $dir/cluster.py $dir/setupNN1th.xml
#echo "import pybrain" | python
