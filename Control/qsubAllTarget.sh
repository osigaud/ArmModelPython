for target in 0.04 0.02 0.01 0.005
do
	qsub -v GAMMA=4 TARGET=$target ScriptClusterOneTargetNController.sh
done
