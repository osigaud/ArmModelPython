for target in 0.04 0.02 0.01 0.005
do
	qsub -v TARGET=$target ScriptClusterOneTargetNController.sh
done
