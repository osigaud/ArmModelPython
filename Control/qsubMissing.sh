for gamma in range(3,10)
do
	qsub -v GAMMA=$gamma ScriptClusterMissing.sh
done
