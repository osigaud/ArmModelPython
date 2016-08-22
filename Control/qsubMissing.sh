for gamma in 3 4 5 6 7 8 9
do
	qsub -v GAMMA=$gamma ScriptClusterMissing.sh
done
