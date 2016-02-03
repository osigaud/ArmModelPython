import os

import sys
sys.path.append(os.getcwd())
sys.path.append(os.getcwd() + "/Script")

from testAllCython import launchCMAESForAllTargetSize

#os.system("python setup.py build_ext --implace")
#os.system("clear")

launchCMAESForAllTargetSize()
