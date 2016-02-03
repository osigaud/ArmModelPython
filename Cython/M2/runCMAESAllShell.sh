#!/bin/sh
xterm -e "python runCMAESMPA.py 0.005" &
xterm -e "python runCMAESMPA.py 0.01" &
xterm -e "python runCMAESMPA.py 0.02" &
xterm -e "python runCMAESMPA.py 0.04" &
