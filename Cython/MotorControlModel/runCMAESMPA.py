#!/usr/bin/env python
# -*- coding: utf-8 -*-


from Main import launchCMAESForSpecificTargetSize

import sys

size = float(sys.argv[1])

launchCMAESForSpecificTargetSize(size)
