#!/usr/bin/env python
# -*- coding: utf-8 -*-

from Main.Main import generateFromRBFN
from Utils.Chrono import Chrono

def run():
    c = Chrono()
    generateFromRBFN(2, "Full4", "Test")
    c.stop()

run()
