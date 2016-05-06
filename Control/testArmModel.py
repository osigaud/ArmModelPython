#!/usr/bin/env python
# -*- coding: utf-8 -*-

from Main.MainCMAES import generateFromRBFN
from Utils.Chrono import Chrono

def run():
    c = Chrono()
    generateFromRBFN(2, "Full", "Test")
    c.stop()

run()
