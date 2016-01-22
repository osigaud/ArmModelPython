#!/usr/bin/env python
# -*- coding: utf-8 -*-

from Main.Main import generateFromNN
from Utils.Chrono import Chrono

def run():
    c = Chrono()
    generateFromNN(1, "Random", "R1")
    c.stop()

run()
