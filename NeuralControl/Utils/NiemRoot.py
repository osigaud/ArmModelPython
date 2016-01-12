#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Module: NiemRoot

Description: on retrouve dans ce fichier une fonction permettant de recuperer la raciene n-ieme d'un nombre
'''
from math import fabs

def tronquerNB(nb, vir):
    ex = 10**vir
    nb = nb*ex
    nb = int(nb)
    nb = float(nb)
    nb = nb/ex
    return nb

