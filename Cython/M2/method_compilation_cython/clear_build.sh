#!/bin/sh
rm -f *.so *.c #Remove all file from compilation
rm -fr build/

find . -name "*.pyc" -type f -delete # clear all *.pyc in the subdirectory (compiled python file resulted from execution) 
find . -name "*.so" -type f -delete  # clear all *.so in the subdirectory (cython compiled files)
find . -name "*.o" -type f -delete  # clear all *.o in the subdirectory (c compiled files)
find . -name "*.c" -type f -delete  # clear all *.c in the subdirectory (interpreted c files)

find . -name "*~" -type f -delete  # clear all *~ temps files

