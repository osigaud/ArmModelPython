#!/bin/sh
./method_compilation_cython/clear_build.sh #Remove all file from compilation

rm log_cythonisation.txt

python method_compilation_cython/setup__ArmModel.py build_ext --inplace >> method_compilation_cython/log_cythonisation.txt #Lauch the compilation of the module ArmModel

rm -r build #On supprime les fichiers temporaires (/build) issu de la compilations - @Todo : verifier si ont peux supprimer ce fichier

echo "Cythonisation done : check log_cythonisation for more informations"
