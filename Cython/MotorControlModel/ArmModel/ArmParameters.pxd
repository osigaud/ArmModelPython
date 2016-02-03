import numpy as np
cimport numpy as np # import special compile-time information about the numpy module for cython

cdef class ArmParameters:
    '''
    class ArmParameters
    '''

    # ATTRIBUTE DECLARATIONS

    cdef char* pathSetupFile # where is the setup file : Arm.params

    cdef float l1  # Arm length
    cdef float l2  # ForeArm length
    cdef float m1  # Arm mass
    cdef float m2  # ForeArm mass
    cdef float I1  # Arm inertia
    cdef float I2  # ForeArm inertia
    cdef float s1  # Distance from the center of segment 1 to its center of mass
    cdef float s2  # Distance from the center of segment 2 to its center of mass
    cdef float k1,k2,k3   # parameter for inertia matrix

    # @todo : set the type in the array

    cdef np.ndarray At # moment arm matrix A
    cdef np.ndarray B # Damping matrix

    cdef float sub # Shoulder upper bound
    cdef float slb # Shoulder lower bound
    cdef float eub # Elbow upper bound
    cdef float elb # Elbow lower bound


    # Function

    cdef readSetupFile(self)
    cdef massMatrix(self)
    cdef BMatrix(self)
    cdef AMatrix(self)
    cdef readStops(self)