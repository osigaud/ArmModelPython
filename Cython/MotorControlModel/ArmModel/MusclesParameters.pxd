import numpy as np
cimport numpy as np # import special compile-time information about the numpy module for cython

cdef class MusclesParameters:
    '''
    class MusclesParameters
    '''

    # ATTRIBUTE DECLARATIONS

    cdef char* pathSetupFile

    ###############################Annexe parameters########################
    #Hogan parameters
    cdef int GammaMax
    cdef float K #stiffness

    # @todo : set the right data type to the array
    cdef np.ndarray Gamma_H #hogan torque initialization

    #stiffness matrix (null)
    # @todo : set the right data type to the array
    cdef np.ndarray Knulle
    #stiffness matrix (low)
    cdef float Kp1
    cdef float Kp2
    cdef float KP1
    cdef float KP2
    # @todo : set the right data type to the array
    cdef np.ndarray Kraid
    #stiffness matrix (high)
    cdef float Kp11
    cdef float Kp22
    cdef float KP11
    cdef float KP22
    # @todo : set the right data type to the array
    cdef np.ndarray Kgrand
    #Proportional gain
    cdef  float Kp
    #Derivative gain
    cdef float Kd

    cdef np.ndarray fmax # Matrix of the maximum force exerted by each muscle
    cdef float knoiseU # amount of motor noise on U

    cdef np.ndarray U0 # muscular activation vector

    # Function

    cdef fmaxMatrix(self)
    cdef activationVectorInit(self)
    cpdef activationVectorUse(self, u1, u2, u3, u4, u5, u6)
