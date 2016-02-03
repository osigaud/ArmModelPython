import numpy as np
cimport numpy as np # import special compile-time information about the numpy module for cython


from ArmModel.MusclesParameters cimport MusclesParameters

from ArmModel.ArmParameters cimport ArmParameters


cdef class Arm:
    '''
    class Arm
    '''

    # ATTRIBUTE DECLARATIONS

    cdef np.ndarray state #numpy array, state vector
    cdef np.ndarray __dotq0
    cdef float dt # Pas de temps utilise pour l'experimentation
    cdef ArmParameters armP
    cdef MusclesParameters musclesP


    # Function

    cpdef setState(self, np.ndarray state)
    cpdef np.ndarray getState(self)

    cpdef np.ndarray get_dotq_0(self)
    cpdef set_dotq_0(self, np.ndarray dotq_0)

    cpdef setDT(self, float dt)
    cpdef float getDT(self)


    cdef ArmParameters get_ArmParameters(self)
    cdef MusclesParameters get_MusclesParameters(self)

    cpdef np.ndarray computeNextState(self, np.ndarray U, np.ndarray state)
    cdef np.ndarray jointStop(self,np.ndarray q)
    # @todo : define the proper output type
    cpdef mgdFull(self, np.ndarray q)
    cpdef np.ndarray jacobian(self, np.ndarray q)
    cpdef list mgdEndEffector(self, np.ndarray q)
    cpdef mgi(self, float xi, float yi)
