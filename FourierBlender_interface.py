import FourierBlender_path

from ctypes import cdll, c_double, c_void_p, c_size_t, POINTER, c_int

lib = cdll.LoadLibrary(FourierBlender_path.binary_path)

lib.construct.argtypes = [c_size_t, c_size_t, c_double]
lib.construct.restype = c_void_p

lib.get_scales.argtypes = [c_void_p, c_size_t]
lib.get_scales.restype = POINTER(c_double * 3)

lib.get_locations.argtypes = [c_void_p, c_size_t]
lib.get_locations.restype = POINTER(c_double * 3)

lib.get_rotations.argtypes = [c_void_p, c_size_t]
lib.get_rotations.restype = POINTER(c_double * 4)

class FourierBlender(object):
    def __init__(self, resolution, num_frames, v):
        self.obj = lib.construct(resolution, num_frames, v)

    def scales(self, i):
        return lib.get_scales(self.obj, i)

    def locations(self, i):
        return lib.get_locations(self.obj, i)

    def rotations(self, i):
        return lib.get_rotations(self.obj, i)
