'''
import ctypes
import os

# Load DLL into memory.

phobos_dir=os.path.join("C:\\","Program Files","JOANNEUM RESEARCH","Phobos","bin")
os.chdir(phobos_dir)
Phobos_dll = "PhobosIntegration.dll"
hllDll = ctypes.WinDLL (Phobos_dll)

class A(Structure):
    _fields_ = [
       ("a1", c_char_p),
       ("a2", c_int)]

class SMatchingParametersInput(Structure):
    _fields_ = [
        ("bHasNodata", ctypes.c_bool),
        ("fNodataValue", ctypes.c_float),
        ("bWriteBackDisp", ctypes.c_bool),
        ("bWriteBackMatchDistance", ctypes.c_bool),
        ("bWriteRevBackMatchDistance", ctypes.c_bool),
        ("strBackMatchDistanceOutputFile", ctypes.c_char_p),
        ("strRevBackMatchDistanceOutputFile", ctypes.c_char_p),
        ("nNumberOfPyramids", ctypes.c_int),
        ("nPredictor", ctypes.c_int),
        ("strDispPredictionFile", ctypes.c_char_p),
        ("strBackDispPredictionFile", ctypes.c_char_p),
        ("fTopPredictionShift", ctypes.c_float),
        ("nJointMode", ctypes.c_int),
        ("nJointModeKernelSize", ctypes.c_int),
        ("nNumberOfMorphologicalIter", ctypes.c_int),
        ("oGPUs", #TODO std::vector<int>),
        ("oPyramidParams", #TODO std::vector<SMatchingParameters>),
        ("oFinalSubPixelParameters", #TODO SMatchingParameters),
        ("nDebugLevel", ctypes.c_int),
        ("strDebugOutput", ctypes.c_char_p)
        ]

#JR::Phobos::ReadIniFile(params, tmp_param_file);



# Set up prototype and parameters for the desired function call.
# HLLAPI

hllApiProto = ctypes.WINFUNCTYPE (
    ctypes.c_int,      # Return type.
    ctypes.c_void_p,   # Parameters 1 ...
    ctypes.c_void_p,
    ctypes.c_void_p,
    ctypes.c_void_p)   # ... thru 4.
hllApiParams = (1, "p1", 0), (1, "p2", 0), (1, "p3",0), (1, "p4",0),
# Actually map the call ("HLLAPI(...)") to a Python name.

hllApi = hllApiProto (("HLLAPI", hllDll), hllApiParams)

# This is how you can actually call the DLL function.
# Set up the variables and call the Python name with them.

p1 = ctypes.c_int (1)
p2 = ctypes.c_char_p (sessionVar)
p3 = ctypes.c_int (1)
p4 = ctypes.c_int (0)
hllApi (ctypes.byref (p1), p2, ctypes.byref (p3), ctypes.byref (p4))
'''