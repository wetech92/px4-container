from binvox.binvox import Binvox
import scipy.io
import numpy as np

readVal = Binvox.read("/root/python/est3.binvox",'dense')
scipy.io.savemat('/root/python/readVal.mat',{"data":readVal.data.astype(int)})
# TEMPORARY WORK. SAVE VOXEL NP ARRAY DATA AS MAT TO PROCESS IT ON MATLAB