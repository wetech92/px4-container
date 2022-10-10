# pulbic libs.
import numpy as np

# private libs.
from .CommonFunctions import Get_Euler2DCM, Get_Vec2AzimElev

class NDO():
    def __init__(self, lx_NDO, ly_NDO, lz_NDO) -> None:
    #.. NDO vars
        self.z_NDO      =   np.zeros(3)
        self.lx_NDO     =   lx_NDO
        self.ly_NDO     =   ly_NDO
        self.lz_NDO     =   lz_NDO

        self.outNDO     =   np.zeros(3)
        self.a_drag_n   =   np.zeros(3)

    def NDO_main(self, dt, Vn, FbCmd, AngEuler, mass, rho, Sref, CD, g):

        # Calc. Aero. Force
        psi, gam        =   Get_Vec2AzimElev(Vn)
        angI2W          =   np.array([0., -gam, psi])
        cI_W            =   Get_Euler2DCM(angI2W)
        cW_I            =   np.transpose(cI_W)

        Spd             =   np.linalg.norm(Vn)
        qbar            =   0.5 * rho * Spd * Spd
        a_drag_w        =   np.array([ -qbar*Sref*CD / mass, 0., 0. ])
        self.a_drag_n   =   np.dot(cW_I, a_drag_w)
        
        # Calc. Acc. Cmd.
        cI_B            =   Get_Euler2DCM(AngEuler)
        cB_I            =   np.transpose(cI_B)
        Acmdn           =   np.dot(cB_I, FbCmd / mass) + np.array([0., 0., g]) + self.a_drag_n 

        dz_NDO      =   np.zeros(3)
        dz_NDO[0]   =   -self.lx_NDO*self.z_NDO[0] - self.lx_NDO * \
            (self.lx_NDO*Vn[0] + Acmdn[0])
        dz_NDO[1]   =   -self.ly_NDO*self.z_NDO[1] - self.ly_NDO * \
            (self.ly_NDO*Vn[1] + Acmdn[1])
        dz_NDO[2]   =   -self.lz_NDO*self.z_NDO[2] - self.lz_NDO * \
            (self.lz_NDO*Vn[2] + Acmdn[2])
        
        self.outNDO[0]    =   self.z_NDO[0] + self.lx_NDO*Vn[0]
        self.outNDO[1]    =   self.z_NDO[1] + self.ly_NDO*Vn[1]
        self.outNDO[2]    =   self.z_NDO[2] + self.lz_NDO*Vn[2]

        self.z_NDO  =   self.z_NDO + dz_NDO*dt

        pass
    
# def NDO_dyn(lx_NDO, ly_NDO, lz_NDO, z_NDO, dt, Acmdn, Vn):    

#     dz_NDO      =   np.zeros(3)
#     dz_NDO[0]   =   -lx_NDO*z_NDO[0] - lx_NDO * \
#         (lx_NDO*Vn[0] + Acmdn[0])
#     dz_NDO[1]   =   -ly_NDO*z_NDO[1] - ly_NDO * \
#         (ly_NDO*Vn[1] + Acmdn[1])
#     dz_NDO[2]   =   -lz_NDO*z_NDO[2] - lz_NDO * \
#         (lz_NDO*Vn[2] + Acmdn[2])
    
#     outNDO   =   np.zeros(3)
#     outNDO[0]    =   z_NDO[0] + lx_NDO*Vn[0]
#     outNDO[1]    =   z_NDO[1] + ly_NDO*Vn[1]
#     outNDO[2]    =   z_NDO[2] + lz_NDO*Vn[2]

#     z_NDO_new       =   z_NDO + dz_NDO*dt

#     return outNDO, z_NDO_new