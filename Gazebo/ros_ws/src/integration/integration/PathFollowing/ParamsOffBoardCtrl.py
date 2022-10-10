# pulbic libs.
import numpy as np
from numpy import ones, zeros
from math import pi, ceil
# private libs.

D2R =   pi/180.

# data structure (class)
class DataMPPI():
    def __init__(self) -> None:
        
        # param - MPPI
        self.UpdateCycle    =   5
        self.tau_LPF        =   0.4
        self.count          =   0

        # parameters - dt 0.08        -   more calculation & performance
        self.N              =   50
        self.dt_MPPI        =   80. / 1000.

        # # parameters - dt 0.16          -   less calculation & performance
        # self.N              =   25
        # self.dt_MPPI        =   160. / 1000.        
        
        self.N_tau_LPF      =   ceil(self.tau_LPF/self.dt_MPPI)
        
        # parameters
        self.K              =   32*2*4      # 128   256
        # self.N              =   25
        self.var1           =   0.3         # 0.3
        self.lamb1          =   0.001         # 0.05
        self.var2           =   0.6        # 0.9
        self.lamb2          =   0.001         # 0.05
        # variables
        self.est_delAccn    =   zeros((self.N, 3))
        # self.dt_MPPI        =   160. / 1000.
        self.init_u1_MPPI   =   3.
        self.u1_MPPI        =   self.init_u1_MPPI*ones(self.N)
        self.init_u2_MPPI   =   3.
        self.u2_MPPI        =   self.init_u2_MPPI*ones(self.N)
        # limit
        self.u1_min         =   0.1
        self.u2_min         =   0.1
        
        pass

class DataGCU():
    def __init__(self, dt=0.004) -> None:
    
    #.. params. controller
        self.Mode_Ctrl      =   1      # 0 : Position, 1 : Attitude
        self.Flag_Write     =   1
        self.dt_GCU         =   dt

    #.. Cost params
        self.W1_cost        =   0.03 * 2.0      # 0.03 * 1.0
        self.W2_cost        =   0.02 * 0.1      # 0.02 * 0.2

    #.. Aero. params
        self.CD_model   =   2.0
        self.Sref       =   0.4
        self.rho        =   1.224

    # #.. GPR vars
    #     self.TrainMax   =   20         # 20220726 DY
    #     self.flagPlot   =   0
    #     self.flagGPR    =   0
    #     self.flagSave   =   1

    # #.. GPR learning
    #     kernel          =   gp.kernels.RBF(length_scale=1.0, length_scale_bounds=(1e-3, 100.0)) * gp.kernels.ConstantKernel(1, (1e-2, 200.0))
    #     self.gpr        =   gp.GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=0, alpha=0.1, normalize_y=True) #0.001

    #.. params. vel. hold controller
        self.Kp_vel             =   2.0 * 1.
        self.Ki_vel             =   self.Kp_vel*0.0         # test NDO
        self.Kd_vel             =   0.
        self.prev_val           =   0.

        self.int_err_spd        =   0.
        self.desSpd             =   3.         # 5.
        self.desSpd_weight      =   0.3
        self.dist_Path          =   0.

    #.. params thrust
        # self.throttle_Hover     =   0.7
        self.throttle_Hover     =   0.36
        self.g0                 =   9.81
        self.Mass               =   2.02

    #.. temp.
        self.FbCmd          =   np.array([0., 0., -self.g0 * self.Mass])

    #.. params. pursuit guidance
        self.Kgain_guidPursuit  =   3.           # default guidance
        self.AccLim             =   9.81 * 1.0
        self.tau_control        =   np.array([0.1, 0.1, 0.1]) * 3.

    #.. params. virtual target
        self.lookAheadDist      =   3.     # 3, 3.5
        self.reachDist          =   self.lookAheadDist
        self.prevWPidx          =   0
        # self.WPs                =   np.zeros(3)
    
    #.. states
        self.Pos                =   np.zeros(3)
        self.Vn                 =   np.zeros(3)
        self.AngEuler           =   np.zeros(3)

        pass