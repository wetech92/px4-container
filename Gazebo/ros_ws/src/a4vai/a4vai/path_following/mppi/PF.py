# Public libs
import numpy as np
from math import pi
R2D = 180. / pi

# Private libs
from .ParamsOffBoardCtrl import DataGCU
from .CommonFunctions import Get_Vec2AzimElev, Get_Euler2DCM
from .VirtualTarget import Calc_VirTgPos
from .Kinematics import Kinematics
from .GCU_Main import Guid_pursuit, SpdCtrller, AccCmdToCtrlCmd
from .PF_Cost import Calc_PF_cost

class PF():
    def __init__(self, dt, WPs) -> None:
    #.. Parameters
        self.GCUParams  =   DataGCU(dt)
        self.WPs        =   WPs

    #.. Datalog
        #self.datalogFile    =   open("/root/datalog/data/datalog.txt",'w')

    #.. Vars.
        self.GCUTime        =   0.
        self.total_cost     =   0.
        self.AccCmdw_w_NDO  =   np.zeros(3)

    def PF_main(self, nextWPidx, Pos, Vn, AngEuler, Acc_disturb):
        
        #.. Virtual Target
            # input
            WPs         =   self.WPs
            LAD         =   self.GCUParams.lookAheadDist
            # function & output
            tgPos       =   Calc_VirTgPos(Pos, nextWPidx, WPs, LAD)


        #.. Kinematics
            # input
            tgPos
            tgVn        =   np.array([0., 0., 0.])
            Pos
            # function & output
            LOSazim, LOSelev, dLOSvec, reldist, tgo  =   Kinematics(tgPos, tgVn, Pos, Vn)

        #.. Guidance
            Kgain           =   self.GCUParams.Kgain_guidPursuit
            tgo
            LOSazim
            LOSelev
            Vn
            AccLim          =   self.GCUParams.AccLim
            # function & output
            AccCmdw_Lat     =   Guid_pursuit(Kgain, tgo, LOSazim, LOSelev, Vn, AccLim)
            AccCmdw         =   AccCmdw_Lat

        #.. speed control module
            # vars. for inputs
            magAccCmdLat    =   np.linalg.norm(AccCmdw_Lat)
            desSpd          =   max(self.GCUParams.desSpd - self.GCUParams.desSpd_weight*magAccCmdLat, self.GCUParams.desSpd * 0.5)

            # input
            Kp              =   self.GCUParams.Kp_vel
            Ki              =   self.GCUParams.Ki_vel
            Kd              =   self.GCUParams.Kd_vel
            int_err_spd     =   self.GCUParams.int_err_spd
            prev_err        =   self.GCUParams.prev_val
            Vn
            dt              =   self.GCUParams.dt_GCU
            # function & output
            AccCmdw_Ax, int_err_spd, prev_err   =   SpdCtrller(Kp, Ki, Kd, int_err_spd, prev_err, dt, Vn, desSpd)
            self.GCUParams.int_err_spd     =   int_err_spd
            self.GCUParams.prev_val        =   prev_err
            AccCmdw[0]      =   AccCmdw_Ax[0]
        
        #.. Disturbance Rejection
            # vars. for inputs
            psi, gam        =   Get_Vec2AzimElev(Vn)
            angI2W          =   np.array([0., -gam, psi])
            cI_W            =   Get_Euler2DCM(angI2W)
            cW_I            =   np.transpose(cI_W)

            AccCmdn         =   np.dot(cW_I, AccCmdw)
            AccCmdn_total   =   AccCmdn - Acc_disturb

            # min, max
            AccCmdn_total     =   np.where(AccCmdn_total > AccLim, AccLim, AccCmdn_total)
            AccCmdn_total     =   np.where(AccCmdn_total < -AccLim, -AccLim, AccCmdn_total)
            
        #.. Accel. to Att.
            # input
            AccCmdn_total
            LOSazim
            throttle_Hover  =   self.GCUParams.throttle_Hover
            mass            =   self.GCUParams.Mass
            g               =   self.GCUParams.g0
            # function & output
            AttCmd, FbCmd   =   AccCmdToCtrlCmd(AccCmdn_total, LOSazim, throttle_Hover, mass, g)
            if (AngEuler[2] - AttCmd[2])*R2D < -200.:
                AttCmd[2]   =   AttCmd[2] - 2*pi
            if (AngEuler[2] - AttCmd[2])*R2D > 200.:
                AttCmd[2]   =   AttCmd[2] + 2*pi
            # function & output
            totalFbCmd      =   np.linalg.norm(FbCmd)
            ThrustHover     =   throttle_Hover / (mass * g)
            ThrustCmd       =   totalFbCmd * ThrustHover

            self.GCUParams.FbCmd = FbCmd

        #.. Calc. Cost
            prevWPidx    =   max(nextWPidx - 1, 0)
            self.GCUParams.prevWPidx    =   prevWPidx
            
            if nextWPidx > 2:
                W1, W2          =   self.GCUParams.W1_cost, self.GCUParams.W2_cost
                cost, dist_Path =   Calc_PF_cost(W1, W2, nextWPidx, WPs, Pos, Vn, ThrustCmd)
            else:
                cost, dist_Path =   0., 0.

            self.total_cost     =   self.total_cost + cost

            # if self.GCUParams.Flag_Write == 1:    
            #     if nextWPidx < WPs.shape[0]:
            #     #.. Save Data
            #         AccCmdw_total   =   np.dot(cI_W, AccCmdn_total)
            #         outNDOw         =   np.dot(cI_W, Acc_disturb)
            #         Spd     =   np.linalg.norm(Vn)
            #         Data = "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n" %(
            #             self.GCUTime, AttCmd[0]*R2D, AttCmd[1]*R2D, AttCmd[2]*R2D, AngEuler[0]*R2D, AngEuler[1]*R2D, AngEuler[2]*R2D,
            #             AccCmdw_total[0], AccCmdw_total[1], AccCmdw_total[2], 0., 0., 0.,
            #             Pos[0], Pos[1], Pos[2], tgPos[0], tgPos[1], tgPos[2], 
            #             cost, self.total_cost, dist_Path, outNDOw[0], outNDOw[1], outNDOw[2],
            #             Spd, self.GCUParams.desSpd, self.GCUParams.lookAheadDist, 0., 0., 0.,
            #             )
            #         self.datalogFile.write(Data)
            #     else:
            #         self.datalogFile.close()

            return ThrustCmd, AttCmd, tgPos, LOSazim