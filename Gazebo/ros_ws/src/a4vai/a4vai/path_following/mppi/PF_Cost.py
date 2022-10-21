# Public libs
import numpy as np

# Private libs
from .VirtualTarget import distToPath

#.. Calc. Cost
def Calc_PF_cost(W1, W2, nextWPidx, WPs, Pos, Vn, Throttle):
    prevWPidx    =   max(nextWPidx - 1, 0)
    d2WP        =   10000.
    for i_wp in range(prevWPidx + 1):
        nearWPidx   =   prevWPidx - i_wp
        nearWP      =   WPs[nearWPidx]
        d2WP_tmp    =   np.linalg.norm(nearWP - Pos)
        if d2WP_tmp > d2WP:
            break
        d2WP        =   d2WP_tmp

    maxWPidx    =   WPs.shape[0] - 1
    arrWPidx    =   np.array([nearWPidx - 1, nearWPidx, nearWPidx + 1, nearWPidx + 2])
    arrWPidx    =   np.where(arrWPidx < 0, 0, arrWPidx)
    arrWPidx    =   np.where(arrWPidx > maxWPidx, maxWPidx, arrWPidx)
    arrD2P      =   np.zeros(arrWPidx.shape[0] - 1)
    for i_wpidx in range(arrWPidx.shape[0] - 1):
        nearWPidx   =   arrWPidx[i_wpidx]
        nextWPidx   =   arrWPidx[i_wpidx + 1]
        prevWP      =   WPs[nearWPidx]
        if nearWPidx == nextWPidx:
            arrD2P[i_wpidx] =   np.linalg.norm(prevWP - Pos)
        else:
            nextWP  =   WPs[nextWPidx]
            arrD2P[i_wpidx] =   distToPath(Pos, prevWP, nextWP)

    dist_Path   =   np.min(arrD2P)

    c_d2p           =   dist_Path * dist_Path
    c_ctrl_e        =   Throttle * Throttle    
    Spd             =   np.linalg.norm(Vn)
    c_Spd           =   1/max(Spd,0.1)
    cost            =   W1 * c_d2p + W2 * c_ctrl_e * c_Spd
    return cost, dist_Path