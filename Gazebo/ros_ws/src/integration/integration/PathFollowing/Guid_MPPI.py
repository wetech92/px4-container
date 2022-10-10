# pulbic libs.
import math as m
import numpy as np
import pycuda.driver as cuda
import pycuda.autoinit
from pycuda.compiler import SourceModule

# private libs.
from .ParamsOffBoardCtrl import DataMPPI, DataGCU

class MPPI():
    def __init__(self) -> None:        
        self.MPPIParams         =   DataMPPI()
        pass

    def Guid_MPPI(self, GCUParams:DataGCU, WPs, Pos, Vn, AngEuler):    
        # MPPI params.
        K               =   self.MPPIParams.K
        N               =   self.MPPIParams.N
        var1            =   self.MPPIParams.var1
        var2            =   self.MPPIParams.var2
        # variables
        u1_MPPI         =   self.MPPIParams.u1_MPPI
        u2_MPPI         =   self.MPPIParams.u2_MPPI
        est_delAccn     =   self.MPPIParams.est_delAccn
        # set MPPI params & vars
        arr_intMPPI     =   np.array([K, N]).astype(np.int32)
        arr_u1_MPPI     =   np.array(u1_MPPI).astype(np.float64)
        arr_delta_u1    =   var1*np.random.randn(N,K).astype(np.float64)
        arr_u2_MPPI     =   np.array(u2_MPPI).astype(np.float64)
        arr_delta_u2    =   var2*np.random.randn(N,K).astype(np.float64)
        arr_stk         =   np.zeros(K).astype(np.float64)
        arr_delAccn     =   np.array(est_delAccn).astype(np.float64)
        gpu_intMPPI     =   cuda.mem_alloc(arr_intMPPI.nbytes)
        gpu_u1_MPPI     =   cuda.mem_alloc(arr_u1_MPPI.nbytes)
        gpu_delta_u1    =   cuda.mem_alloc(arr_delta_u1.nbytes)
        gpu_u2_MPPI     =   cuda.mem_alloc(arr_u2_MPPI.nbytes)
        gpu_delta_u2    =   cuda.mem_alloc(arr_delta_u2.nbytes)
        gpu_stk         =   cuda.mem_alloc(arr_stk.nbytes)
        gpu_delAccn     =   cuda.mem_alloc(arr_delAccn.nbytes)
        cuda.memcpy_htod(gpu_intMPPI,arr_intMPPI)
        cuda.memcpy_htod(gpu_u1_MPPI,arr_u1_MPPI)
        cuda.memcpy_htod(gpu_delta_u1,arr_delta_u1)
        cuda.memcpy_htod(gpu_u2_MPPI,arr_u2_MPPI)
        cuda.memcpy_htod(gpu_delta_u2,arr_delta_u2)
        cuda.memcpy_htod(gpu_stk,arr_stk)
        cuda.memcpy_htod(gpu_delAccn,arr_delAccn)

        # model & scenario params & vars
        prevWPidx       =   GCUParams.prevWPidx
        numWPs          =   WPs.shape[0]
        # WPs             =   WPs
        reachDist       =   GCUParams.reachDist
        lookAheadDist   =   GCUParams.lookAheadDist
        dt              =   self.MPPIParams.dt_MPPI 
        desSpd          =   GCUParams.desSpd
        Kgain_PG        =   GCUParams.Kgain_guidPursuit
        tau_control     =   GCUParams.tau_control
        Mass            =   GCUParams.Mass
        throttle_Hover  =   GCUParams.throttle_Hover
        g0              =   GCUParams.g0
        AccLim          =   GCUParams.AccLim
        Kp_vel          =   GCUParams.Kp_vel
        Ki_vel          =   GCUParams.Ki_vel
        Kd_vel          =   GCUParams.Kd_vel
        CD0_md          =   GCUParams.CD_model
        Sref            =   GCUParams.Sref
        rho             =   GCUParams.rho
        W1              =   GCUParams.W1_cost
        W2              =   GCUParams.W2_cost
        # Pos             =   GCUParams.Pos
        # Vn              =   GCUParams.Vn
        # AngEuler        =   GCUParams.AngEuler

        # set model & scenario params & vars
        arrWPParams     =   np.array([prevWPidx, numWPs]).astype(np.int32)
        arrWPsNED       =   np.array(WPs).astype(np.float64)
        arrDistParams   =   np.array([reachDist, lookAheadDist]).astype(np.float64)
        arrModelParams  =   np.array([dt, desSpd, Kgain_PG, tau_control[0], tau_control[1], tau_control[2], \
            Mass, throttle_Hover, g0, AccLim, Kp_vel, Ki_vel, Kd_vel, CD0_md, Sref, rho, W1, W2]).astype(np.float64)
        arrInitStates   =   np.array([Pos, Vn, AngEuler]).astype(np.float64)
        gpuWPParams     =   cuda.mem_alloc(arrWPParams.nbytes)
        gpuWPsNED       =   cuda.mem_alloc(arrWPsNED.nbytes)
        gpuDistParams   =   cuda.mem_alloc(arrDistParams.nbytes)
        gpuModelParams  =   cuda.mem_alloc(arrModelParams.nbytes)
        gpuInitStates   =   cuda.mem_alloc(arrInitStates.nbytes)
        cuda.memcpy_htod(gpuWPParams,arrWPParams)
        cuda.memcpy_htod(gpuWPsNED,arrWPsNED)
        cuda.memcpy_htod(gpuDistParams,arrDistParams)
        cuda.memcpy_htod(gpuModelParams,arrModelParams)
        cuda.memcpy_htod(gpuInitStates,arrInitStates)

        mod     =   SourceModule("""
        // calculation functions
        __device__ void GetAngleSndCosLaw(double len1, double len2, double len3, double *ang3);
        __device__ void GetEuclideanNorm(double vec[3], double *res);
        __device__ void Get_Vec2AzimElev(double vec[3], double *azim, double *elev);
        __device__ void Get_Euler2DCM(double AngEuler[3], double DCM[3][3]);
        __device__ void Get_invDCM(double DCM_in[3][3], double DCM_out[3][3]);
        __device__ void Mul_Mat33Vec3(double Mat[3][3], double Vec_in[3], double Vec_out[3]);
        __device__ void VecCross(double x[3], double y[3], double res[3]);
        __device__ void Get_invM33(double M33[3][3], double invM33[3][3]);

        // virtual targets & way point functions
        __device__ void CheckWayPoint(double Posn[3], int *prevWPidx, double *arrWPsNED, int numWPs, double reachDist, int *check);
        __device__ void distToPath(double Posn[3], double prevWP[3], double nextWP[3], double *res);
        __device__ void Calc_tgPos_direct(double Posn[3], int prevWPidx, double *arrWPsNED, int numWPs, double lookAheadDist, double tgPosn[3]);

        // quadrotor module functions
        __device__ void Kinematics(double tgPosn[3], double tgVn[3], double Posn[3], double Vn[3], double *LOSazim, double *LOSelev, double dLOSvec[3], double *relDist, double *tgo);
        __device__ void Guid_pursuit(double Kgain_PG, double tgo, double LOSazim, double LOSelev, double Vn[3], double AccLim, double AccCmdw[3]);
        __device__ void SpdCtrller(double Kp, double Ki, double Kd, double *int_err_spd, double *prev_err_spd, double dt, double Vn[3], double desSpd, double *AccCmdXw);
        __device__ void AccCmdToCtrlCmd(double AccCmdn[3], double psi_FPA, double throttle_Hover, double mass, double g, double AngEuler_Cmd[3], double Fcmd_b[3]);
        __device__ void Dynamics_p6dof(double Fb_tot[3], double g0, double tau_ctrl[3], double AngEuler_Cmd[3], double AngEuler[3], double Vb[3], double cI_B[3][3], double Mass, double dot_AngEuler[3], double dot_Vb[3], double dot_Posn[3]);
        __device__ void Integration_Euler(double Vb[3], double AngEuler[3], double Pos[3], double dot_Vb[3], double dot_AngEuler[3], double dot_Pos[3], double dt);

        // main function
        __global__ void mainCuda(int* arr_intMPPI, double* arr_u1_MPPI, double* arr_delta_u1, double* arr_u2_MPPI, double* arr_delta_u2, double* arr_stk, double *arr_delAccn, int* arrWPParams, double *arrWPsNED, double *arrDistParams, double *arrModelParams, double *arrInitStates)
        {
            double pi   =   acos(-1.);

            // parallel GPU core index
            int idx     =   threadIdx.x + threadIdx.y*blockDim.x + blockIdx.x*blockDim.x*blockDim.y + blockIdx.y*blockDim.x*blockDim.y*gridDim.x;
            
            // MPPI params.
            int K               =   arr_intMPPI[0];
            int N               =   arr_intMPPI[1];

            // way points
            double reachDist    =   arrDistParams[0];
            int prevWPidx       =   arrWPParams[0];
            int numWPs          =   arrWPParams[1];
            
            // virtual target
            double lookAheadDist=   arrDistParams[1];
            double tgPosn[3]    =   { 0., };
            double tgVn[3]      =   { 0., };

            // GCU params
            double desSpd       =   arrModelParams[1];
            double Kgain_PG     =   arrModelParams[2];
            double tau_control[3]   =   {arrModelParams[3], arrModelParams[4], arrModelParams[5]};
            double Mass         =   arrModelParams[6];
            double throttle_Hover   =   arrModelParams[7];
            double g0           =   arrModelParams[8];
            double AccLim       =   arrModelParams[9];
            double Kp           =   arrModelParams[10];
            double Ki           =   arrModelParams[11];
            double Kd           =   arrModelParams[12];
            double CD0_md       =   arrModelParams[13];
            double Sref         =   arrModelParams[14];
            double rho          =   arrModelParams[15];
            double W1           =   arrModelParams[16];
            double W2           =   arrModelParams[17];

            // init. states
            double Posn[3]      =   {arrInitStates[0], arrInitStates[1], arrInitStates[2]};
            double Vn[3]        =   {arrInitStates[3], arrInitStates[4], arrInitStates[5]};
            double AngEuler[3]      =   {arrInitStates[6], arrInitStates[7], arrInitStates[8]};
            double cI_B[3][3]   =   { 0., };
            Get_Euler2DCM(AngEuler, cI_B);
            double Vb[3]        =   { 0., } ;
            Mul_Mat33Vec3(cI_B, Vn, Vb);
            double cB_I[3][3]   =   { 0., };
            Get_invDCM(cI_B, cB_I);

            // main loop
            double dt           =   arrModelParams[0];
            double int_err_spd  =   0.;
            double prev_err_spd =   0.;
            for(int i_n = 0; i_n < N; i_n++)
            {
            //.. MPPI input
                //Kgain_PG            =   arr_u1_MPPI[i_n] + arr_delta_u1[idx + K*i_n];
                desSpd              =   arr_u1_MPPI[i_n] + arr_delta_u1[idx + K*i_n];
                lookAheadDist       =   arr_u2_MPPI[i_n] + arr_delta_u2[idx + K*i_n];
                reachDist           =   lookAheadDist;

            //.. Target info
                Calc_tgPos_direct(Posn, prevWPidx, arrWPsNED, numWPs, lookAheadDist, tgPosn);

            //.. Kinematics
                double LOSazim      =   0.;
                double LOSelev      =   0.;
                double dLOSvec[3]   =   {0.,};
                double relDist      =   0.;
                double tgo          =   0.;
                Kinematics(tgPosn, tgVn, Posn, Vn, &LOSazim, &LOSelev, dLOSvec, &relDist, &tgo);

            //.. Check way points
                int checkWP =   0;
                CheckWayPoint(Posn, &prevWPidx, arrWPsNED, numWPs, reachDist, &checkWP);
                if(checkWP == 2)   //  WP ends
                {
                    break;
                }
                
            //.. PS guidance
                double AccCmdw[3]   =   { 0., };
                Guid_pursuit(Kgain_PG, tgo, LOSazim, LOSelev, Vn, AccLim, AccCmdw);
                
            //.. MPPI guidance
                //double accCmdYw     =   arr_u1_MPPI[i_n] + arr_delta_u1[idx + K*i_n];
                //double accCmdZw     =   arr_u2_MPPI[i_n] + arr_delta_u2[idx + K*i_n];
                
            // Speed Controller
                // temp. params
                double desSpd_weight    =   0.5;
                double desSpd_min       =   desSpd * 0.5;

                double magAccCmdLat     =   0.;
                GetEuclideanNorm(AccCmdw, &magAccCmdLat);
                double desSpd_penalty  =   max(desSpd - desSpd_weight*magAccCmdLat, desSpd_min);
                double AccCmdXw     =   0.;
                SpdCtrller(Kp, Ki, Kd, &int_err_spd, &prev_err_spd, dt, Vn, desSpd_penalty, &AccCmdXw);
                AccCmdw[0]          =   AccCmdXw;

            //.. Calc. AccCmdn
                double psi          =   0;
                double gam          =   0;
                Get_Vec2AzimElev(Vn, &psi, &gam);
                double angI2W[3]    =   { 0., -gam, psi };
                double cI_W[3][3]   =   { 0., };
                double cW_I[3][3]   =   { 0., };
                Get_Euler2DCM(angI2W, cI_W);
                Get_invDCM(cI_W, cW_I);
                double AccCmdn[3]   =   { 0., };
                Mul_Mat33Vec3(cW_I, AccCmdw, AccCmdn);

                // temp. params.
                double Spd      =   0.;            
                GetEuclideanNorm(Vn, &Spd);
                double qbar     =   0.5 * rho * Spd * Spd;
                double AccAdy_w[3]  =   { -qbar*Sref*CD0_md / Mass, 0., 0. };
                double AccAdy_n[3]  =   { 0., };
                double AccAdy_b[3]  =   { 0., };
                Mul_Mat33Vec3(cW_I, AccAdy_w, AccAdy_n);            
                Mul_Mat33Vec3(cI_B, AccAdy_n, AccAdy_b);
                double Fady_b[3]    =   {AccAdy_b[0] * Mass, AccAdy_b[1] * Mass, AccAdy_b[2] * Mass};

            //.. AccCmdn all
                double AccCmdn_w_tot[3]  =   { 0., };
                for(int i_a = 0; i_a < 3; i_a++)
                {
                    AccCmdn_w_tot[i_a]  =   AccCmdn[i_a] - AccAdy_n[i_a];
                }
            //.. limit
                for(int i =0; i<3; i++)
                {
                    AccCmdn_w_tot[i]    =   min(AccCmdn_w_tot[i], AccLim);
                    AccCmdn_w_tot[i]    =   max(AccCmdn_w_tot[i], -AccLim);
                }


            //.. AccCmd to Attitude Control Cmd.
                double AngEuler_Cmd[3]  =   { 0., };
                double Fcmd_b[3]        =   {0.,};
                AccCmdToCtrlCmd(AccCmdn_w_tot, LOSazim, throttle_Hover, Mass, g0, AngEuler_Cmd, Fcmd_b);

            //.. Yaw continuity
                if(AngEuler[2] - AngEuler_Cmd[2] < -200. / 57.3)
                {
                    AngEuler_Cmd[2] = AngEuler_Cmd[2] - 2*pi;
                }
                if(AngEuler[2] - AngEuler_Cmd[2] > 200. / 57.3)
                {
                    AngEuler_Cmd[2] = AngEuler_Cmd[2] + 2*pi;
                }

            //.. Dynamics
                
                double Fb_tot[3]    =   { Fady_b[0] + Fcmd_b[0], Fady_b[1] + Fcmd_b[1], Fady_b[2] + Fcmd_b[2] };
                double dot_AngEuler[3]  =   { 0., };
                double dot_Vb[3]    =   { 0., };
                double dot_Posn[3]  =   { 0., };

                Dynamics_p6dof(Fb_tot, g0, tau_control, AngEuler_Cmd, AngEuler, Vb, cI_B, Mass, dot_AngEuler, dot_Vb, dot_Posn);

                double accb[3]      =   { Fb_tot[0]/Mass, Fb_tot[1]/Mass, Fb_tot[2]/Mass };
                double acci[3]    =   { 0., };
                Mul_Mat33Vec3(cB_I, accb, acci);
                double accw[3]    =   { 0., };
                Mul_Mat33Vec3(cI_W, acci, accw);

            //.. integration - euler
                Integration_Euler(Vb, AngEuler, Posn, dot_Vb, dot_AngEuler, dot_Posn, dt);
                Get_Euler2DCM(AngEuler, cI_B);
                double cB_I[3][3]   =   { 0., };
                double Vn_new[3]    =   { 0., };
                Get_invDCM(cI_B, cB_I);
                Mul_Mat33Vec3(cB_I, Vb, Vn_new);
                for(int i_v = 0; i_v < 3; i_v++)
                {
                    Vn[i_v] = Vn_new[i_v];
                }

            //.. calc. cost
                double HE1 =   LOSazim - AngEuler[2];
                HE1    =   abs(atan2(sin(HE1),cos(HE1)));
                double HE2 =   LOSelev - AngEuler[1];
                HE2    =   abs(atan2(sin(HE2),cos(HE2)));

            //.. calc. dist. 2 path
                int nearWPidx       =   0;
                double nearWP[3]    =   { 0., };
                double vec2WP[3]    =   { 0., };
                double d2WP         =   10000.;
                for(int i_wpidx = 0; i_wpidx < prevWPidx + 1; i_wpidx++)
                {
                    nearWPidx   =   prevWPidx - i_wpidx;
                    for(int i_wp = 0; i_wp < 3; i_wp++)
                    {
                        nearWP[i_wp] = arrWPsNED[nearWPidx*3 + i_wp];
                        vec2WP[i_wp] = nearWP[i_wp] - Posn[i_wp];
                    }
                    double d2WP_tmp     =   0.;
                    GetEuclideanNorm(vec2WP, &d2WP_tmp);
                    if (d2WP_tmp > d2WP)
                    {
                        break;
                    }
                    d2WP    =   d2WP_tmp;
                }

                int maxWPidx    =   numWPs - 1;
                int arrWPidx[4] =   {nearWPidx - 1, nearWPidx, nearWPidx + 1, nearWPidx + 2};
                for (int i_wpidx = 0; i_wpidx < 4; i_wpidx++)
                {
                    if(arrWPidx[i_wpidx] < 0) arrWPidx[i_wpidx] = 0;
                    if(arrWPidx[i_wpidx] > maxWPidx) arrWPidx[i_wpidx] = maxWPidx;
                }

                int nextWPidx       = 0;
                double arrD2P[3]    = { 0., };
                double dist2Path    =   10000.;
                for (int i_wpidx = 0; i_wpidx < 3; i_wpidx++)
                {
                    double d2p  =   0.;
                    nearWPidx   =   arrWPidx[i_wpidx];
                    nextWPidx   =   arrWPidx[i_wpidx + 1];
                    double prevWP_tmp[3]    =   { 0., };
                    for(int i_wp = 0; i_wp < 3; i_wp ++) prevWP_tmp[i_wp] = arrWPsNED[nearWPidx*3 + i_wp];
                    if (nearWPidx == nextWPidx)
                    {
                        double vec2WP[3]    =   { 0., };
                        for(int i_wp2 = 0; i_wp2 < 3; i_wp2 ++) vec2WP[i_wp2] = prevWP_tmp[i_wp2] - Posn[i_wp2];
                        GetEuclideanNorm(vec2WP, &d2p);
                    }
                    else
                    {
                        double nextWP_tmp[3]    =   { 0., };
                        for(int i_wp2 = 0; i_wp2 < 3; i_wp2 ++) nextWP_tmp[i_wp2] = arrWPsNED[nextWPidx*3 + i_wp2];
                        distToPath(Posn, prevWP_tmp, nextWP_tmp, &d2p);
                    }
                    arrD2P[i_wpidx] =   d2p;
                    dist2Path   =   min(dist2Path, arrD2P[i_wpidx]);
                }
                
                //double c_d2p        =   max(0., dist2Path - 0.1);
                double c_d2p        =   dist2Path * dist2Path;
                        
                double totalFbCmd   =   0.;
                GetEuclideanNorm(Fb_tot, &totalFbCmd);
                double ThrustHover  =   throttle_Hover / (Mass * g0); 
                double ThrustCmd    =   totalFbCmd * ThrustHover;
                double c_ctrl_e     =   ThrustCmd*ThrustCmd;
                double c_Spd        =   1/max(Spd, 0.1);
                arr_stk[idx]        =   arr_stk[idx] + W1*c_d2p + W2*c_ctrl_e*c_Spd;
            }
        }

        // calculation functions
        __device__ void GetAngleSndCosLaw(double len1, double len2, double len3, double *ang3)
        {
            double cosAng3  =   (len1*len1+len2*len2-len3*len3)/(2*len1*len2);
            ang3[0]         =   acos(cosAng3);
        }
        __device__ void GetEuclideanNorm(double vec[3], double *res)
        {
            double temp     =   0;
            for(int i = 0; i < 3; i++)
            {
                temp    =   temp + vec[i]*vec[i];
            }
            res[0]  =   sqrt(temp);
        }
        __device__ void Get_Vec2AzimElev(double vec[3], double *azim, double *elev)
        {
            azim[0]     =   atan2(vec[1], vec[0]);
            double len  =   sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
            elev[0]     =   atan2(-vec[2], len);
        }
        __device__ void Get_Euler2DCM(double AngEuler[3], double DCM[3][3])
        {
            double spsi     =   sin( AngEuler[2] );
            double cpsi     =   cos( AngEuler[2] );
            double sthe     =   sin( AngEuler[1] );
            double cthe     =   cos( AngEuler[1] );
            double sphi     =   sin( AngEuler[0] );
            double cphi     =   cos( AngEuler[0] );

            DCM[0][0]       =   cpsi * cthe ;
            DCM[1][0]       =   cpsi * sthe * sphi - spsi * cphi ;
            DCM[2][0]       =   cpsi * sthe * cphi + spsi * sphi ;
            
            DCM[0][1]       =   spsi * cthe ;
            DCM[1][1]       =   spsi * sthe * sphi + cpsi * cphi ;
            DCM[2][1]       =   spsi * sthe * cphi - cpsi * sphi ;
            
            DCM[0][2]       =   -sthe ;
            DCM[1][2]       =   cthe * sphi ;
            DCM[2][2]       =   cthe * cphi ;
        }
        __device__ void Get_invDCM(double DCM_in[3][3], double DCM_out[3][3])
        {
            DCM_out[0][0]   =   DCM_in[0][0];
            DCM_out[0][1]   =   DCM_in[1][0];
            DCM_out[0][2]   =   DCM_in[2][0];
            DCM_out[1][0]   =   DCM_in[0][1];
            DCM_out[1][1]   =   DCM_in[1][1];
            DCM_out[1][2]   =   DCM_in[2][1];
            DCM_out[2][0]   =   DCM_in[0][2];
            DCM_out[2][1]   =   DCM_in[1][2];
            DCM_out[2][2]   =   DCM_in[2][2];
        }
        __device__ void Mul_Mat33Vec3(double Mat[3][3], double Vec_in[3], double Vec_out[3])
        {
            for(int i_r = 0; i_r < 3; i_r++)
            {
                for(int i_c = 0; i_c < 3; i_c ++)
                {
                    Vec_out[i_r]    =   Vec_out[i_r] + Mat[i_r][i_c]*Vec_in[i_c];
                }
            }
        }
        __device__ void VecCross(double x[3], double y[3], double res[3])
        {
            res[0] = x[1] * y[2] - x[2] * y[1];
            res[1] = -(x[0] * y[2] - x[2] * y[0]);
            res[2] = x[0] * y[1] - x[1] * y[0];
        }
        __device__ void Get_invM33(double M33[3][3], double invM33[3][3])
        {
        //..3x3 Matrix Inverse
            double det_M33 = 0.;
            det_M33 = det_M33 + M33[0][0] * M33[1][1] * M33[2][2];
            det_M33 = det_M33 + M33[0][1] * M33[1][2] * M33[2][0];
            det_M33 = det_M33 + M33[0][2] * M33[1][0] * M33[2][1];
            det_M33 = det_M33 - M33[0][0] * M33[1][2] * M33[2][1];
            det_M33 = det_M33 - M33[0][1] * M33[1][0] * M33[2][2];
            det_M33 = det_M33 - M33[0][2] * M33[1][1] * M33[2][0];
            // coding rule
            if (fabs(det_M33) < 1e-308)
            {
                det_M33 = 1e-308;
            }
            else
            {
                // coding rule
            }
            invM33[0][0] = (M33[1][1] * M33[2][2] - M33[1][2] * M33[2][1]) / det_M33;
            invM33[0][1] = (M33[2][1] * M33[0][2] - M33[2][2] * M33[0][1]) / det_M33;
            invM33[0][2] = (M33[0][1] * M33[1][2] - M33[0][2] * M33[1][1]) / det_M33;
            invM33[1][0] = (M33[1][2] * M33[2][0] - M33[1][0] * M33[2][2]) / det_M33;
            invM33[1][1] = (M33[2][2] * M33[0][0] - M33[2][0] * M33[0][2]) / det_M33;
            invM33[1][2] = (M33[0][2] * M33[1][0] - M33[0][0] * M33[1][2]) / det_M33;
            invM33[2][0] = (M33[1][0] * M33[2][1] - M33[1][1] * M33[2][0]) / det_M33;
            invM33[2][1] = (M33[2][0] * M33[0][1] - M33[0][0] * M33[2][1]) / det_M33;
            invM33[2][2] = (M33[0][0] * M33[1][1] - M33[0][1] * M33[1][0]) / det_M33;
        }
        // virtual targets & way points functions
        __device__ void CheckWayPoint(double Posn[3], int *prevWPidx, double *arrWPsNED, int numWPs, double reachDist, int *check)
        {
            // get ext WP
            double nextWP[3]    =   {0.,};
            for(int i = 0; i < 3; i++)
            {
                nextWP[i]   =   arrWPsNED[prevWPidx[0]*3 + i + 3];
            }

            // get distance to next WP
            double relPosToNextWP[3]=   {nextWP[0] - Posn[0], nextWP[1] - Posn[1], nextWP[2] - Posn[2]};
            double distToNextWP     =   0.;
            GetEuclideanNorm(relPosToNextWP, &distToNextWP);

            // check way point, 0 : nothing changes, 1 : wp changes, 2 : wp ends
            check[0]    =   0;
            if(reachDist >= distToNextWP)
            {
                prevWPidx[0]   =   prevWPidx[0] + 1;
                check[0]    =   1;
            }
            if(prevWPidx[0] == numWPs - 1)
            {
                check[0]    =   2;
            }
        }
        __device__ void distToPath(double Posn[3], double prevWP[3], double nextWP[3], double *res)
        {
            // trangle lengths
            double vec1[3]  =   {prevWP[0] - Posn[0], prevWP[1] - Posn[1], prevWP[2] - Posn[2]};
            double vec2[3]  =   {nextWP[0] - Posn[0], nextWP[1] - Posn[1], nextWP[2] - Posn[2]};
            double vec3[3]  =   {nextWP[0] - prevWP[0], nextWP[1] - prevWP[1], nextWP[2] - prevWP[2]};
            double len1     =   0.;
            double len2     =   0.;
            double len3     =   0.;
            GetEuclideanNorm(vec1, &len1);
            GetEuclideanNorm(vec2, &len2);
            GetEuclideanNorm(vec3, &len3);
            len3    =   max(len3, 0.0001);

            // get angle
            double ang2     =   0.;
            double ang1     =   0.;
            GetAngleSndCosLaw(len1, len3, len2, &ang2);
            GetAngleSndCosLaw(len2, len3, len1, &ang1);
            if(isnan(ang2))
            {
                ang2    =   0.;
            }
            if(isnan(ang1))
            {
                ang1    =   0.;
            }
            
            // distance to path
            double distToPath   =   len1*sin(ang2);

            double pi   =   acos(-1.);
            if(ang2 > 0.5*pi)
            {
                distToPath = len1;
            }
            if(ang1 > 0.5*pi)
            {
                distToPath = len2;
            }


            res[0]  =   distToPath;
        }
        __device__ void Calc_tgPos_direct(double Posn[3], int prevWPidx, double *arrWPsNED, int numWPs, double lookAheadDist, double tgPosn[3])
        {
            double minval   =   0.00001;        
            double prevWP[3]    =   {0.,};
            double nextWP[3]    =   {0.,};
            for(int i = 0; i < 3; i++)
            {
                prevWP[i]   =   arrWPsNED[prevWPidx*3 + i];
                nextWP[i]   =   arrWPsNED[prevWPidx*3 + i + 3];
            }

            // trangle lengths
            double vec1[3]  =   {prevWP[0] - Posn[0], prevWP[1] - Posn[1], prevWP[2] - Posn[2]};
            double vec2[3]  =   {nextWP[0] - Posn[0], nextWP[1] - Posn[1], nextWP[2] - Posn[2]};
            double vec3[3]  =   {nextWP[0] - prevWP[0], nextWP[1] - prevWP[1], nextWP[2] - prevWP[2]};
            double len1     =   0.;
            double len2     =   0.;
            double len3     =   0.;
            GetEuclideanNorm(vec1, &len1);
            GetEuclideanNorm(vec2, &len2);
            GetEuclideanNorm(vec3, &len3);
            len1    =   max(len1, minval);
            len2    =   max(len2, minval);
            len3    =   max(len3, minval);

            // get angle
            double ang2     =   0.;
            double ang1     =   0.;
            GetAngleSndCosLaw(len1, len3, len2, &ang2);
            GetAngleSndCosLaw(len2, len3, len1, &ang1);
            if(isnan(ang2))
            {
                ang2    =   0.;
            }
            if(isnan(ang1))
            {
                ang1    =   0.;
            }
            
            // distance to path
            double distToPath   =   len1*sin(ang2);
            double cosangle     =   max(0., cos(ang2));

            // closePosOnPath
            double unitVecWP1ToWP2[3]   =   {vec3[0]/len3, vec3[1]/len3 , vec3[2]/len3};
            double closestPosOnPath[3]  =   {0., 0., 0.};
            closestPosOnPath[0]         =   prevWP[0] + unitVecWP1ToWP2[0]*len1*cosangle;
            closestPosOnPath[1]         =   prevWP[1] + unitVecWP1ToWP2[1]*len1*cosangle;
            closestPosOnPath[2]         =   prevWP[2] + unitVecWP1ToWP2[2]*len1*cosangle;

            double pi   =   acos(-1.);
            if(ang2 > 0.5*pi)
            {
                distToPath = len1;
                for(int i_pos = 0; i_pos < 3; i_pos ++)
                {
                    closestPosOnPath[i_pos] =   prevWP[i_pos];
                }
            }
            if(ang1 > 0.5*pi)
            {
                distToPath = len2;
                for(int i_pos = 0; i_pos < 3; i_pos ++)
                {
                    closestPosOnPath[i_pos] =   nextWP[i_pos];
                }
            }

            // target position
            if(distToPath >= lookAheadDist)
            {
                for(int i = 0; i < 3; i++)
                {
                    tgPosn[i]     =   closestPosOnPath[i];
                }

            }
            else
            {
                double pos1[3]  =   {0.,};
                double pos2[3]  =   {closestPosOnPath[0], closestPosOnPath[1], closestPosOnPath[2]};
                double sumDist  =   distToPath;
                double vec12[3]     =   {0.,};
                double dist12       =   0.;
                for(int i_wp  = prevWPidx; i_wp < numWPs - 1; i_wp++)
                {                
                    for(int i_pos = 0;  i_pos < 3; i_pos++)
                    {
                        pos1[i_pos]     =   pos2[i_pos];
                        pos2[i_pos]     =   arrWPsNED[i_wp*3 + i_pos + 3];
                        vec12[i_pos]    =   pos2[i_pos] - pos1[i_pos];
                    }
                    GetEuclideanNorm(vec12, &dist12);
                    sumDist     =   sumDist + dist12;
                    if(sumDist > lookAheadDist)
                    {
                        double unitVec[3] = {0.,};
                        double magVec = 0.;
                        for(int i_pos = 0;  i_pos < 3; i_pos++)
                        {
                            unitVec[i_pos]  =   vec12[i_pos] / dist12;
                            magVec          =   sumDist - lookAheadDist;
                            tgPosn[i_pos] =   pos2[i_pos] - magVec*unitVec[i_pos];
                        }
                        break;
                    }                
                }
                if((prevWPidx + 1 == numWPs - 1) && (sumDist <= lookAheadDist))
                {
                    for(int i_pos = 0;  i_pos < 3; i_pos++)
                    {
                        tgPosn[i_pos] =   arrWPsNED[prevWPidx*3 + i_pos + 3];
                    }
                }

            }
        }

        // quadrotor module functions
        __device__ void Kinematics(double tgPosn[3], double tgVn[3], double Posn[3], double Vn[3], double *LOSazim, double *LOSelev, double dLOSvec[3], double *relDist, double *tgo)
        {
        //.. azim, elev
            double relPosn[3]   =   {tgPosn[0] - Posn[0], tgPosn[1] - Posn[1], tgPosn[2] - Posn[2]};
            double azim     =   0;
            double elev     =   0;
            Get_Vec2AzimElev(relPosn, &azim, &elev);
            LOSazim[0]     =   azim;
            LOSelev[0]     =   elev;
        //.. relDist & tgo
            double relDist_ =   0.;
            double Spd =   0.;            
            GetEuclideanNorm(relPosn, &relDist_);
            GetEuclideanNorm(Vn, &Spd);
            relDist_        =   max(relDist_, 0.001);
            Spd             =   max(Spd, 0.1);
            relDist[0]      =   relDist_;
            tgo[0]          =   relDist_ / Spd;
        //.. dLOSvec
            double relVn[3]     =   {tgVn[0] - Vn[0], tgVn[1] - Vn[1], tgVn[2] - Vn[2]};
            double Pos_X_V[3]   =   { 0., };
            VecCross(relPosn, relVn, Pos_X_V);
            for(int i = 0; i < 3; i++)
            {
                dLOSvec[i]  =   Pos_X_V[i]/relDist_/relDist_;
            }
        }
        __device__ void Guid_pursuit(double Kgain_PG, double tgo, double LOSazim, double LOSelev, double Vn[3], double AccLim, double AccCmdw[3])
        {
        //.. lead angle
            double psi = 0.;
            double gam = 0.;
            Get_Vec2AzimElev(Vn, &psi, &gam);
            double err_psi = LOSazim - psi;
            double err_gam = LOSelev - gam;
            err_psi     =   atan2(sin(err_psi), cos(err_psi));
            err_gam     =   atan2(sin(err_gam), cos(err_gam));
            double Spd = 0.;
            GetEuclideanNorm(Vn, &Spd);
        //.. calc. acc. cmd.
            AccCmdw[1]       =   Kgain_PG * Spd * err_psi/tgo * cos(gam);
            AccCmdw[2]       =   - Kgain_PG * Spd * err_gam/tgo;
        //.. limit
            for(int i =1; i<3; i++)
            {
                AccCmdw[i]  =   min(AccCmdw[i], AccLim);
                AccCmdw[i]  =   max(AccCmdw[i], -AccLim);
            }
        }
        __device__ void SpdCtrller(double Kp, double Ki, double Kd, double *int_err_spd, double *prev_err_spd, double dt, double Vn[3], double desSpd, double *AccCmdXw)
        {
            double Spd       =   0.;
            GetEuclideanNorm(Vn, &Spd);
            double err_spd      =   desSpd - Spd;
            int_err_spd[0]      =   int_err_spd[0] + err_spd * dt;
            double derr_spd     =   0.;
            if(prev_err_spd[0] != 0.)
            {
                derr_spd     =   (err_spd - prev_err_spd[0])/dt;
            }
            prev_err_spd[0] =   err_spd;
            AccCmdXw[0]     =   Kp * err_spd + Ki * int_err_spd[0] + Kd * derr_spd;
        }
        __device__ void AccCmdToCtrlCmd(double AccCmdn[3], double psi_FPA, double throttle_Hover, double mass, double g, double AngEuler_Cmd[3], double Fcmd_b[3])
        {
            double totalAccCmdn[3]  =   { 0., };
            for(int i_a = 0; i_a < 3; i_a++)
            {
                totalAccCmdn[i_a]   =   AccCmdn[i_a];
            }
            totalAccCmdn[2]     =   totalAccCmdn[2] - g;
            double magAccCmd    =   0.;
            GetEuclideanNorm(totalAccCmdn, &magAccCmd);
            double FPAeuler[3]      =   { 0., 0., psi_FPA };
            double mat_R3[3][3]     =   { 0., };
            Get_Euler2DCM(FPAeuler, mat_R3);
            double AccCmdn_R3[3]     =   { 0., };            
            Mul_Mat33Vec3(mat_R3, totalAccCmdn, AccCmdn_R3);
            double phi          =   asin(AccCmdn_R3[1]/magAccCmd);
            double theta        =   asin(-AccCmdn_R3[0]/cos(phi)/magAccCmd);
            double psi          =   FPAeuler[2];
            AngEuler_Cmd[0]     =   phi;
            AngEuler_Cmd[1]     =   theta;
            AngEuler_Cmd[2]     =   psi;

            double magAccCmd_g  =   magAccCmd/g;
            double ThrottleCmd  =   min(magAccCmd_g*throttle_Hover, 1.);
            double AccCmdb[3]   =   {0., 0., -ThrottleCmd / throttle_Hover * g};
            Fcmd_b[2]    =   AccCmdb[2] * mass;
        }
        __device__ void Dynamics_p6dof(double Fb_tot[3], double g0, double tau_ctrl[3], double AngEuler_Cmd[3], double AngEuler[3], double Vb[3], double cI_B[3][3], double Mass, double dot_AngEuler[3], double dot_Vb[3], double dot_Posn[3])
        {
        //.. Specific Force  
            double accb[3]      =   { Fb_tot[0]/Mass, Fb_tot[1]/Mass, Fb_tot[2]/Mass };

        //.. Getting Gravitational Acceleration in Inertia Frame
            double gn[3]        =   { 0., 0., g0 };
            double gb[3]        =   { 0., };
            Mul_Mat33Vec3(cI_B, gn, gb);

        //.. Determining Kinematic Relationship beween Euler Angle and Body Rate
            double cthe            =   cos( AngEuler[1] ) ;
            double tthe            =   tan( AngEuler[1] ) ;
            double sphi            =   sin( AngEuler[0] ) ;
            double cphi            =   cos( AngEuler[0] ) ;
            double sthe            =   sin( AngEuler[1] ) ;
            double cMat[3][3]       =   {0.,};
            cMat[0][0]       =   1.0   ;
            cMat[0][1]       =   sphi * tthe   ;
            cMat[0][2]       =   cphi * tthe   ;
            cMat[1][1]       =   cphi   ;
            cMat[1][2]       =   -sphi   ;
            cMat[2][1]       =   sphi / cthe  ;
            cMat[2][2]       =   cphi / cthe  ;

        //.. Computing Dynamics - psuedo 6dof
            for(int i_d = 0; i_d < 3; i_d++)
            {
                dot_AngEuler[i_d]   =   (1./tau_ctrl[i_d])*(AngEuler_Cmd[i_d] - AngEuler[i_d]);
            }       
            double cMat_inv[3][3]   =   {0.,};
            double Wb[3]    = {0.,};
            Get_invM33(cMat, cMat_inv);;
            Mul_Mat33Vec3(cMat_inv, dot_AngEuler, Wb);
            double Wb_X_Vb[3]   = {0.,};
            VecCross(Wb, Vb, Wb_X_Vb);
            for(int i_d = 0; i_d < 3; i_d++)
            {
                dot_Vb[i_d]         =   -Wb_X_Vb[i_d] + gb[i_d] + accb[i_d];
            }
            double Vn[3]       =   {0.,};
            double cB_I[3][3]   =   { 0., };
            Get_invDCM(cI_B, cB_I);
            Mul_Mat33Vec3(cB_I, Vb, Vn);
            for(int i_d = 0; i_d < 3; i_d++)
            {
                dot_Posn[i_d]       =   Vn[i_d];
            }
        }
        __device__ void Integration_Euler(double Vb[3], double AngEuler[3], double Pos[3], double dot_Vb[3], double dot_AngEuler[3], double dot_Pos[3], double dt)
        {
            for(int i = 0; i < 3; i++)
            {
                Vb[i]       =   Vb[i] + dot_Vb[i] * dt;
                AngEuler[i] =   AngEuler[i] + dot_AngEuler[i] * dt;
                AngEuler[i] =   atan2(sin(AngEuler[i]), cos(AngEuler[i]));
                Pos[i]      =   Pos[i] + dot_Pos[i] * dt;
            }
        }
        """)

        func        =   mod.get_function("mainCuda")
        n = 32
        blocksz     =   (n, 1, 1)
        gridsz      =   (m.ceil(self.MPPIParams.K/n), 1)
        func(gpu_intMPPI,gpu_u1_MPPI,gpu_delta_u1,gpu_u2_MPPI,gpu_delta_u2,gpu_stk,gpu_delAccn,gpuWPParams,gpuWPsNED,gpuDistParams,gpuModelParams,gpuInitStates, block=blocksz, grid=gridsz)

        res_stk         =   np.empty_like(arr_stk)
        cuda.memcpy_dtoh(res_stk, gpu_stk)
        # uav.outGCU.tempvar1    =   np.mean(res_stk)
        
        # entropy - cuda
        entropy1    =   np.zeros(self.MPPIParams.N)
        entropy2    =   np.zeros(self.MPPIParams.N)
        lamb1       =   self.MPPIParams.lamb1
        lamb2       =   self.MPPIParams.lamb2
        arrNum1     =   np.zeros((self.MPPIParams.N,self.MPPIParams.K)).astype(np.float64)
        arrNum2     =   np.zeros((self.MPPIParams.N,self.MPPIParams.K)).astype(np.float64)
        arrDen1     =   np.zeros((self.MPPIParams.N,self.MPPIParams.K)).astype(np.float64)
        arrDen2     =   np.zeros((self.MPPIParams.N,self.MPPIParams.K)).astype(np.float64)
        arrLamb1    =   lamb1*np.ones(1).astype(np.float64)
        arrLamb2    =   lamb2*np.ones(1).astype(np.float64)
        gpuNum1     =   cuda.mem_alloc(arrNum1.nbytes)
        gpuNum2     =   cuda.mem_alloc(arrNum2.nbytes)
        gpuDen1     =   cuda.mem_alloc(arrDen1.nbytes)
        gpuDen2     =   cuda.mem_alloc(arrDen2.nbytes)
        gpuLamb1    =   cuda.mem_alloc(arrLamb1.nbytes)
        gpuLamb2    =   cuda.mem_alloc(arrLamb2.nbytes)
        cuda.memcpy_htod(gpuNum1, arrNum1)
        cuda.memcpy_htod(gpuNum2, arrNum2)
        cuda.memcpy_htod(gpuDen1, arrDen1)
        cuda.memcpy_htod(gpuDen2, arrDen2)
        cuda.memcpy_htod(gpuLamb1, arrLamb1)
        cuda.memcpy_htod(gpuLamb2, arrLamb2)

        mod_ent = SourceModule("""
        __global__ void Entropy(double* arrNum1, double* arrNum2, double* arrDen1, double* arrDen2, double *arrLamb1, double *arrLamb2, double *arr_delta_u1, double *arr_delta_u2, double *arr_stk)
        {            
            // parameters
            double lamb1  =   arrLamb1[0];
            double lamb2  =   arrLamb2[0];

            // index variables  
            int k       =   threadIdx.x + threadIdx.y*blockDim.x + blockIdx.x*blockDim.x*blockDim.y;
            int idx     =   threadIdx.x + threadIdx.y*blockDim.x + blockIdx.x*blockDim.x*blockDim.y + blockIdx.y*blockDim.x*blockDim.y*gridDim.x;

            // calc num, den
            arrNum1[idx]    =   exp((-1/lamb1)*arr_stk[k])*arr_delta_u1[idx];
            arrDen1[idx]     =   exp((-1/lamb1)*arr_stk[k]);
            arrNum2[idx]    =   exp((-1/lamb2)*arr_stk[k])*arr_delta_u2[idx];
            arrDen2[idx]     =   exp((-1/lamb2)*arr_stk[k]);
            // printf("|%d\t", k);
        }
        """)
        func2 = mod_ent.get_function("Entropy")
        blocksz     =   (n, 1, 1)
        gridsz      =   (m.ceil(self.MPPIParams.K/n), self.MPPIParams.N)
        func2(gpuNum1, gpuNum2, gpuDen1, gpuDen2, gpuLamb1, gpuLamb2, gpu_delta_u1, gpu_delta_u2, gpu_stk, block=blocksz, grid=gridsz)

        resNum1     =   np.empty_like(arrNum1)
        resNum2     =   np.empty_like(arrNum2)
        resDen1     =   np.empty_like(arrDen1)
        resDen2     =   np.empty_like(arrDen2)
        cuda.memcpy_dtoh(resNum1, gpuNum1)
        cuda.memcpy_dtoh(resNum2, gpuNum2)
        cuda.memcpy_dtoh(resDen1, gpuDen1)
        cuda.memcpy_dtoh(resDen2, gpuDen2)

        sumNum1     =   resNum1.sum(axis=1)
        sumNum2     =   resNum2.sum(axis=1)
        sumDen1     =   resDen1.sum(axis=1)
        sumDen2     =   resDen2.sum(axis=1)
        entropy1     =   sumNum1/sumDen1
        entropy2     =   sumNum2/sumDen2
        
        accCmd      =   np.zeros(3)
        if np.any(np.isnan(entropy1)) or np.any(np.isnan(entropy2)):
            accCmd[0]  =   2.
        else:        
            u1_MPPI       =   u1_MPPI + entropy1    
            u2_MPPI       =   u2_MPPI + entropy2

        accCmd[1]   =   u1_MPPI[1]
        accCmd[2]   =   u2_MPPI[1]
        return accCmd, u1_MPPI, u2_MPPI
