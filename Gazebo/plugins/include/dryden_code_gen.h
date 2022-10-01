/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: dryden_code_gen.h
 *
 * Code generated for Simulink model 'dryden_code_gen'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Sat Oct  1 10:39:28 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_dryden_code_gen_h_
#define RTW_HEADER_dryden_code_gen_h_
#ifndef dryden_code_gen_COMMON_INCLUDES_
#define dryden_code_gen_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* dryden_code_gen_COMMON_INCLUDES_ */

#include "dryden_code_gen_types.h"
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T LugV1[2];                     /* '<S20>/Lug//V1' */
  real_T w[2];                         /* '<S20>/w' */
  real_T w_c[2];                       /* '<S20>/w ' */
  real_T LwgV1[2];                     /* '<S20>/Lwg//V 1' */
  real_T LugV1_h[2];                   /* '<S19>/Lug//V1' */
  real_T w_e[2];                       /* '<S19>/w' */
  real_T w_i[2];                       /* '<S19>/w ' */
  real_T w1[2];                        /* '<S19>/w 1' */
  real_T LugV1_e[2];                   /* '<S18>/Lug//V1' */
  real_T w_f[2];                       /* '<S18>/w' */
  real_T w1_p[2];                      /* '<S18>/w1' */
  real_T w_ic[2];                      /* '<S17>/w' */
  real_T w_p[2];                       /* '<S16>/w' */
  real_T LugV1_k[2];                   /* '<S15>/Lug//V1' */
  real_T w_g[2];                       /* '<S15>/w' */
  real_T sigma_w[2];                   /* '<S15>/sigma_w' */
} B_dryden_code_gen_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T NextOutput[4];                /* '<S14>/White Noise' */
  uint32_T PreLookUpIndexSearchaltitude_DW;
                               /* '<S21>/PreLook-Up Index Search  (altitude)' */
  uint32_T PreLookUpIndexSearchprobofexcee;
                         /* '<S21>/PreLook-Up Index Search  (prob of exceed)' */
  uint32_T RandSeed[4];                /* '<S14>/White Noise' */
  int8_T ifHeightMaxlowaltitudeelseifHei;
  /* '<S10>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  int8_T ifHeightMaxlowaltitudeelseifH_i;
  /* '<S9>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  boolean_T Hwgws_MODE;                /* '<S5>/Hwgw(s)' */
  boolean_T Hvgws_MODE;                /* '<S5>/Hvgw(s)' */
  boolean_T Hugws_MODE;                /* '<S5>/Hugw(s)' */
  boolean_T Hrgw_MODE;                 /* '<S4>/Hrgw' */
  boolean_T Hqgw_MODE;                 /* '<S4>/Hqgw' */
  boolean_T Hpgw_MODE;                 /* '<S4>/Hpgw' */
} DW_dryden_code_gen_T;

/* Continuous states (default storage) */
typedef struct {
  real_T wg_p1_CSTATE[2];              /* '<S20>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S20>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S19>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S19>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S18>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S17>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S16>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S15>/pgw_p' */
} X_dryden_code_gen_T;

/* State derivatives (default storage) */
typedef struct {
  real_T wg_p1_CSTATE[2];              /* '<S20>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S20>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S19>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S19>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S18>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S17>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S16>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S15>/pgw_p' */
} XDot_dryden_code_gen_T;

/* State disabled  */
typedef struct {
  boolean_T wg_p1_CSTATE[2];           /* '<S20>/wg_p1' */
  boolean_T wg_p2_CSTATE[2];           /* '<S20>/wg_p2' */
  boolean_T vg_p1_CSTATE[2];           /* '<S19>/vg_p1' */
  boolean_T vgw_p2_CSTATE[2];          /* '<S19>/vgw_p2' */
  boolean_T ug_p_CSTATE[2];            /* '<S18>/ug_p' */
  boolean_T rgw_p_CSTATE[2];           /* '<S17>/rgw_p' */
  boolean_T qgw_p_CSTATE[2];           /* '<S16>/qgw_p' */
  boolean_T pgw_p_CSTATE[2];           /* '<S15>/pgw_p' */
} XDis_dryden_code_gen_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T UnitConversion;         /* '<S3>/Unit Conversion' */
  const real_T UnitConversion_a;       /* '<S6>/Unit Conversion' */
  const real_T LimitFunction10ftto1000ft;
                                     /* '<S39>/Limit Function 10ft to 1000ft' */
  const real_T UnitConversion_h;       /* '<S41>/Unit Conversion' */
  const real_T UnitConversion_o;       /* '<S13>/Unit Conversion' */
  const real_T sigma_wg;               /* '<S22>/sigma_wg ' */
  const real_T PreLookUpIndexSearchaltitud;
                               /* '<S21>/PreLook-Up Index Search  (altitude)' */
  const real_T PreLookUpIndexSearchprobofe;
                         /* '<S21>/PreLook-Up Index Search  (prob of exceed)' */
  const real_T MediumHighAltitudeIntensity;
                                   /* '<S21>/Medium//High Altitude Intensity' */
  const real_T UnitConversion_c;       /* '<S12>/Unit Conversion' */
  const real_T UnitConversion_a5;      /* '<S7>/Unit Conversion' */
  const real_T LowAltitudeScaleLength; /* '<S39>/Low Altitude Scale Length' */
  const real_T LimitHeighth1000ft;     /* '<S22>/Limit Height h<1000ft' */
  const real_T LowAltitudeIntensity;   /* '<S22>/Low Altitude Intensity' */
  const real_T sigma_ugsigma_vg;       /* '<S22>/sigma_ug, sigma_vg' */
  const real_T TmpSignalConversionAtsincos[3];
  const real_T sincos_o1[3];           /* '<S2>/sincos' */
  const real_T sincos_o2[3];           /* '<S2>/sincos' */
  const real_T Fcn11;                  /* '<S2>/Fcn11' */
  const real_T Fcn21;                  /* '<S2>/Fcn21' */
  const real_T Fcn31;                  /* '<S2>/Fcn31' */
  const real_T Fcn12;                  /* '<S2>/Fcn12' */
  const real_T Fcn22;                  /* '<S2>/Fcn22' */
  const real_T Fcn32;                  /* '<S2>/Fcn32' */
  const real_T Fcn13;                  /* '<S2>/Fcn13' */
  const real_T Fcn23;                  /* '<S2>/Fcn23' */
  const real_T Fcn33;                  /* '<S2>/Fcn33' */
  const real_T VectorConcatenate[9];   /* '<S42>/Vector Concatenate' */
  const real_T Sqrt[4];                /* '<S14>/Sqrt' */
  const real_T Sqrt1;                  /* '<S14>/Sqrt1' */
  const real_T Divide[4];              /* '<S14>/Divide' */
  const real_T Sum;                    /* '<S31>/Sum' */
  const real_T Sum1;                   /* '<S31>/Sum1' */
  const real_T TrigonometricFunction_o1;/* '<S36>/Trigonometric Function' */
  const real_T TrigonometricFunction_o2;/* '<S36>/Trigonometric Function' */
  const real_T TrigonometricFunction_o1_p;/* '<S38>/Trigonometric Function' */
  const real_T TrigonometricFunction_o2_c;/* '<S38>/Trigonometric Function' */
  const real_T Sum_n;                  /* '<S23>/Sum' */
  const real_T Sum1_g;                 /* '<S23>/Sum1' */
  const real_T TrigonometricFunction_o1_px;/* '<S28>/Trigonometric Function' */
  const real_T TrigonometricFunction_o2_e;/* '<S28>/Trigonometric Function' */
  const real_T TrigonometricFunction1_o1;/* '<S30>/Trigonometric Function1' */
  const real_T TrigonometricFunction1_o2;/* '<S30>/Trigonometric Function1' */
  const real_T LwgV[2];                /* '<S20>/Lwg//V' */
  const real_T upi[2];                 /* '<S20>/1//pi' */
  const real_T sqrt_o;                 /* '<S20>/sqrt' */
  const real_T sqrt1[2];               /* '<S20>/sqrt1' */
  const real_T LvgV[2];                /* '<S19>/Lvg//V' */
  const real_T upi_i[2];               /* '<S19>/(1//pi)' */
  const real_T sqrt_b[2];              /* '<S19>/sqrt' */
  const real_T LugV[2];                /* '<S18>/Lug//V' */
  const real_T upi_g[2];               /* '<S18>/(2//pi)' */
  const real_T sqrt_n[2];              /* '<S18>/sqrt' */
  const real_T pi3;                    /* '<S17>/pi//3' */
  const real_T pi4;                    /* '<S16>/pi//4' */
  const real_T TmpSignalConversionAtL13Inp[2];
  const real_T L13[2];                 /* '<S15>/L^1//3' */
  const real_T sqrt08V;                /* '<S15>/sqrt(0.8//V)' */
  const real_T w4;                     /* '<S15>/w4' */
  const real_T u16;                    /* '<S15>/u^1//6' */
  const real_T w1[2];                  /* '<S15>/w1' */
  const real_T w2[2];                  /* '<S15>/w2' */
  const real_T w3;                     /* '<S15>/w3' */
  const uint32_T PreLookUpIndexSearchaltit_i;
                               /* '<S21>/PreLook-Up Index Search  (altitude)' */
  const uint32_T PreLookUpIndexSearchprobo_o;
                         /* '<S21>/PreLook-Up Index Search  (prob of exceed)' */
} ConstB_dryden_code_gen_T;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T uvw_ptr[3];                   /* '<Root>/uvw_ptr' */
  real_T pqr_ptr[3];                   /* '<Root>/pqr_ptr' */
} ExtY_dryden_code_gen_T;

/* Real-time Model Data Structure */
struct tag_RTM_dryden_code_gen_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_dryden_code_gen_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[16];
  real_T odeF[4][16];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
extern B_dryden_code_gen_T dryden_code_gen_B;

/* Continuous states (default storage) */
extern X_dryden_code_gen_T dryden_code_gen_X;

/* Block states (default storage) */
extern DW_dryden_code_gen_T dryden_code_gen_DW;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_dryden_code_gen_T dryden_code_gen_Y;
extern const ConstB_dryden_code_gen_T dryden_code_gen_ConstB;/* constant block i/o */

/* Model entry point functions */
extern void dryden_code_gen_initialize(void);
extern void dryden_code_gen_step(void);
extern void dryden_code_gen_terminate(void);

/* Real-time Model object */
extern RT_MODEL_dryden_code_gen_T *const dryden_code_gen_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<Root>/Scope1' : Unused code path elimination
 * Block '<S1>/Cast' : Eliminate redundant data type conversion
 * Block '<S1>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S1>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S1>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S1>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S1>/Cast To Double4' : Eliminate redundant data type conversion
 * Block '<S27>/Reshape' : Reshape block reduction
 * Block '<S27>/Reshape1' : Reshape block reduction
 * Block '<S29>/Reshape' : Reshape block reduction
 * Block '<S35>/Reshape' : Reshape block reduction
 * Block '<S35>/Reshape1' : Reshape block reduction
 * Block '<S37>/Reshape' : Reshape block reduction
 * Block '<S11>/Lv' : Eliminated nontunable gain of 1
 * Block '<S11>/Lw' : Eliminated nontunable gain of 1
 * Block '<S42>/Reshape (9) to [3x3] column-major' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'dryden_code_gen'
 * '<S1>'   : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))'
 * '<S2>'   : 'dryden_code_gen/Rotation Angles to Direction Cosine Matrix'
 * '<S3>'   : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Angle Conversion'
 * '<S4>'   : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates'
 * '<S5>'   : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities'
 * '<S6>'   : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Length Conversion'
 * '<S7>'   : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Length Conversion1'
 * '<S8>'   : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities'
 * '<S9>'   : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates'
 * '<S10>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities'
 * '<S11>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Turbulence scale lengths'
 * '<S12>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Velocity Conversion'
 * '<S13>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Velocity Conversion2'
 * '<S14>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/White Noise'
 * '<S15>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hpgw'
 * '<S16>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hqgw'
 * '<S17>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on angular rates/Hrgw'
 * '<S18>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hugw(s)'
 * '<S19>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hvgw(s)'
 * '<S20>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Filters on velocities/Hwgw(s)'
 * '<S21>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/High Altitude Intensity'
 * '<S22>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/RMS turbulence  intensities/Low Altitude Intensity'
 * '<S23>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Interpolate  rates'
 * '<S24>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Low altitude  rates'
 * '<S25>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Medium//High  altitude rates'
 * '<S26>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Merge Subsystems'
 * '<S27>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Interpolate  rates/wind to body transformation'
 * '<S28>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Interpolate  rates/wind to body transformation/convert to earth coords'
 * '<S29>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Low altitude  rates/wind to body transformation'
 * '<S30>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select angular rates/Low altitude  rates/wind to body transformation/convert to earth coords'
 * '<S31>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Interpolate  velocities'
 * '<S32>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Low altitude  velocities'
 * '<S33>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Medium//High  altitude velocities'
 * '<S34>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Merge Subsystems'
 * '<S35>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Interpolate  velocities/wind to body transformation'
 * '<S36>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Interpolate  velocities/wind to body transformation/convert to earth coords'
 * '<S37>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Low altitude  velocities/wind to body transformation'
 * '<S38>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Select velocities/Low altitude  velocities/wind to body transformation/convert to earth coords'
 * '<S39>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Turbulence scale lengths/Low altitude scale length'
 * '<S40>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Turbulence scale lengths/Medium//High altitude scale length'
 * '<S41>'  : 'dryden_code_gen/Dryden Wind Turbulence Model  (Continuous (+q +r))/Turbulence scale lengths/Medium//High altitude scale length/Length Conversion'
 * '<S42>'  : 'dryden_code_gen/Rotation Angles to Direction Cosine Matrix/Create 3x3 Matrix'
 */
#endif                                 /* RTW_HEADER_dryden_code_gen_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
