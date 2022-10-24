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
 * C/C++ source code generated on : Mon Oct 10 16:08:30 2022
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
#include "rt_nonfinite.h"

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
  real_T Product[4];                   /* '<S14>/Product' */
  real_T w[2];                         /* '<S20>/w' */
  real_T w_c[2];                       /* '<S20>/w ' */
  real_T LwgV1[2];                     /* '<S20>/Lwg//V 1' */
  real_T w_e[2];                       /* '<S19>/w' */
  real_T w_i[2];                       /* '<S19>/w ' */
  real_T w1[2];                        /* '<S19>/w 1' */
  real_T w_f[2];                       /* '<S18>/w' */
  real_T w1_p[2];                      /* '<S18>/w1' */
  real_T w_ic[2];                      /* '<S17>/w' */
  real_T w_p[2];                       /* '<S16>/w' */
  real_T w_g[2];                       /* '<S15>/w' */
  real_T sigma_w[2];                   /* '<S15>/sigma_w' */
} B_dryden_code_gen_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T NextOutput[4];                /* '<S14>/White Noise' */
  uint32_T PreLookUpIndexSearchprobofexcee;
                         /* '<S21>/PreLook-Up Index Search  (prob of exceed)' */
  uint32_T PreLookUpIndexSearchaltitude_DW;
                               /* '<S21>/PreLook-Up Index Search  (altitude)' */
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
  const real_T UnitConversion_o;       /* '<S13>/Unit Conversion' */
  const real_T sigma_wg;               /* '<S22>/sigma_wg ' */
  const real_T UnitConversion_a;       /* '<S7>/Unit Conversion' */
  const real_T UnitConversion_h;       /* '<S41>/Unit Conversion' */
  const real_T PreLookUpIndexSearchprobofe;
                         /* '<S21>/PreLook-Up Index Search  (prob of exceed)' */
  const real_T Sqrt[4];                /* '<S14>/Sqrt' */
  const real_T Sqrt1;                  /* '<S14>/Sqrt1' */
  const real_T Divide[4];              /* '<S14>/Divide' */
  const real_T Sum;                    /* '<S31>/Sum' */
  const real_T Sum_n;                  /* '<S23>/Sum' */
  const real_T sqrt_o;                 /* '<S20>/sqrt' */
  const real_T w4;                     /* '<S15>/w4' */
  const real_T u16;                    /* '<S15>/u^1//6' */
  const uint32_T PreLookUpIndexSearchprobo_o;
                         /* '<S21>/PreLook-Up Index Search  (prob of exceed)' */
} ConstB_dryden_code_gen_T;

#ifndef ODE8_INTG
#define ODE8_INTG

/* ODE8 Integration Data */
typedef struct {
  real_T *deltaY;                      /* output diff */
  real_T *f[13];                       /* derivatives */
  real_T *x0;                          /* Initial State */
} ODE8_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: h_vec
   * Referenced by: '<S21>/PreLook-Up Index Search  (altitude)'
   */
  real_T PreLookUpIndexSearchaltitude_Br[12];

  /* Expression: sigma_vec'
   * Referenced by: '<S21>/Medium//High Altitude Intensity'
   */
  real_T MediumHighAltitudeIntensity_Tab[84];

  /* Computed Parameter: MediumHighAltitudeIntensity_max
   * Referenced by: '<S21>/Medium//High Altitude Intensity'
   */
  uint32_T MediumHighAltitudeIntensity_max[2];
} ConstP_dryden_code_gen_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Altitude;                     /* '<Root>/Altitude' */
  real_T Velocity;                     /* '<Root>/Velocity' */
  real_T w;                            /* '<Root>/w' */
  real_T x;                            /* '<Root>/x' */
  real_T y;                            /* '<Root>/y' */
  real_T z;                            /* '<Root>/z' */
} ExtU_dryden_code_gen_T;

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
  real_T OdeDeltaY[16];
  real_T odeF[13][16];
  real_T odeX0[16];
  ODE8_IntgData intgData;

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

/* External inputs (root inport signals with default storage) */
extern ExtU_dryden_code_gen_T dryden_code_gen_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_dryden_code_gen_T dryden_code_gen_Y;
extern const ConstB_dryden_code_gen_T dryden_code_gen_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_dryden_code_gen_T dryden_code_gen_ConstP;

/* Model entry point functions */
extern void dryden_code_gen_initialize(void);
extern void dryden_code_gen_step(void);
extern void dryden_code_gen_terminate(void);

/* Real-time Model object */
extern RT_MODEL_dryden_code_gen_T *const dryden_code_gen_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
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
 * Block '<S51>/Reshape (9) to [3x3] column-major' : Reshape block reduction
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
 * '<S2>'   : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix'
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
 * '<S42>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/A11'
 * '<S43>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/A12'
 * '<S44>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/A13'
 * '<S45>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/A21'
 * '<S46>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/A22'
 * '<S47>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/A23'
 * '<S48>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/A31'
 * '<S49>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/A32'
 * '<S50>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/A33'
 * '<S51>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/Create 3x3 Matrix'
 * '<S52>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
 * '<S53>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
 * '<S54>'  : 'dryden_code_gen/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 */
#endif                                 /* RTW_HEADER_dryden_code_gen_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */