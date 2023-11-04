/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: dryden_code_gen.c
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

#include "dryden_code_gen.h"
#include "rtwtypes.h"
#include "dryden_code_gen_private.h"
#include <math.h>
#include <emmintrin.h>
#include "rt_nonfinite.h"

/* Block signals (default storage) */
B_dryden_code_gen_T dryden_code_gen_B;

/* Continuous states */
X_dryden_code_gen_T dryden_code_gen_X;

/* Block states (default storage) */
DW_dryden_code_gen_T dryden_code_gen_DW;

/* External inputs (root inport signals with default storage) */
ExtU_dryden_code_gen_T dryden_code_gen_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_dryden_code_gen_T dryden_code_gen_Y;

/* Real-time model */
static RT_MODEL_dryden_code_gen_T dryden_code_gen_M_;
RT_MODEL_dryden_code_gen_T *const dryden_code_gen_M = &dryden_code_gen_M_;
uint32_T plook_bincpa(real_T u, const real_T bp[], uint32_T maxIndex, real_T
                      *fraction, uint32_T *prevIndex)
{
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'on'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32d_prevIdx(u, bp, *prevIndex, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex;
    *fraction = 0.0;
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

real_T intrp2d_la_pw(const uint32_T bpIndex[], const real_T frac[], const real_T
                     table[], const uint32_T stride, const uint32_T maxIndex[])
{
  real_T y;
  real_T yL_0d0;
  uint32_T offset_1d;

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Overflow mode: 'portable wrapping'
   */
  offset_1d = bpIndex[1U] * stride + bpIndex[0U];
  if (bpIndex[0U] == maxIndex[0U]) {
    y = table[offset_1d];
  } else {
    yL_0d0 = table[offset_1d];
    y = (table[offset_1d + 1U] - yL_0d0) * frac[0U] + yL_0d0;
  }

  if (bpIndex[1U] == maxIndex[1U]) {
  } else {
    offset_1d += stride;
    if (bpIndex[0U] == maxIndex[0U]) {
      yL_0d0 = table[offset_1d];
    } else {
      yL_0d0 = table[offset_1d];
      yL_0d0 += (table[offset_1d + 1U] - yL_0d0) * frac[0U];
    }

    y += (yL_0d0 - y) * frac[1U];
  }

  return y;
}

uint32_T binsearch_u32d_prevIdx(real_T u, const real_T bp[], uint32_T startIndex,
  uint32_T maxIndex)
{
  uint32_T bpIndex;
  uint32_T found;
  uint32_T iLeft;
  uint32_T iRght;

  /* Binary Search using Previous Index */
  bpIndex = startIndex;
  iLeft = 0U;
  iRght = maxIndex;
  found = 0U;
  while (found == 0U) {
    if (u < bp[bpIndex]) {
      iRght = bpIndex - 1U;
      bpIndex = ((bpIndex + iLeft) - 1U) >> 1U;
    } else if (u < bp[bpIndex + 1U]) {
      found = 1U;
    } else {
      iLeft = bpIndex + 1U;
      bpIndex = ((bpIndex + iRght) + 1U) >> 1U;
    }
  }

  return bpIndex;
}

/*
 * This function updates continuous states using the ODE8 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
#define dryden_code_gen_NSTAGES        13

  static real_T rt_ODE8_B[13] = {
    4.174749114153025E-2, 0.0, 0.0, 0.0,
    0.0, -5.54523286112393E-2, 2.393128072011801E-1, 7.03510669403443E-1,
    -7.597596138144609E-1, 6.605630309222863E-1, 1.581874825101233E-1,
    -2.381095387528628E-1, 2.5E-1
  };

  static real_T rt_ODE8_C[13] = {
    0.0, 5.555555555555556E-2, 8.333333333333333E-2, 1.25E-1,
    3.125E-1, 3.75E-1, 1.475E-1, 4.65E-1,
    5.648654513822596E-1, 6.5E-1, 9.246562776405044E-1, 1.0, 1.0
  };

  static real_T rt_ODE8_A[13][13] = {
    /* rt_ODE8_A[0][] */
    { 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0 },

    /* rt_ODE8_A[1][] */
    { 5.555555555555556E-2, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0 },

    /* rt_ODE8_A[2][] */
    { 2.083333333333333E-2, 6.25E-2, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0 },

    /* rt_ODE8_A[3][] */
    { 3.125E-2, 0.0, 9.375E-2, 0.0,
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0 },

    /* rt_ODE8_A[4][] */
    { 3.125E-1, 0.0, -1.171875, 1.171875,
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0 },

    /* rt_ODE8_A[5][] */
    { 3.75E-2, 0.0, 0.0, 1.875E-1,
      1.5E-1, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0 },

    /* rt_ODE8_A[6][] */
    { 4.791013711111111E-2, 0.0, 0.0, 1.122487127777778E-1,
      -2.550567377777778E-2, 1.284682388888889E-2, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0 },

    /* rt_ODE8_A[7][] */
    { 1.691798978729228E-2, 0.0, 0.0, 3.878482784860432E-1,
      3.597736985150033E-2, 1.969702142156661E-1, -1.727138523405018E-1, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0 },

    /* rt_ODE8_A[8][] */
    { 6.90957533591923E-2, 0.0, 0.0, -6.342479767288542E-1,
      -1.611975752246041E-1, 1.386503094588253E-1, 9.409286140357563E-1,
      2.11636326481944E-1,
      0.0, 0.0, 0.0, 0.0, 0.0 },

    /* rt_ODE8_A[9][] */
    { 1.835569968390454E-1, 0.0, 0.0, -2.468768084315592,
      -2.912868878163005E-1, -2.647302023311738E-2, 2.8478387641928,
      2.813873314698498E-1,
      1.237448998633147E-1, 0.0, 0.0, 0.0, 0.0 },

    /* rt_ODE8_A[10][] */
    { -1.215424817395888, 0.0, 0.0, 1.667260866594577E1,
      9.157418284168179E-1, -6.056605804357471, -1.600357359415618E1,
      1.484930308629766E1,
      -1.337157573528985E1, 5.134182648179638, 0.0, 0.0, 0.0 },

    /* rt_ODE8_A[11][] */
    { 2.588609164382643E-1, 0.0, 0.0, -4.774485785489205,
      -4.350930137770325E-1, -3.049483332072241, 5.577920039936099,
      6.155831589861039,
      -5.062104586736938, 2.193926173180679, 1.346279986593349E-1, 0.0, 0.0 },

    /* rt_ODE8_A[12][] */
    { 8.224275996265075E-1, 0.0, 0.0, -1.165867325727766E1,
      -7.576221166909362E-1, 7.139735881595818E-1, 1.207577498689006E1,
      -2.127659113920403,
      1.990166207048956, -2.342864715440405E-1, 1.758985777079423E-1, 0.0, 0.0 },
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE8_IntgData *intgData = (ODE8_IntgData *)rtsiGetSolverData(si);
  real_T *deltaY = intgData->deltaY;
  real_T *x0 = intgData->x0;
  real_T* f[dryden_code_gen_NSTAGES];
  int idx,stagesIdx,statesIdx;
  double deltaX;
  int_T nXc = 16;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  f[0] = intgData->f[0];
  f[1] = intgData->f[1];
  f[2] = intgData->f[2];
  f[3] = intgData->f[3];
  f[4] = intgData->f[4];
  f[5] = intgData->f[5];
  f[6] = intgData->f[6];
  f[7] = intgData->f[7];
  f[8] = intgData->f[8];
  f[9] = intgData->f[9];
  f[10] = intgData->f[10];
  f[11] = intgData->f[11];
  f[12] = intgData->f[12];

  /* Save the state values at time t in y and x0*/
  (void) memset(deltaY, 0,
                (uint_T)nXc*sizeof(real_T));
  (void) memcpy(x0, x,
                nXc*sizeof(real_T));
  for (stagesIdx=0;stagesIdx<dryden_code_gen_NSTAGES;stagesIdx++) {
    (void) memcpy(x, x0,
                  (uint_T)nXc*sizeof(real_T));
    for (statesIdx=0;statesIdx<nXc;statesIdx++) {
      deltaX = 0;
      for (idx=0;idx<stagesIdx;idx++) {
        deltaX = deltaX + h*rt_ODE8_A[stagesIdx][idx]*f[idx][statesIdx];
      }

      x[statesIdx] = x0[statesIdx] + deltaX;
    }

    if (stagesIdx==0) {
      rtsiSetdX(si, f[stagesIdx]);
      dryden_code_gen_derivatives();
    } else {
      (stagesIdx==dryden_code_gen_NSTAGES-1)? rtsiSetT(si, tnew) : rtsiSetT(si,
          t + h*rt_ODE8_C[stagesIdx]);
      rtsiSetdX(si, f[stagesIdx]);
      dryden_code_gen_step();
      dryden_code_gen_derivatives();
    }

    for (statesIdx=0;statesIdx<nXc;statesIdx++) {
      deltaY[statesIdx] = deltaY[statesIdx] + h*rt_ODE8_B[stagesIdx]*f[stagesIdx]
        [statesIdx];
    }
  }

  for (statesIdx=0;statesIdx<nXc;statesIdx++) {
    x[statesIdx] = x0[statesIdx] + deltaY[statesIdx];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T hi;
  uint32_T lo;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return (real_T)*u * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T si;
  real_T sr;
  real_T y;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

/* Model step function */
void dryden_code_gen_step(void)
{
  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  real_T rtb_VectorConcatenate[9];
  real_T rtb_VectorConcatenate_c[3];
  real_T frac[2];
  real_T rtb_LowAltitudeScaleLength;
  real_T rtb_MediumHighAltitudeIntensity;
  real_T rtb_Product3_p;
  real_T rtb_Product_f;
  real_T rtb_UnitConversion;
  real_T rtb_UnitConversion_g;
  real_T rtb_VectorConcatenate_tmp;
  real_T rtb_VectorConcatenate_tmp_0;
  real_T rtb_VectorConcatenate_tmp_1;
  real_T rtb_VectorConcatenate_tmp_2;
  real_T rtb_VectorConcatenate_tmp_3;
  real_T rtb_VectorConcatenate_tmp_4;
  real_T rtb_VectorConcatenate_tmp_5;
  real_T rtb_VectorConcatenate_tmp_6;
  real_T rtb_sigma_ugsigma_vg;
  int32_T i;
  uint32_T bpIndex[2];
  if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&dryden_code_gen_M->solverInfo,
                          ((dryden_code_gen_M->Timing.clockTick0+1)*
      dryden_code_gen_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(dryden_code_gen_M)) {
    dryden_code_gen_M->Timing.t[0] = rtsiGetT(&dryden_code_gen_M->solverInfo);
  }

  /* UnitConversion: '<S6>/Unit Conversion' incorporates:
   *  Inport: '<Root>/Altitude'
   */
  /* Unit Conversion - from: m to: ft
     Expression: output = (3.28084*input) + (0) */
  rtb_UnitConversion = 3.280839895013123 * dryden_code_gen_U.Altitude;

  /* UnitConversion: '<S12>/Unit Conversion' incorporates:
   *  Inport: '<Root>/Velocity'
   */
  /* Unit Conversion - from: m/s to: ft/s
     Expression: output = (3.28084*input) + (0) */
  rtb_UnitConversion_g = 3.280839895013123 * dryden_code_gen_U.Velocity;

  /* Saturate: '<S39>/Limit Function 10ft to 1000ft' incorporates:
   *  Saturate: '<S22>/Limit Height h<1000ft'
   */
  if (rtb_UnitConversion > 1000.0) {
    rtb_Product_f = 1000.0;
    rtb_Product3_p = 1000.0;
  } else {
    if (rtb_UnitConversion < 10.0) {
      rtb_Product_f = 10.0;
    } else {
      rtb_Product_f = rtb_UnitConversion;
    }

    if (rtb_UnitConversion < 0.0) {
      rtb_Product3_p = 0.0;
    } else {
      rtb_Product3_p = rtb_UnitConversion;
    }
  }

  /* End of Saturate: '<S39>/Limit Function 10ft to 1000ft' */

  /* Fcn: '<S39>/Low Altitude Scale Length' */
  rtb_LowAltitudeScaleLength = rtb_Product_f / rt_powd_snf(0.000823 *
    rtb_Product_f + 0.177, 1.2);

  /* Product: '<S22>/sigma_ug, sigma_vg' incorporates:
   *  Fcn: '<S22>/Low Altitude Intensity'
   */
  rtb_sigma_ugsigma_vg = 1.0 / rt_powd_snf(0.000823 * rtb_Product3_p + 0.177,
    0.4) * dryden_code_gen_ConstB.sigma_wg;

  /* Interpolation_n-D: '<S21>/Medium//High Altitude Intensity' incorporates:
   *  PreLookup: '<S21>/PreLook-Up Index Search  (altitude)'
   */
  bpIndex[0] = plook_bincpa(rtb_UnitConversion,
    dryden_code_gen_ConstP.PreLookUpIndexSearchaltitude_Br, 11U, &rtb_Product3_p,
    &dryden_code_gen_DW.PreLookUpIndexSearchaltitude_DW);
  frac[0] = rtb_Product3_p;
  frac[1] = dryden_code_gen_ConstB.PreLookUpIndexSearchprobofe;
  bpIndex[1] = dryden_code_gen_ConstB.PreLookUpIndexSearchprobo_o;
  rtb_MediumHighAltitudeIntensity = intrp2d_la_pw(bpIndex, frac,
    dryden_code_gen_ConstP.MediumHighAltitudeIntensity_Tab, 12U,
    dryden_code_gen_ConstP.MediumHighAltitudeIntensity_max);

  /* Outputs for Enabled SubSystem: '<S5>/Hvgw(s)' incorporates:
   *  EnablePort: '<S19>/Enable'
   */
  if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
    /* RandomNumber: '<S14>/White Noise' */
    dryden_code_gen_B.Product[0] = dryden_code_gen_DW.NextOutput[0];

    /* Product: '<S14>/Product' incorporates:
     *  RandomNumber: '<S14>/White Noise'
     */
    dryden_code_gen_B.Product[0] *= dryden_code_gen_ConstB.Divide[0];

    /* RandomNumber: '<S14>/White Noise' */
    dryden_code_gen_B.Product[1] = dryden_code_gen_DW.NextOutput[1];

    /* Product: '<S14>/Product' incorporates:
     *  RandomNumber: '<S14>/White Noise'
     */
    dryden_code_gen_B.Product[1] *= dryden_code_gen_ConstB.Divide[1];

    /* RandomNumber: '<S14>/White Noise' */
    dryden_code_gen_B.Product[2] = dryden_code_gen_DW.NextOutput[2];

    /* Product: '<S14>/Product' incorporates:
     *  RandomNumber: '<S14>/White Noise'
     */
    dryden_code_gen_B.Product[2] *= dryden_code_gen_ConstB.Divide[2];

    /* RandomNumber: '<S14>/White Noise' */
    dryden_code_gen_B.Product[3] = dryden_code_gen_DW.NextOutput[3];

    /* Product: '<S14>/Product' incorporates:
     *  RandomNumber: '<S14>/White Noise'
     */
    dryden_code_gen_B.Product[3] *= dryden_code_gen_ConstB.Divide[3];

    /* Outputs for Enabled SubSystem: '<S5>/Hugw(s)' incorporates:
     *  EnablePort: '<S18>/Enable'
     */
    if (rtsiIsModeUpdateTimeStep(&dryden_code_gen_M->solverInfo) &&
        (!dryden_code_gen_DW.Hugws_MODE)) {
      /* InitializeConditions for Integrator: '<S18>/ug_p' */
      dryden_code_gen_X.ug_p_CSTATE[0] = 0.0;
      dryden_code_gen_X.ug_p_CSTATE[1] = 0.0;
      dryden_code_gen_DW.Hugws_MODE = true;
    }

    /* End of Outputs for SubSystem: '<S5>/Hugw(s)' */
    if (rtsiIsModeUpdateTimeStep(&dryden_code_gen_M->solverInfo) &&
        (!dryden_code_gen_DW.Hvgws_MODE)) {
      /* InitializeConditions for Integrator: '<S19>/vg_p1' */
      dryden_code_gen_X.vg_p1_CSTATE[0] = 0.0;

      /* InitializeConditions for Integrator: '<S19>/vgw_p2' */
      dryden_code_gen_X.vgw_p2_CSTATE[0] = 0.0;

      /* InitializeConditions for Integrator: '<S19>/vg_p1' */
      dryden_code_gen_X.vg_p1_CSTATE[1] = 0.0;

      /* InitializeConditions for Integrator: '<S19>/vgw_p2' */
      dryden_code_gen_X.vgw_p2_CSTATE[1] = 0.0;
      dryden_code_gen_DW.Hvgws_MODE = true;
    }
  }

  /* End of Outputs for SubSystem: '<S5>/Hvgw(s)' */

  /* Outputs for Enabled SubSystem: '<S5>/Hugw(s)' incorporates:
   *  EnablePort: '<S18>/Enable'
   */
  if (dryden_code_gen_DW.Hugws_MODE) {
    /* Product: '<S18>/Lug//V' */
    frac[0] = rtb_LowAltitudeScaleLength / rtb_UnitConversion_g;
    frac[1] = dryden_code_gen_ConstB.UnitConversion_h / rtb_UnitConversion_g;

    /* Product: '<S18>/w' incorporates:
     *  Gain: '<S18>/(2//pi)'
     *  Integrator: '<S18>/ug_p'
     *  Product: '<S18>/Lug//V1'
     *  Sqrt: '<S18>/sqrt'
     *  Sum: '<S18>/Sum'
     */
    dryden_code_gen_B.w_f[0] = (sqrt(0.63661977236758138 * frac[0]) *
      dryden_code_gen_B.Product[0] - dryden_code_gen_X.ug_p_CSTATE[0]) / frac[0];
    dryden_code_gen_B.w_f[1] = (sqrt(0.63661977236758138 * frac[1]) *
      dryden_code_gen_B.Product[0] - dryden_code_gen_X.ug_p_CSTATE[1]) / frac[1];

    /* Product: '<S18>/w1' incorporates:
     *  Integrator: '<S18>/ug_p'
     */
    dryden_code_gen_B.w1_p[0] = dryden_code_gen_X.ug_p_CSTATE[0] *
      rtb_sigma_ugsigma_vg;
    dryden_code_gen_B.w1_p[1] = dryden_code_gen_X.ug_p_CSTATE[1] *
      rtb_MediumHighAltitudeIntensity;
  }

  /* End of Outputs for SubSystem: '<S5>/Hugw(s)' */

  /* Outputs for Enabled SubSystem: '<S5>/Hvgw(s)' incorporates:
   *  EnablePort: '<S19>/Enable'
   */
  if (dryden_code_gen_DW.Hvgws_MODE) {
    /* Product: '<S19>/Lvg//V' incorporates:
     *  Gain: '<S11>/Lv'
     */
    frac[0] = rtb_LowAltitudeScaleLength / rtb_UnitConversion_g;
    frac[1] = dryden_code_gen_ConstB.UnitConversion_h / rtb_UnitConversion_g;

    /* Product: '<S19>/w' incorporates:
     *  Gain: '<S19>/(1//pi)'
     *  Integrator: '<S19>/vg_p1'
     *  Product: '<S19>/Lug//V1'
     *  Sqrt: '<S19>/sqrt'
     *  Sum: '<S19>/Sum'
     */
    dryden_code_gen_B.w_e[0] = (sqrt(0.31830988618379069 * frac[0]) *
      dryden_code_gen_B.Product[1] - dryden_code_gen_X.vg_p1_CSTATE[0]) / frac[0];

    /* Product: '<S19>/w ' incorporates:
     *  Gain: '<S19>/(1//pi)'
     *  Gain: '<S19>/sqrt(3)'
     *  Integrator: '<S19>/vg_p1'
     *  Integrator: '<S19>/vgw_p2'
     *  Product: '<S19>/Lvg//V '
     *  Sum: '<S19>/Sum1'
     */
    dryden_code_gen_B.w_i[0] = (dryden_code_gen_B.w_e[0] * frac[0] *
      1.7320508075688772 + (dryden_code_gen_X.vg_p1_CSTATE[0] -
      dryden_code_gen_X.vgw_p2_CSTATE[0])) / frac[0];

    /* Product: '<S19>/w' incorporates:
     *  Gain: '<S19>/(1//pi)'
     *  Integrator: '<S19>/vg_p1'
     *  Product: '<S19>/Lug//V1'
     *  Sqrt: '<S19>/sqrt'
     *  Sum: '<S19>/Sum'
     */
    dryden_code_gen_B.w_e[1] = (sqrt(0.31830988618379069 * frac[1]) *
      dryden_code_gen_B.Product[1] - dryden_code_gen_X.vg_p1_CSTATE[1]) / frac[1];

    /* Product: '<S19>/w ' incorporates:
     *  Gain: '<S19>/(1//pi)'
     *  Gain: '<S19>/sqrt(3)'
     *  Integrator: '<S19>/vg_p1'
     *  Integrator: '<S19>/vgw_p2'
     *  Product: '<S19>/Lvg//V '
     *  Sum: '<S19>/Sum1'
     */
    dryden_code_gen_B.w_i[1] = (dryden_code_gen_B.w_e[1] * frac[1] *
      1.7320508075688772 + (dryden_code_gen_X.vg_p1_CSTATE[1] -
      dryden_code_gen_X.vgw_p2_CSTATE[1])) / frac[1];

    /* Product: '<S19>/w 1' incorporates:
     *  Integrator: '<S19>/vgw_p2'
     */
    dryden_code_gen_B.w1[0] = rtb_sigma_ugsigma_vg *
      dryden_code_gen_X.vgw_p2_CSTATE[0];
    dryden_code_gen_B.w1[1] = rtb_MediumHighAltitudeIntensity *
      dryden_code_gen_X.vgw_p2_CSTATE[1];
  }

  /* End of Outputs for SubSystem: '<S5>/Hvgw(s)' */

  /* Gain: '<S11>/Lw' */
  frac[0] = rtb_Product_f;

  /* Outputs for Enabled SubSystem: '<S5>/Hwgw(s)' incorporates:
   *  EnablePort: '<S20>/Enable'
   */
  if (rtmIsMajorTimeStep(dryden_code_gen_M) && rtsiIsModeUpdateTimeStep
      (&dryden_code_gen_M->solverInfo) && (!dryden_code_gen_DW.Hwgws_MODE)) {
    /* InitializeConditions for Integrator: '<S20>/wg_p1' */
    dryden_code_gen_X.wg_p1_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S20>/wg_p2' */
    dryden_code_gen_X.wg_p2_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S20>/wg_p1' */
    dryden_code_gen_X.wg_p1_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S20>/wg_p2' */
    dryden_code_gen_X.wg_p2_CSTATE[1] = 0.0;
    dryden_code_gen_DW.Hwgws_MODE = true;
  }

  if (dryden_code_gen_DW.Hwgws_MODE) {
    /* Product: '<S20>/Lwg//V' incorporates:
     *  Gain: '<S11>/Lw'
     */
    dryden_code_gen_B.w_c[0] = rtb_Product_f / rtb_UnitConversion_g;

    /* Product: '<S20>/w' incorporates:
     *  Gain: '<S20>/1//pi'
     *  Integrator: '<S20>/wg_p1'
     *  Product: '<S20>/Lug//V1'
     *  Sqrt: '<S20>/sqrt1'
     *  Sum: '<S20>/Sum'
     */
    dryden_code_gen_B.w[0] = (sqrt(0.31830988618379069 * dryden_code_gen_B.w_c[0])
      * dryden_code_gen_B.Product[2] - dryden_code_gen_X.wg_p1_CSTATE[0]) /
      dryden_code_gen_B.w_c[0];

    /* Product: '<S20>/w ' incorporates:
     *  Integrator: '<S20>/wg_p1'
     *  Integrator: '<S20>/wg_p2'
     *  Product: '<S20>/Lwg//V'
     *  Product: '<S20>/Lwg//V '
     *  Sum: '<S20>/Sum1'
     */
    dryden_code_gen_B.w_c[0] = (dryden_code_gen_B.w[0] *
      dryden_code_gen_ConstB.sqrt_o * dryden_code_gen_B.w_c[0] +
      (dryden_code_gen_X.wg_p1_CSTATE[0] - dryden_code_gen_X.wg_p2_CSTATE[0])) /
      dryden_code_gen_B.w_c[0];

    /* Product: '<S20>/Lwg//V' incorporates:
     *  Gain: '<S11>/Lw'
     */
    dryden_code_gen_B.w_c[1] = dryden_code_gen_ConstB.UnitConversion_h /
      rtb_UnitConversion_g;

    /* Product: '<S20>/w' incorporates:
     *  Gain: '<S20>/1//pi'
     *  Integrator: '<S20>/wg_p1'
     *  Product: '<S20>/Lug//V1'
     *  Sqrt: '<S20>/sqrt1'
     *  Sum: '<S20>/Sum'
     */
    dryden_code_gen_B.w[1] = (sqrt(0.31830988618379069 * dryden_code_gen_B.w_c[1])
      * dryden_code_gen_B.Product[2] - dryden_code_gen_X.wg_p1_CSTATE[1]) /
      dryden_code_gen_B.w_c[1];

    /* Product: '<S20>/w ' incorporates:
     *  Integrator: '<S20>/wg_p1'
     *  Integrator: '<S20>/wg_p2'
     *  Product: '<S20>/Lwg//V'
     *  Product: '<S20>/Lwg//V '
     *  Sum: '<S20>/Sum1'
     */
    dryden_code_gen_B.w_c[1] = (dryden_code_gen_B.w[1] *
      dryden_code_gen_ConstB.sqrt_o * dryden_code_gen_B.w_c[1] +
      (dryden_code_gen_X.wg_p1_CSTATE[1] - dryden_code_gen_X.wg_p2_CSTATE[1])) /
      dryden_code_gen_B.w_c[1];

    /* Product: '<S20>/Lwg//V 1' incorporates:
     *  Integrator: '<S20>/wg_p2'
     */
    dryden_code_gen_B.LwgV1[0] = dryden_code_gen_ConstB.sigma_wg *
      dryden_code_gen_X.wg_p2_CSTATE[0];
    dryden_code_gen_B.LwgV1[1] = rtb_MediumHighAltitudeIntensity *
      dryden_code_gen_X.wg_p2_CSTATE[1];
  }

  /* End of Outputs for SubSystem: '<S5>/Hwgw(s)' */

  /* Sqrt: '<S53>/sqrt' incorporates:
   *  Inport: '<Root>/w'
   *  Inport: '<Root>/x'
   *  Inport: '<Root>/y'
   *  Inport: '<Root>/z'
   *  Product: '<S54>/Product'
   *  Product: '<S54>/Product1'
   *  Product: '<S54>/Product2'
   *  Product: '<S54>/Product3'
   *  Sum: '<S54>/Sum'
   */
  rtb_Product3_p = sqrt(((dryden_code_gen_U.w * dryden_code_gen_U.w +
    dryden_code_gen_U.x * dryden_code_gen_U.x) + dryden_code_gen_U.y *
    dryden_code_gen_U.y) + dryden_code_gen_U.z * dryden_code_gen_U.z);

  /* Product: '<S52>/Product' incorporates:
   *  Inport: '<Root>/w'
   */
  rtb_Product_f = dryden_code_gen_U.w / rtb_Product3_p;

  /* Product: '<S52>/Product1' incorporates:
   *  Inport: '<Root>/x'
   */
  rtb_LowAltitudeScaleLength = dryden_code_gen_U.x / rtb_Product3_p;

  /* Product: '<S52>/Product2' incorporates:
   *  Inport: '<Root>/y'
   */
  rtb_sigma_ugsigma_vg = dryden_code_gen_U.y / rtb_Product3_p;

  /* Product: '<S52>/Product3' incorporates:
   *  Inport: '<Root>/z'
   */
  rtb_Product3_p = dryden_code_gen_U.z / rtb_Product3_p;

  /* Product: '<S42>/Product3' incorporates:
   *  Product: '<S46>/Product3'
   */
  rtb_VectorConcatenate_tmp_1 = rtb_Product_f * rtb_Product_f;

  /* Product: '<S42>/Product2' incorporates:
   *  Product: '<S46>/Product2'
   */
  rtb_VectorConcatenate_tmp_2 = rtb_LowAltitudeScaleLength *
    rtb_LowAltitudeScaleLength;

  /* Product: '<S42>/Product1' incorporates:
   *  Product: '<S46>/Product1'
   *  Product: '<S50>/Product1'
   */
  rtb_VectorConcatenate_tmp_3 = rtb_sigma_ugsigma_vg * rtb_sigma_ugsigma_vg;

  /* Product: '<S42>/Product' incorporates:
   *  Product: '<S46>/Product'
   *  Product: '<S50>/Product'
   */
  rtb_VectorConcatenate_tmp_4 = rtb_Product3_p * rtb_Product3_p;

  /* Sum: '<S42>/Sum' incorporates:
   *  Product: '<S42>/Product'
   *  Product: '<S42>/Product1'
   *  Product: '<S42>/Product2'
   *  Product: '<S42>/Product3'
   */
  rtb_VectorConcatenate[0] = ((rtb_VectorConcatenate_tmp_1 +
    rtb_VectorConcatenate_tmp_2) - rtb_VectorConcatenate_tmp_3) -
    rtb_VectorConcatenate_tmp_4;

  /* Product: '<S45>/Product3' incorporates:
   *  Product: '<S43>/Product3'
   */
  rtb_VectorConcatenate_tmp = rtb_Product3_p * rtb_Product_f;

  /* Product: '<S45>/Product2' incorporates:
   *  Product: '<S43>/Product2'
   */
  rtb_VectorConcatenate_tmp_0 = rtb_LowAltitudeScaleLength *
    rtb_sigma_ugsigma_vg;

  /* Gain: '<S45>/Gain' incorporates:
   *  Product: '<S45>/Product2'
   *  Product: '<S45>/Product3'
   *  Sum: '<S45>/Sum'
   */
  rtb_VectorConcatenate[1] = (rtb_VectorConcatenate_tmp_0 -
    rtb_VectorConcatenate_tmp) * 2.0;

  /* Product: '<S48>/Product2' incorporates:
   *  Product: '<S44>/Product2'
   */
  rtb_VectorConcatenate_tmp_5 = rtb_LowAltitudeScaleLength * rtb_Product3_p;

  /* Product: '<S48>/Product1' incorporates:
   *  Product: '<S44>/Product1'
   */
  rtb_VectorConcatenate_tmp_6 = rtb_Product_f * rtb_sigma_ugsigma_vg;

  /* Gain: '<S48>/Gain' incorporates:
   *  Product: '<S48>/Product1'
   *  Product: '<S48>/Product2'
   *  Sum: '<S48>/Sum'
   */
  rtb_VectorConcatenate[2] = (rtb_VectorConcatenate_tmp_6 +
    rtb_VectorConcatenate_tmp_5) * 2.0;

  /* Gain: '<S43>/Gain' incorporates:
   *  Sum: '<S43>/Sum'
   */
  rtb_VectorConcatenate[3] = (rtb_VectorConcatenate_tmp +
    rtb_VectorConcatenate_tmp_0) * 2.0;

  /* Sum: '<S46>/Sum' incorporates:
   *  Sum: '<S50>/Sum'
   */
  rtb_VectorConcatenate_tmp_1 -= rtb_VectorConcatenate_tmp_2;
  rtb_VectorConcatenate[4] = (rtb_VectorConcatenate_tmp_1 +
    rtb_VectorConcatenate_tmp_3) - rtb_VectorConcatenate_tmp_4;

  /* Product: '<S49>/Product1' incorporates:
   *  Product: '<S47>/Product1'
   */
  rtb_VectorConcatenate_tmp_2 = rtb_Product_f * rtb_LowAltitudeScaleLength;

  /* Product: '<S49>/Product2' incorporates:
   *  Product: '<S47>/Product2'
   */
  rtb_VectorConcatenate_tmp = rtb_sigma_ugsigma_vg * rtb_Product3_p;

  /* Gain: '<S49>/Gain' incorporates:
   *  Product: '<S49>/Product1'
   *  Product: '<S49>/Product2'
   *  Sum: '<S49>/Sum'
   */
  rtb_VectorConcatenate[5] = (rtb_VectorConcatenate_tmp -
    rtb_VectorConcatenate_tmp_2) * 2.0;

  /* Gain: '<S44>/Gain' incorporates:
   *  Sum: '<S44>/Sum'
   */
  rtb_VectorConcatenate[6] = (rtb_VectorConcatenate_tmp_5 -
    rtb_VectorConcatenate_tmp_6) * 2.0;

  /* Gain: '<S47>/Gain' incorporates:
   *  Sum: '<S47>/Sum'
   */
  rtb_VectorConcatenate[7] = (rtb_VectorConcatenate_tmp_2 +
    rtb_VectorConcatenate_tmp) * 2.0;

  /* Sum: '<S50>/Sum' */
  rtb_VectorConcatenate[8] = (rtb_VectorConcatenate_tmp_1 -
    rtb_VectorConcatenate_tmp_3) + rtb_VectorConcatenate_tmp_4;

  /* If: '<S10>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  if (rtsiIsModeUpdateTimeStep(&dryden_code_gen_M->solverInfo)) {
    if (rtb_UnitConversion <= 1000.0) {
      dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifHei = 0;
    } else if (rtb_UnitConversion >= 2000.0) {
      dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifHei = 1;
    } else {
      dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifHei = 2;
    }
  }

  switch (dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifHei) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S10>/Low altitude  velocities' incorporates:
     *  ActionPort: '<S32>/Action Port'
     */
    /* SignalConversion generated from: '<S37>/Vector Concatenate' */
    rtb_VectorConcatenate_c[2] = dryden_code_gen_B.LwgV1[0];

    /* Trigonometry: '<S38>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S3>/Unit Conversion'
     */
    rtb_Product_f = sin(dryden_code_gen_ConstB.UnitConversion);
    rtb_Product3_p = cos(dryden_code_gen_ConstB.UnitConversion);

    /* Sum: '<S38>/Sum' incorporates:
     *  Product: '<S38>/Product1'
     *  Product: '<S38>/Product2'
     */
    rtb_VectorConcatenate_c[0] = dryden_code_gen_B.w1_p[0] * rtb_Product3_p -
      rtb_Product_f * dryden_code_gen_B.w1[0];

    /* Sum: '<S38>/Sum1' incorporates:
     *  Product: '<S38>/Product1'
     *  Product: '<S38>/Product2'
     */
    rtb_VectorConcatenate_c[1] = rtb_Product_f * dryden_code_gen_B.w1_p[0] +
      dryden_code_gen_B.w1[0] * rtb_Product3_p;

    /* End of Outputs for SubSystem: '<S10>/Low altitude  velocities' */
    for (i = 0; i <= 0; i += 2) {
      /* Outputs for IfAction SubSystem: '<S10>/Low altitude  velocities' incorporates:
       *  ActionPort: '<S32>/Action Port'
       */
      _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_set1_pd(0.0));
      tmp_0 = _mm_loadu_pd(&rtb_VectorConcatenate[i]);
      tmp_1 = _mm_loadu_pd(&dryden_code_gen_Y.pqr_ptr[i]);
      _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_add_pd(_mm_mul_pd(tmp_0,
        _mm_set1_pd(rtb_VectorConcatenate_c[0])), tmp_1));
      tmp_0 = _mm_loadu_pd(&rtb_VectorConcatenate[i + 3]);
      tmp_1 = _mm_loadu_pd(&dryden_code_gen_Y.pqr_ptr[i]);
      _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_add_pd(_mm_mul_pd(tmp_0,
        _mm_set1_pd(rtb_VectorConcatenate_c[1])), tmp_1));
      tmp_0 = _mm_loadu_pd(&rtb_VectorConcatenate[i + 6]);
      tmp_1 = _mm_loadu_pd(&dryden_code_gen_Y.pqr_ptr[i]);
      _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_add_pd(_mm_mul_pd(tmp_0,
        _mm_set1_pd(rtb_VectorConcatenate_c[2])), tmp_1));

      /* End of Outputs for SubSystem: '<S10>/Low altitude  velocities' */
    }

    /* Outputs for IfAction SubSystem: '<S10>/Low altitude  velocities' incorporates:
     *  ActionPort: '<S32>/Action Port'
     */
    /* Reshape: '<S37>/Reshape1' incorporates:
     *  Concatenate: '<S37>/Vector Concatenate'
     *  Concatenate: '<S51>/Vector Concatenate'
     *  Product: '<S37>/Product'
     */
    for (i = 2; i < 3; i++) {
      dryden_code_gen_Y.pqr_ptr[i] = 0.0;
      dryden_code_gen_Y.pqr_ptr[i] += rtb_VectorConcatenate[i] *
        rtb_VectorConcatenate_c[0];
      dryden_code_gen_Y.pqr_ptr[i] += rtb_VectorConcatenate[i + 3] *
        rtb_VectorConcatenate_c[1];
      dryden_code_gen_Y.pqr_ptr[i] += rtb_VectorConcatenate[i + 6] *
        rtb_VectorConcatenate_c[2];
    }

    /* End of Reshape: '<S37>/Reshape1' */
    /* End of Outputs for SubSystem: '<S10>/Low altitude  velocities' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S10>/Medium//High  altitude velocities' incorporates:
     *  ActionPort: '<S33>/Action Port'
     */
    /* Gain: '<S33>/Gain' */
    dryden_code_gen_Y.pqr_ptr[0] = dryden_code_gen_B.w1_p[1];
    dryden_code_gen_Y.pqr_ptr[1] = dryden_code_gen_B.w1[1];
    dryden_code_gen_Y.pqr_ptr[2] = dryden_code_gen_B.LwgV1[1];

    /* End of Outputs for SubSystem: '<S10>/Medium//High  altitude velocities' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S10>/Interpolate  velocities' incorporates:
     *  ActionPort: '<S31>/Action Port'
     */
    /* Trigonometry: '<S36>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S3>/Unit Conversion'
     */
    rtb_Product3_p = sin(dryden_code_gen_ConstB.UnitConversion);
    rtb_LowAltitudeScaleLength = cos(dryden_code_gen_ConstB.UnitConversion);

    /* Sum: '<S36>/Sum' incorporates:
     *  Product: '<S36>/Product1'
     *  Product: '<S36>/Product2'
     */
    rtb_Product_f = dryden_code_gen_B.w1_p[0] * rtb_LowAltitudeScaleLength -
      rtb_Product3_p * dryden_code_gen_B.w1[0];

    /* Sum: '<S36>/Sum1' incorporates:
     *  Product: '<S36>/Product1'
     *  Product: '<S36>/Product2'
     */
    rtb_Product3_p = rtb_Product3_p * dryden_code_gen_B.w1_p[0] +
      dryden_code_gen_B.w1[0] * rtb_LowAltitudeScaleLength;

    /* SignalConversion generated from: '<S35>/Vector Concatenate' */
    rtb_LowAltitudeScaleLength = dryden_code_gen_B.LwgV1[0];

    /* End of Outputs for SubSystem: '<S10>/Interpolate  velocities' */
    for (i = 0; i <= 0; i += 2) {
      /* Outputs for IfAction SubSystem: '<S10>/Interpolate  velocities' incorporates:
       *  ActionPort: '<S31>/Action Port'
       */
      tmp_0 = _mm_loadu_pd(&rtb_VectorConcatenate[i]);
      tmp_1 = _mm_loadu_pd(&rtb_VectorConcatenate[i + 3]);
      tmp = _mm_loadu_pd(&rtb_VectorConcatenate[i + 6]);
      _mm_storeu_pd(&rtb_VectorConcatenate_c[i], _mm_add_pd(_mm_mul_pd(tmp,
        _mm_set1_pd(rtb_LowAltitudeScaleLength)), _mm_add_pd(_mm_mul_pd(tmp_1,
        _mm_set1_pd(rtb_Product3_p)), _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd
        (rtb_Product_f)), _mm_set1_pd(0.0)))));

      /* End of Outputs for SubSystem: '<S10>/Interpolate  velocities' */
    }

    /* Outputs for IfAction SubSystem: '<S10>/Interpolate  velocities' incorporates:
     *  ActionPort: '<S31>/Action Port'
     */
    /* Product: '<S35>/Product' incorporates:
     *  Concatenate: '<S35>/Vector Concatenate'
     *  Concatenate: '<S51>/Vector Concatenate'
     */
    for (i = 2; i < 3; i++) {
      rtb_VectorConcatenate_c[i] = (rtb_VectorConcatenate[i + 3] *
        rtb_Product3_p + rtb_VectorConcatenate[i] * rtb_Product_f) +
        rtb_VectorConcatenate[i + 6] * rtb_LowAltitudeScaleLength;
    }

    /* End of Product: '<S35>/Product' */

    /* Sum: '<S31>/Sum3' incorporates:
     *  Constant: '<S31>/max_height_low'
     *  Product: '<S31>/Product1'
     *  Sum: '<S31>/Sum1'
     *  Sum: '<S31>/Sum2'
     */
    dryden_code_gen_Y.pqr_ptr[0] = (dryden_code_gen_B.w1_p[1] -
      rtb_VectorConcatenate_c[0]) * (rtb_UnitConversion - 1000.0) /
      dryden_code_gen_ConstB.Sum + rtb_VectorConcatenate_c[0];
    dryden_code_gen_Y.pqr_ptr[1] = (dryden_code_gen_B.w1[1] -
      rtb_VectorConcatenate_c[1]) * (rtb_UnitConversion - 1000.0) /
      dryden_code_gen_ConstB.Sum + rtb_VectorConcatenate_c[1];
    dryden_code_gen_Y.pqr_ptr[2] = (dryden_code_gen_B.LwgV1[1] -
      rtb_VectorConcatenate_c[2]) * (rtb_UnitConversion - 1000.0) /
      dryden_code_gen_ConstB.Sum + rtb_VectorConcatenate_c[2];

    /* End of Outputs for SubSystem: '<S10>/Interpolate  velocities' */
    break;
  }

  /* End of If: '<S10>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */

  /* UnitConversion: '<S1>/Unit Conversion' */
  /* Unit Conversion - from: ft/s to: m/s
     Expression: output = (0.3048*input) + (0) */
  dryden_code_gen_Y.pqr_ptr[0] *= 0.3048;

  /* Outport: '<Root>/uvw_ptr' */
  dryden_code_gen_Y.uvw_ptr[0] = dryden_code_gen_Y.pqr_ptr[0];

  /* UnitConversion: '<S1>/Unit Conversion' */
  dryden_code_gen_Y.pqr_ptr[1] *= 0.3048;

  /* Outport: '<Root>/uvw_ptr' */
  dryden_code_gen_Y.uvw_ptr[1] = dryden_code_gen_Y.pqr_ptr[1];

  /* UnitConversion: '<S1>/Unit Conversion' */
  dryden_code_gen_Y.pqr_ptr[2] *= 0.3048;

  /* Outport: '<Root>/uvw_ptr' */
  dryden_code_gen_Y.uvw_ptr[2] = dryden_code_gen_Y.pqr_ptr[2];

  /* Outputs for Enabled SubSystem: '<S4>/Hqgw' incorporates:
   *  EnablePort: '<S16>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S4>/Hpgw' incorporates:
   *  EnablePort: '<S15>/Enable'
   */
  if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
    if (rtsiIsModeUpdateTimeStep(&dryden_code_gen_M->solverInfo) &&
        (!dryden_code_gen_DW.Hpgw_MODE)) {
      /* InitializeConditions for Integrator: '<S15>/pgw_p' */
      dryden_code_gen_X.pgw_p_CSTATE[0] = 0.0;
      dryden_code_gen_X.pgw_p_CSTATE[1] = 0.0;
      dryden_code_gen_DW.Hpgw_MODE = true;
    }

    if (rtsiIsModeUpdateTimeStep(&dryden_code_gen_M->solverInfo) &&
        (!dryden_code_gen_DW.Hqgw_MODE)) {
      /* InitializeConditions for Integrator: '<S16>/qgw_p' */
      dryden_code_gen_X.qgw_p_CSTATE[0] = 0.0;
      dryden_code_gen_X.qgw_p_CSTATE[1] = 0.0;
      dryden_code_gen_DW.Hqgw_MODE = true;
    }
  }

  /* End of Outputs for SubSystem: '<S4>/Hqgw' */
  if (dryden_code_gen_DW.Hpgw_MODE) {
    /* Fcn: '<S15>/sqrt(0.8//V)' */
    rtb_Product_f = 0.8 / rtb_UnitConversion_g;
    if (rtb_Product_f < 0.0) {
      rtb_Product_f = -sqrt(-rtb_Product_f);
    } else {
      rtb_Product_f = sqrt(rtb_Product_f);
    }

    /* Product: '<S15>/w3' */
    rtb_Product3_p = rtb_UnitConversion_g * dryden_code_gen_ConstB.w4;

    /* Product: '<S15>/w' incorporates:
     *  Fcn: '<S15>/sqrt(0.8//V)'
     *  Integrator: '<S15>/pgw_p'
     *  Math: '<S15>/L^1//3'
     *  Product: '<S15>/Lug//V1'
     *  Product: '<S15>/w1'
     *  Product: '<S15>/w2'
     *  Sum: '<S15>/Sum'
     */
    dryden_code_gen_B.w_g[0] = (rtb_Product_f / rt_powd_snf(frac[0],
      0.33333333333333331) * dryden_code_gen_ConstB.u16 *
      dryden_code_gen_B.Product[3] - dryden_code_gen_X.pgw_p_CSTATE[0]) *
      rtb_Product3_p;

    /* Math: '<S15>/L^1//3' incorporates:
     *  Gain: '<S11>/Lw'
     */
    if (dryden_code_gen_ConstB.UnitConversion_h < 0.0) {
      rtb_LowAltitudeScaleLength = -rt_powd_snf
        (-dryden_code_gen_ConstB.UnitConversion_h, 0.33333333333333331);
    } else {
      rtb_LowAltitudeScaleLength = rt_powd_snf
        (dryden_code_gen_ConstB.UnitConversion_h, 0.33333333333333331);
    }

    /* Product: '<S15>/w' incorporates:
     *  Fcn: '<S15>/sqrt(0.8//V)'
     *  Integrator: '<S15>/pgw_p'
     *  Product: '<S15>/Lug//V1'
     *  Product: '<S15>/w1'
     *  Product: '<S15>/w2'
     *  Sum: '<S15>/Sum'
     */
    dryden_code_gen_B.w_g[1] = (rtb_Product_f / rtb_LowAltitudeScaleLength *
      dryden_code_gen_ConstB.u16 * dryden_code_gen_B.Product[3] -
      dryden_code_gen_X.pgw_p_CSTATE[1]) * rtb_Product3_p;

    /* Product: '<S15>/sigma_w' incorporates:
     *  Integrator: '<S15>/pgw_p'
     */
    dryden_code_gen_B.sigma_w[0] = dryden_code_gen_ConstB.sigma_wg *
      dryden_code_gen_X.pgw_p_CSTATE[0];
    dryden_code_gen_B.sigma_w[1] = rtb_MediumHighAltitudeIntensity *
      dryden_code_gen_X.pgw_p_CSTATE[1];
  }

  /* End of Outputs for SubSystem: '<S4>/Hpgw' */

  /* Outputs for Enabled SubSystem: '<S4>/Hqgw' incorporates:
   *  EnablePort: '<S16>/Enable'
   */
  if (dryden_code_gen_DW.Hqgw_MODE) {
    /* Product: '<S16>/w' incorporates:
     *  Gain: '<S16>/pi//4'
     */
    rtb_MediumHighAltitudeIntensity = 0.78539816339744828 * rtb_UnitConversion_g
      / dryden_code_gen_ConstB.UnitConversion_a;

    /* Product: '<S16>/w' incorporates:
     *  Integrator: '<S16>/qgw_p'
     *  Product: '<S16>/wg//V'
     *  Sum: '<S16>/Sum'
     */
    dryden_code_gen_B.w_p[0] = (dryden_code_gen_B.LwgV1[0] /
      rtb_UnitConversion_g - dryden_code_gen_X.qgw_p_CSTATE[0]) *
      rtb_MediumHighAltitudeIntensity;
    dryden_code_gen_B.w_p[1] = (dryden_code_gen_B.LwgV1[1] /
      rtb_UnitConversion_g - dryden_code_gen_X.qgw_p_CSTATE[1]) *
      rtb_MediumHighAltitudeIntensity;
  }

  /* End of Outputs for SubSystem: '<S4>/Hqgw' */

  /* Outputs for Enabled SubSystem: '<S4>/Hrgw' incorporates:
   *  EnablePort: '<S17>/Enable'
   */
  if (rtmIsMajorTimeStep(dryden_code_gen_M) && rtsiIsModeUpdateTimeStep
      (&dryden_code_gen_M->solverInfo) && (!dryden_code_gen_DW.Hrgw_MODE)) {
    /* InitializeConditions for Integrator: '<S17>/rgw_p' */
    dryden_code_gen_X.rgw_p_CSTATE[0] = 0.0;
    dryden_code_gen_X.rgw_p_CSTATE[1] = 0.0;
    dryden_code_gen_DW.Hrgw_MODE = true;
  }

  if (dryden_code_gen_DW.Hrgw_MODE) {
    /* Product: '<S17>/w' incorporates:
     *  Gain: '<S17>/pi//3'
     */
    rtb_MediumHighAltitudeIntensity = 1.0471975511965976 * rtb_UnitConversion_g /
      dryden_code_gen_ConstB.UnitConversion_a;

    /* Product: '<S17>/w' incorporates:
     *  Integrator: '<S17>/rgw_p'
     *  Product: '<S17>/vg//V'
     *  Sum: '<S17>/Sum'
     */
    dryden_code_gen_B.w_ic[0] = (dryden_code_gen_B.w1[0] / rtb_UnitConversion_g
      - dryden_code_gen_X.rgw_p_CSTATE[0]) * rtb_MediumHighAltitudeIntensity;
    dryden_code_gen_B.w_ic[1] = (dryden_code_gen_B.w1[1] / rtb_UnitConversion_g
      - dryden_code_gen_X.rgw_p_CSTATE[1]) * rtb_MediumHighAltitudeIntensity;
  }

  /* End of Outputs for SubSystem: '<S4>/Hrgw' */

  /* If: '<S9>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  if (rtsiIsModeUpdateTimeStep(&dryden_code_gen_M->solverInfo)) {
    if (rtb_UnitConversion <= 1000.0) {
      dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifH_i = 0;
    } else if (rtb_UnitConversion >= 2000.0) {
      dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifH_i = 1;
    } else {
      dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifH_i = 2;
    }
  }

  switch (dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifH_i) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S9>/Low altitude  rates' incorporates:
     *  ActionPort: '<S24>/Action Port'
     */
    /* SignalConversion generated from: '<S29>/Vector Concatenate' */
    rtb_VectorConcatenate_c[2] = dryden_code_gen_B.w_ic[0];

    /* Trigonometry: '<S30>/Trigonometric Function1' incorporates:
     *  UnitConversion: '<S3>/Unit Conversion'
     */
    rtb_UnitConversion = sin(dryden_code_gen_ConstB.UnitConversion);
    rtb_UnitConversion_g = cos(dryden_code_gen_ConstB.UnitConversion);

    /* Sum: '<S30>/Sum' incorporates:
     *  Product: '<S30>/Product1'
     *  Product: '<S30>/Product2'
     */
    rtb_VectorConcatenate_c[0] = dryden_code_gen_B.sigma_w[0] *
      rtb_UnitConversion_g - rtb_UnitConversion * dryden_code_gen_B.w_p[0];

    /* Sum: '<S30>/Sum1' incorporates:
     *  Product: '<S30>/Product1'
     *  Product: '<S30>/Product2'
     */
    rtb_VectorConcatenate_c[1] = rtb_UnitConversion * dryden_code_gen_B.sigma_w
      [0] + dryden_code_gen_B.w_p[0] * rtb_UnitConversion_g;

    /* End of Outputs for SubSystem: '<S9>/Low altitude  rates' */
    for (i = 0; i <= 0; i += 2) {
      /* Outputs for IfAction SubSystem: '<S9>/Low altitude  rates' incorporates:
       *  ActionPort: '<S24>/Action Port'
       */
      _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_set1_pd(0.0));
      tmp_0 = _mm_loadu_pd(&rtb_VectorConcatenate[i]);
      tmp_1 = _mm_loadu_pd(&dryden_code_gen_Y.pqr_ptr[i]);
      _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_add_pd(_mm_mul_pd(tmp_0,
        _mm_set1_pd(rtb_VectorConcatenate_c[0])), tmp_1));
      tmp_0 = _mm_loadu_pd(&rtb_VectorConcatenate[i + 3]);
      tmp_1 = _mm_loadu_pd(&dryden_code_gen_Y.pqr_ptr[i]);
      _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_add_pd(_mm_mul_pd(tmp_0,
        _mm_set1_pd(rtb_VectorConcatenate_c[1])), tmp_1));
      tmp_0 = _mm_loadu_pd(&rtb_VectorConcatenate[i + 6]);
      tmp_1 = _mm_loadu_pd(&dryden_code_gen_Y.pqr_ptr[i]);
      _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_add_pd(_mm_mul_pd(tmp_0,
        _mm_set1_pd(rtb_VectorConcatenate_c[2])), tmp_1));

      /* End of Outputs for SubSystem: '<S9>/Low altitude  rates' */
    }

    /* Outputs for IfAction SubSystem: '<S9>/Low altitude  rates' incorporates:
     *  ActionPort: '<S24>/Action Port'
     */
    /* Reshape: '<S29>/Reshape1' incorporates:
     *  Concatenate: '<S29>/Vector Concatenate'
     *  Concatenate: '<S51>/Vector Concatenate'
     *  Product: '<S29>/Product'
     */
    for (i = 2; i < 3; i++) {
      dryden_code_gen_Y.pqr_ptr[i] = 0.0;
      dryden_code_gen_Y.pqr_ptr[i] += rtb_VectorConcatenate[i] *
        rtb_VectorConcatenate_c[0];
      dryden_code_gen_Y.pqr_ptr[i] += rtb_VectorConcatenate[i + 3] *
        rtb_VectorConcatenate_c[1];
      dryden_code_gen_Y.pqr_ptr[i] += rtb_VectorConcatenate[i + 6] *
        rtb_VectorConcatenate_c[2];
    }

    /* End of Reshape: '<S29>/Reshape1' */
    /* End of Outputs for SubSystem: '<S9>/Low altitude  rates' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S9>/Medium//High  altitude rates' incorporates:
     *  ActionPort: '<S25>/Action Port'
     */
    /* Gain: '<S25>/Gain' */
    dryden_code_gen_Y.pqr_ptr[0] = dryden_code_gen_B.sigma_w[1];
    dryden_code_gen_Y.pqr_ptr[1] = dryden_code_gen_B.w_p[1];
    dryden_code_gen_Y.pqr_ptr[2] = dryden_code_gen_B.w_ic[1];

    /* End of Outputs for SubSystem: '<S9>/Medium//High  altitude rates' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S9>/Interpolate  rates' incorporates:
     *  ActionPort: '<S23>/Action Port'
     */
    /* Trigonometry: '<S28>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S3>/Unit Conversion'
     */
    rtb_UnitConversion_g = sin(dryden_code_gen_ConstB.UnitConversion);
    rtb_MediumHighAltitudeIntensity = cos(dryden_code_gen_ConstB.UnitConversion);

    /* Sum: '<S28>/Sum' incorporates:
     *  Product: '<S28>/Product1'
     *  Product: '<S28>/Product2'
     */
    rtb_Product_f = dryden_code_gen_B.sigma_w[0] *
      rtb_MediumHighAltitudeIntensity - rtb_UnitConversion_g *
      dryden_code_gen_B.w_p[0];

    /* Sum: '<S28>/Sum1' incorporates:
     *  Product: '<S28>/Product1'
     *  Product: '<S28>/Product2'
     */
    rtb_Product3_p = rtb_UnitConversion_g * dryden_code_gen_B.sigma_w[0] +
      dryden_code_gen_B.w_p[0] * rtb_MediumHighAltitudeIntensity;

    /* SignalConversion generated from: '<S27>/Vector Concatenate' */
    rtb_LowAltitudeScaleLength = dryden_code_gen_B.w_ic[0];

    /* End of Outputs for SubSystem: '<S9>/Interpolate  rates' */
    for (i = 0; i <= 0; i += 2) {
      /* Outputs for IfAction SubSystem: '<S9>/Interpolate  rates' incorporates:
       *  ActionPort: '<S23>/Action Port'
       */
      tmp_0 = _mm_loadu_pd(&rtb_VectorConcatenate[i]);
      tmp_1 = _mm_loadu_pd(&rtb_VectorConcatenate[i + 3]);
      tmp = _mm_loadu_pd(&rtb_VectorConcatenate[i + 6]);
      _mm_storeu_pd(&rtb_VectorConcatenate_c[i], _mm_add_pd(_mm_mul_pd(tmp,
        _mm_set1_pd(rtb_LowAltitudeScaleLength)), _mm_add_pd(_mm_mul_pd(tmp_1,
        _mm_set1_pd(rtb_Product3_p)), _mm_add_pd(_mm_mul_pd(tmp_0, _mm_set1_pd
        (rtb_Product_f)), _mm_set1_pd(0.0)))));

      /* End of Outputs for SubSystem: '<S9>/Interpolate  rates' */
    }

    /* Outputs for IfAction SubSystem: '<S9>/Interpolate  rates' incorporates:
     *  ActionPort: '<S23>/Action Port'
     */
    /* Product: '<S27>/Product' incorporates:
     *  Concatenate: '<S27>/Vector Concatenate'
     *  Concatenate: '<S51>/Vector Concatenate'
     */
    for (i = 2; i < 3; i++) {
      rtb_VectorConcatenate_c[i] = (rtb_VectorConcatenate[i + 3] *
        rtb_Product3_p + rtb_VectorConcatenate[i] * rtb_Product_f) +
        rtb_VectorConcatenate[i + 6] * rtb_LowAltitudeScaleLength;
    }

    /* End of Product: '<S27>/Product' */

    /* Sum: '<S23>/Sum3' incorporates:
     *  Constant: '<S23>/max_height_low'
     *  Product: '<S23>/Product1'
     *  Sum: '<S23>/Sum1'
     *  Sum: '<S23>/Sum2'
     */
    dryden_code_gen_Y.pqr_ptr[0] = (dryden_code_gen_B.sigma_w[1] -
      rtb_VectorConcatenate_c[0]) * (rtb_UnitConversion - 1000.0) /
      dryden_code_gen_ConstB.Sum_n + rtb_VectorConcatenate_c[0];
    dryden_code_gen_Y.pqr_ptr[1] = (dryden_code_gen_B.w_p[1] -
      rtb_VectorConcatenate_c[1]) * (rtb_UnitConversion - 1000.0) /
      dryden_code_gen_ConstB.Sum_n + rtb_VectorConcatenate_c[1];
    dryden_code_gen_Y.pqr_ptr[2] = (dryden_code_gen_B.w_ic[1] -
      rtb_VectorConcatenate_c[2]) * (rtb_UnitConversion - 1000.0) /
      dryden_code_gen_ConstB.Sum_n + rtb_VectorConcatenate_c[2];

    /* End of Outputs for SubSystem: '<S9>/Interpolate  rates' */
    break;
  }

  /* End of If: '<S9>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
    if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
      /* Update for RandomNumber: '<S14>/White Noise' */
      dryden_code_gen_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
        (&dryden_code_gen_DW.RandSeed[0]);
      dryden_code_gen_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
        (&dryden_code_gen_DW.RandSeed[1]);
      dryden_code_gen_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
        (&dryden_code_gen_DW.RandSeed[2]);
      dryden_code_gen_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf
        (&dryden_code_gen_DW.RandSeed[3]);
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
    rt_ertODEUpdateContinuousStates(&dryden_code_gen_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++dryden_code_gen_M->Timing.clockTick0;
    dryden_code_gen_M->Timing.t[0] = rtsiGetSolverStopTime
      (&dryden_code_gen_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.1s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.1, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      dryden_code_gen_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void dryden_code_gen_derivatives(void)
{
  XDot_dryden_code_gen_T *_rtXdot;
  _rtXdot = ((XDot_dryden_code_gen_T *) dryden_code_gen_M->derivs);

  /* Derivatives for Enabled SubSystem: '<S5>/Hugw(s)' */
  if (dryden_code_gen_DW.Hugws_MODE) {
    /* Derivatives for Integrator: '<S18>/ug_p' */
    _rtXdot->ug_p_CSTATE[0] = dryden_code_gen_B.w_f[0];
    _rtXdot->ug_p_CSTATE[1] = dryden_code_gen_B.w_f[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_dryden_code_gen_T *) dryden_code_gen_M->derivs)->
             ug_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S5>/Hugw(s)' */

  /* Derivatives for Enabled SubSystem: '<S5>/Hvgw(s)' */
  if (dryden_code_gen_DW.Hvgws_MODE) {
    /* Derivatives for Integrator: '<S19>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[0] = dryden_code_gen_B.w_e[0];

    /* Derivatives for Integrator: '<S19>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[0] = dryden_code_gen_B.w_i[0];

    /* Derivatives for Integrator: '<S19>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[1] = dryden_code_gen_B.w_e[1];

    /* Derivatives for Integrator: '<S19>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[1] = dryden_code_gen_B.w_i[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_dryden_code_gen_T *) dryden_code_gen_M->derivs)
             ->vg_p1_CSTATE[0]);
      for (i=0; i < 4; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S5>/Hvgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S5>/Hwgw(s)' */
  if (dryden_code_gen_DW.Hwgws_MODE) {
    /* Derivatives for Integrator: '<S20>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[0] = dryden_code_gen_B.w[0];

    /* Derivatives for Integrator: '<S20>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[0] = dryden_code_gen_B.w_c[0];

    /* Derivatives for Integrator: '<S20>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[1] = dryden_code_gen_B.w[1];

    /* Derivatives for Integrator: '<S20>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[1] = dryden_code_gen_B.w_c[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_dryden_code_gen_T *) dryden_code_gen_M->derivs)
             ->wg_p1_CSTATE[0]);
      for (i=0; i < 4; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S5>/Hwgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S4>/Hpgw' */
  if (dryden_code_gen_DW.Hpgw_MODE) {
    /* Derivatives for Integrator: '<S15>/pgw_p' */
    _rtXdot->pgw_p_CSTATE[0] = dryden_code_gen_B.w_g[0];
    _rtXdot->pgw_p_CSTATE[1] = dryden_code_gen_B.w_g[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_dryden_code_gen_T *) dryden_code_gen_M->derivs)
             ->pgw_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S4>/Hpgw' */

  /* Derivatives for Enabled SubSystem: '<S4>/Hqgw' */
  if (dryden_code_gen_DW.Hqgw_MODE) {
    /* Derivatives for Integrator: '<S16>/qgw_p' */
    _rtXdot->qgw_p_CSTATE[0] = dryden_code_gen_B.w_p[0];
    _rtXdot->qgw_p_CSTATE[1] = dryden_code_gen_B.w_p[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_dryden_code_gen_T *) dryden_code_gen_M->derivs)
             ->qgw_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S4>/Hqgw' */

  /* Derivatives for Enabled SubSystem: '<S4>/Hrgw' */
  if (dryden_code_gen_DW.Hrgw_MODE) {
    /* Derivatives for Integrator: '<S17>/rgw_p' */
    _rtXdot->rgw_p_CSTATE[0] = dryden_code_gen_B.w_ic[0];
    _rtXdot->rgw_p_CSTATE[1] = dryden_code_gen_B.w_ic[1];
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_dryden_code_gen_T *) dryden_code_gen_M->derivs)
             ->rgw_p_CSTATE[0]);
      for (i=0; i < 2; i++) {
        dx[i] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S4>/Hrgw' */
}

/* Model initialize function */
void dryden_code_gen_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&dryden_code_gen_M->solverInfo,
                          &dryden_code_gen_M->Timing.simTimeStep);
    rtsiSetTPtr(&dryden_code_gen_M->solverInfo, &rtmGetTPtr(dryden_code_gen_M));
    rtsiSetStepSizePtr(&dryden_code_gen_M->solverInfo,
                       &dryden_code_gen_M->Timing.stepSize0);
    rtsiSetdXPtr(&dryden_code_gen_M->solverInfo, &dryden_code_gen_M->derivs);
    rtsiSetContStatesPtr(&dryden_code_gen_M->solverInfo, (real_T **)
                         &dryden_code_gen_M->contStates);
    rtsiSetNumContStatesPtr(&dryden_code_gen_M->solverInfo,
      &dryden_code_gen_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&dryden_code_gen_M->solverInfo,
      &dryden_code_gen_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&dryden_code_gen_M->solverInfo,
      &dryden_code_gen_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&dryden_code_gen_M->solverInfo,
      &dryden_code_gen_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&dryden_code_gen_M->solverInfo, (&rtmGetErrorStatus
      (dryden_code_gen_M)));
    rtsiSetRTModelPtr(&dryden_code_gen_M->solverInfo, dryden_code_gen_M);
  }

  rtsiSetSimTimeStep(&dryden_code_gen_M->solverInfo, MAJOR_TIME_STEP);
  dryden_code_gen_M->intgData.deltaY= dryden_code_gen_M->OdeDeltaY;
  dryden_code_gen_M->intgData.f[0] = dryden_code_gen_M->odeF[0];
  dryden_code_gen_M->intgData.f[1] = dryden_code_gen_M->odeF[1];
  dryden_code_gen_M->intgData.f[2] = dryden_code_gen_M->odeF[2];
  dryden_code_gen_M->intgData.f[3] = dryden_code_gen_M->odeF[3];
  dryden_code_gen_M->intgData.f[4] = dryden_code_gen_M->odeF[4];
  dryden_code_gen_M->intgData.f[5] = dryden_code_gen_M->odeF[5];
  dryden_code_gen_M->intgData.f[6] = dryden_code_gen_M->odeF[6];
  dryden_code_gen_M->intgData.f[7] = dryden_code_gen_M->odeF[7];
  dryden_code_gen_M->intgData.f[8] = dryden_code_gen_M->odeF[8];
  dryden_code_gen_M->intgData.f[9] = dryden_code_gen_M->odeF[9];
  dryden_code_gen_M->intgData.f[10] = dryden_code_gen_M->odeF[10];
  dryden_code_gen_M->intgData.f[11] = dryden_code_gen_M->odeF[11];
  dryden_code_gen_M->intgData.f[12] = dryden_code_gen_M->odeF[12];
  dryden_code_gen_M->intgData.x0 = dryden_code_gen_M->odeX0;
  dryden_code_gen_M->contStates = ((X_dryden_code_gen_T *) &dryden_code_gen_X);
  rtsiSetSolverData(&dryden_code_gen_M->solverInfo, (void *)
                    &dryden_code_gen_M->intgData);
  rtsiSetIsMinorTimeStepWithModeChange(&dryden_code_gen_M->solverInfo, false);
  rtsiSetSolverName(&dryden_code_gen_M->solverInfo,"ode8");
  rtmSetTPtr(dryden_code_gen_M, &dryden_code_gen_M->Timing.tArray[0]);
  dryden_code_gen_M->Timing.stepSize0 = 0.1;

  /* Start for If: '<S10>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifHei = -1;

  /* Start for If: '<S9>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifH_i = -1;

  /* InitializeConditions for RandomNumber: '<S14>/White Noise' */
  dryden_code_gen_DW.RandSeed[0] = 1529675776U;
  dryden_code_gen_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&dryden_code_gen_DW.RandSeed[0]);
  dryden_code_gen_DW.RandSeed[1] = 1529741312U;
  dryden_code_gen_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&dryden_code_gen_DW.RandSeed[1]);
  dryden_code_gen_DW.RandSeed[2] = 1529806848U;
  dryden_code_gen_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&dryden_code_gen_DW.RandSeed[2]);
  dryden_code_gen_DW.RandSeed[3] = 1529872384U;
  dryden_code_gen_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf
    (&dryden_code_gen_DW.RandSeed[3]);

  /* SystemInitialize for Enabled SubSystem: '<S5>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S18>/ug_p' */
  dryden_code_gen_X.ug_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S5>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S5>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S19>/vg_p1' */
  dryden_code_gen_X.vg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S19>/vgw_p2' */
  dryden_code_gen_X.vgw_p2_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S5>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S5>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S20>/wg_p1' */
  dryden_code_gen_X.wg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S20>/wg_p2' */
  dryden_code_gen_X.wg_p2_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S5>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S4>/Hpgw' */
  /* InitializeConditions for Integrator: '<S15>/pgw_p' */
  dryden_code_gen_X.pgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S4>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S4>/Hqgw' */
  /* InitializeConditions for Integrator: '<S16>/qgw_p' */
  dryden_code_gen_X.qgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S4>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S4>/Hrgw' */
  /* InitializeConditions for Integrator: '<S17>/rgw_p' */
  dryden_code_gen_X.rgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S4>/Hrgw' */

  /* SystemInitialize for Enabled SubSystem: '<S5>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S18>/ug_p' */
  dryden_code_gen_X.ug_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S5>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S5>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S19>/vg_p1' */
  dryden_code_gen_X.vg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S19>/vgw_p2' */
  dryden_code_gen_X.vgw_p2_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S5>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S5>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S20>/wg_p1' */
  dryden_code_gen_X.wg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S20>/wg_p2' */
  dryden_code_gen_X.wg_p2_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S5>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S4>/Hpgw' */
  /* InitializeConditions for Integrator: '<S15>/pgw_p' */
  dryden_code_gen_X.pgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S4>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S4>/Hqgw' */
  /* InitializeConditions for Integrator: '<S16>/qgw_p' */
  dryden_code_gen_X.qgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S4>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S4>/Hrgw' */
  /* InitializeConditions for Integrator: '<S17>/rgw_p' */
  dryden_code_gen_X.rgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S4>/Hrgw' */
}

/* Model terminate function */
void dryden_code_gen_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */