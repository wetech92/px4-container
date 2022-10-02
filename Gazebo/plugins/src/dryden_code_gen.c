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
 * C/C++ source code generated on : Sat Oct  1 10:39:28 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "dryden_code_gen.h"
#include "rtwtypes.h"
#include <emmintrin.h>
#include "dryden_code_gen_private.h"
#include <math.h>

/* Block signals (default storage) */
B_dryden_code_gen_T dryden_code_gen_B;

/* Continuous states */
X_dryden_code_gen_T dryden_code_gen_X;

/* Block states (default storage) */
DW_dryden_code_gen_T dryden_code_gen_DW;

/* External outputs (root outports fed by signals with default storage) */
ExtY_dryden_code_gen_T dryden_code_gen_Y;

/* Real-time model */
static RT_MODEL_dryden_code_gen_T dryden_code_gen_M_;
RT_MODEL_dryden_code_gen_T *const dryden_code_gen_M = &dryden_code_gen_M_;

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 16;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  dryden_code_gen_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  dryden_code_gen_step();
  dryden_code_gen_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  dryden_code_gen_step();
  dryden_code_gen_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  dryden_code_gen_step();
  dryden_code_gen_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
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
  real_T rtb_VectorConcatenate_c[3];
  real_T rtb_WhiteNoise_idx_0;
  real_T rtb_WhiteNoise_idx_1;
  real_T rtb_WhiteNoise_idx_2;
  real_T rtb_WhiteNoise_idx_3;
  int32_T i;
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

  /* Outputs for Enabled SubSystem: '<S5>/Hvgw(s)' incorporates:
   *  EnablePort: '<S19>/Enable'
   */
  if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
    /* RandomNumber: '<S14>/White Noise' incorporates:
     *  Product: '<S14>/Product'
     */
    rtb_WhiteNoise_idx_0 = dryden_code_gen_ConstB.Divide[0] *
      dryden_code_gen_DW.NextOutput[0];
    rtb_WhiteNoise_idx_1 = dryden_code_gen_ConstB.Divide[1] *
      dryden_code_gen_DW.NextOutput[1];
    rtb_WhiteNoise_idx_2 = dryden_code_gen_ConstB.Divide[2] *
      dryden_code_gen_DW.NextOutput[2];
    rtb_WhiteNoise_idx_3 = dryden_code_gen_ConstB.Divide[3] *
      dryden_code_gen_DW.NextOutput[3];

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
    if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
      /* Product: '<S18>/Lug//V1' */
      dryden_code_gen_B.LugV1_e[0] = dryden_code_gen_ConstB.sqrt_n[0] *
        rtb_WhiteNoise_idx_0;
      dryden_code_gen_B.LugV1_e[1] = dryden_code_gen_ConstB.sqrt_n[1] *
        rtb_WhiteNoise_idx_0;
    }

    /* Product: '<S18>/w' incorporates:
     *  Integrator: '<S18>/ug_p'
     *  Sum: '<S18>/Sum'
     */
    dryden_code_gen_B.w_f[0] = (dryden_code_gen_B.LugV1_e[0] -
      dryden_code_gen_X.ug_p_CSTATE[0]) / dryden_code_gen_ConstB.LugV[0];
    dryden_code_gen_B.w_f[1] = (dryden_code_gen_B.LugV1_e[1] -
      dryden_code_gen_X.ug_p_CSTATE[1]) / dryden_code_gen_ConstB.LugV[1];

    /* Product: '<S18>/w1' incorporates:
     *  Integrator: '<S18>/ug_p'
     */
    dryden_code_gen_B.w1_p[0] = dryden_code_gen_X.ug_p_CSTATE[0] *
      dryden_code_gen_ConstB.sigma_ugsigma_vg;
    dryden_code_gen_B.w1_p[1] = dryden_code_gen_X.ug_p_CSTATE[1] *
      dryden_code_gen_ConstB.MediumHighAltitudeIntensity;
  }

  /* End of Outputs for SubSystem: '<S5>/Hugw(s)' */

  /* Outputs for Enabled SubSystem: '<S5>/Hvgw(s)' incorporates:
   *  EnablePort: '<S19>/Enable'
   */
  if (dryden_code_gen_DW.Hvgws_MODE) {
    if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
      /* Product: '<S19>/Lug//V1' */
      dryden_code_gen_B.LugV1_h[0] = dryden_code_gen_ConstB.sqrt_b[0] *
        rtb_WhiteNoise_idx_1;
      dryden_code_gen_B.LugV1_h[1] = dryden_code_gen_ConstB.sqrt_b[1] *
        rtb_WhiteNoise_idx_1;
    }

    /* Product: '<S19>/w' incorporates:
     *  Integrator: '<S19>/vg_p1'
     *  Sum: '<S19>/Sum'
     */
    dryden_code_gen_B.w_e[0] = (dryden_code_gen_B.LugV1_h[0] -
      dryden_code_gen_X.vg_p1_CSTATE[0]) / dryden_code_gen_ConstB.LvgV[0];

    /* Product: '<S19>/w ' incorporates:
     *  Gain: '<S19>/sqrt(3)'
     *  Integrator: '<S19>/vg_p1'
     *  Integrator: '<S19>/vgw_p2'
     *  Product: '<S19>/Lvg//V '
     *  Sum: '<S19>/Sum1'
     */
    dryden_code_gen_B.w_i[0] = (dryden_code_gen_B.w_e[0] *
      dryden_code_gen_ConstB.LvgV[0] * 1.7320508075688772 +
      (dryden_code_gen_X.vg_p1_CSTATE[0] - dryden_code_gen_X.vgw_p2_CSTATE[0])) /
      dryden_code_gen_ConstB.LvgV[0];

    /* Product: '<S19>/w' incorporates:
     *  Integrator: '<S19>/vg_p1'
     *  Sum: '<S19>/Sum'
     */
    dryden_code_gen_B.w_e[1] = (dryden_code_gen_B.LugV1_h[1] -
      dryden_code_gen_X.vg_p1_CSTATE[1]) / dryden_code_gen_ConstB.LvgV[1];

    /* Product: '<S19>/w ' incorporates:
     *  Gain: '<S19>/sqrt(3)'
     *  Integrator: '<S19>/vg_p1'
     *  Integrator: '<S19>/vgw_p2'
     *  Product: '<S19>/Lvg//V '
     *  Sum: '<S19>/Sum1'
     */
    dryden_code_gen_B.w_i[1] = (dryden_code_gen_B.w_e[1] *
      dryden_code_gen_ConstB.LvgV[1] * 1.7320508075688772 +
      (dryden_code_gen_X.vg_p1_CSTATE[1] - dryden_code_gen_X.vgw_p2_CSTATE[1])) /
      dryden_code_gen_ConstB.LvgV[1];

    /* Product: '<S19>/w 1' incorporates:
     *  Integrator: '<S19>/vgw_p2'
     */
    dryden_code_gen_B.w1[0] = dryden_code_gen_ConstB.sigma_ugsigma_vg *
      dryden_code_gen_X.vgw_p2_CSTATE[0];
    dryden_code_gen_B.w1[1] = dryden_code_gen_ConstB.MediumHighAltitudeIntensity
      * dryden_code_gen_X.vgw_p2_CSTATE[1];
  }

  /* End of Outputs for SubSystem: '<S5>/Hvgw(s)' */

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
    if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
      /* Product: '<S20>/Lug//V1' */
      dryden_code_gen_B.LugV1[0] = dryden_code_gen_ConstB.sqrt1[0] *
        rtb_WhiteNoise_idx_2;
      dryden_code_gen_B.LugV1[1] = dryden_code_gen_ConstB.sqrt1[1] *
        rtb_WhiteNoise_idx_2;
    }

    /* Product: '<S20>/w' incorporates:
     *  Integrator: '<S20>/wg_p1'
     *  Sum: '<S20>/Sum'
     */
    dryden_code_gen_B.w[0] = (dryden_code_gen_B.LugV1[0] -
      dryden_code_gen_X.wg_p1_CSTATE[0]) / dryden_code_gen_ConstB.LwgV[0];

    /* Product: '<S20>/w ' incorporates:
     *  Integrator: '<S20>/wg_p1'
     *  Integrator: '<S20>/wg_p2'
     *  Product: '<S20>/Lwg//V '
     *  Sum: '<S20>/Sum1'
     */
    dryden_code_gen_B.w_c[0] = (dryden_code_gen_B.w[0] *
      dryden_code_gen_ConstB.sqrt_o * dryden_code_gen_ConstB.LwgV[0] +
      (dryden_code_gen_X.wg_p1_CSTATE[0] - dryden_code_gen_X.wg_p2_CSTATE[0])) /
      dryden_code_gen_ConstB.LwgV[0];

    /* Product: '<S20>/w' incorporates:
     *  Integrator: '<S20>/wg_p1'
     *  Sum: '<S20>/Sum'
     */
    dryden_code_gen_B.w[1] = (dryden_code_gen_B.LugV1[1] -
      dryden_code_gen_X.wg_p1_CSTATE[1]) / dryden_code_gen_ConstB.LwgV[1];

    /* Product: '<S20>/w ' incorporates:
     *  Integrator: '<S20>/wg_p1'
     *  Integrator: '<S20>/wg_p2'
     *  Product: '<S20>/Lwg//V '
     *  Sum: '<S20>/Sum1'
     */
    dryden_code_gen_B.w_c[1] = (dryden_code_gen_B.w[1] *
      dryden_code_gen_ConstB.sqrt_o * dryden_code_gen_ConstB.LwgV[1] +
      (dryden_code_gen_X.wg_p1_CSTATE[1] - dryden_code_gen_X.wg_p2_CSTATE[1])) /
      dryden_code_gen_ConstB.LwgV[1];

    /* Product: '<S20>/Lwg//V 1' incorporates:
     *  Integrator: '<S20>/wg_p2'
     */
    dryden_code_gen_B.LwgV1[0] = dryden_code_gen_ConstB.sigma_wg *
      dryden_code_gen_X.wg_p2_CSTATE[0];
    dryden_code_gen_B.LwgV1[1] =
      dryden_code_gen_ConstB.MediumHighAltitudeIntensity *
      dryden_code_gen_X.wg_p2_CSTATE[1];
  }

  /* End of Outputs for SubSystem: '<S5>/Hwgw(s)' */

  /* Outputs for Enabled SubSystem: '<S4>/Hqgw' incorporates:
   *  EnablePort: '<S16>/Enable'
   */
  if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
    /* If: '<S10>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
    if (rtsiIsModeUpdateTimeStep(&dryden_code_gen_M->solverInfo)) {
      if (dryden_code_gen_ConstB.UnitConversion_a <= 1000.0) {
        dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifHei = 0;
      } else if (dryden_code_gen_ConstB.UnitConversion_a >= 2000.0) {
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

      /* Sum: '<S38>/Sum' incorporates:
       *  Product: '<S38>/Product1'
       *  Product: '<S38>/Product2'
       */
      rtb_VectorConcatenate_c[0] = dryden_code_gen_B.w1_p[0] *
        dryden_code_gen_ConstB.TrigonometricFunction_o2_c -
        dryden_code_gen_ConstB.TrigonometricFunction_o1_p *
        dryden_code_gen_B.w1[0];

      /* Sum: '<S38>/Sum1' incorporates:
       *  Product: '<S38>/Product1'
       *  Product: '<S38>/Product2'
       */
      rtb_VectorConcatenate_c[1] =
        dryden_code_gen_ConstB.TrigonometricFunction_o1_p *
        dryden_code_gen_B.w1_p[0] + dryden_code_gen_B.w1[0] *
        dryden_code_gen_ConstB.TrigonometricFunction_o2_c;

      /* End of Outputs for SubSystem: '<S10>/Low altitude  velocities' */
      for (i = 0; i <= 0; i += 2) {
        /* Outputs for IfAction SubSystem: '<S10>/Low altitude  velocities' incorporates:
         *  ActionPort: '<S32>/Action Port'
         */
        _mm_storeu_pd(&dryden_code_gen_Y.uvw_ptr[i], _mm_set1_pd(0.0));
        tmp = _mm_loadu_pd(&dryden_code_gen_Y.uvw_ptr[i]);
        _mm_storeu_pd(&dryden_code_gen_Y.uvw_ptr[i], _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i]),
           _mm_set1_pd(rtb_VectorConcatenate_c[0])), tmp));
        tmp = _mm_loadu_pd(&dryden_code_gen_Y.uvw_ptr[i]);
        _mm_storeu_pd(&dryden_code_gen_Y.uvw_ptr[i], _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i + 3]),
           _mm_set1_pd(rtb_VectorConcatenate_c[1])), tmp));
        tmp = _mm_loadu_pd(&dryden_code_gen_Y.uvw_ptr[i]);
        _mm_storeu_pd(&dryden_code_gen_Y.uvw_ptr[i], _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i + 6]),
           _mm_set1_pd(rtb_VectorConcatenate_c[2])), tmp));

        /* End of Outputs for SubSystem: '<S10>/Low altitude  velocities' */
      }

      /* Outputs for IfAction SubSystem: '<S10>/Low altitude  velocities' incorporates:
       *  ActionPort: '<S32>/Action Port'
       */
      /* Reshape: '<S37>/Reshape1' incorporates:
       *  Concatenate: '<S37>/Vector Concatenate'
       *  Concatenate: '<S42>/Vector Concatenate'
       *  Merge: '<S34>/Merge'
       *  Product: '<S37>/Product'
       */
      for (i = 2; i < 3; i++) {
        dryden_code_gen_Y.uvw_ptr[i] = 0.0;
        dryden_code_gen_Y.uvw_ptr[i] +=
          dryden_code_gen_ConstB.VectorConcatenate[i] * rtb_VectorConcatenate_c
          [0];
        dryden_code_gen_Y.uvw_ptr[i] +=
          dryden_code_gen_ConstB.VectorConcatenate[i + 3] *
          rtb_VectorConcatenate_c[1];
        dryden_code_gen_Y.uvw_ptr[i] +=
          dryden_code_gen_ConstB.VectorConcatenate[i + 6] *
          rtb_VectorConcatenate_c[2];
      }

      /* End of Reshape: '<S37>/Reshape1' */
      /* End of Outputs for SubSystem: '<S10>/Low altitude  velocities' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S10>/Medium//High  altitude velocities' incorporates:
       *  ActionPort: '<S33>/Action Port'
       */
      /* Gain: '<S33>/Gain' incorporates:
       *  Merge: '<S34>/Merge'
       */
      dryden_code_gen_Y.uvw_ptr[0] = dryden_code_gen_B.w1_p[1];
      dryden_code_gen_Y.uvw_ptr[1] = dryden_code_gen_B.w1[1];
      dryden_code_gen_Y.uvw_ptr[2] = dryden_code_gen_B.LwgV1[1];

      /* End of Outputs for SubSystem: '<S10>/Medium//High  altitude velocities' */
      break;

     case 2:
      /* Outputs for IfAction SubSystem: '<S10>/Interpolate  velocities' incorporates:
       *  ActionPort: '<S31>/Action Port'
       */
      /* Sum: '<S36>/Sum' incorporates:
       *  Product: '<S36>/Product1'
       *  Product: '<S36>/Product2'
       */
      rtb_WhiteNoise_idx_0 = dryden_code_gen_B.w1_p[0] *
        dryden_code_gen_ConstB.TrigonometricFunction_o2 -
        dryden_code_gen_ConstB.TrigonometricFunction_o1 * dryden_code_gen_B.w1[0];

      /* Sum: '<S36>/Sum1' incorporates:
       *  Product: '<S36>/Product1'
       *  Product: '<S36>/Product2'
       */
      rtb_WhiteNoise_idx_1 = dryden_code_gen_ConstB.TrigonometricFunction_o1 *
        dryden_code_gen_B.w1_p[0] + dryden_code_gen_B.w1[0] *
        dryden_code_gen_ConstB.TrigonometricFunction_o2;

      /* SignalConversion generated from: '<S35>/Vector Concatenate' */
      rtb_WhiteNoise_idx_2 = dryden_code_gen_B.LwgV1[0];

      /* End of Outputs for SubSystem: '<S10>/Interpolate  velocities' */
      for (i = 0; i <= 0; i += 2) {
        /* Outputs for IfAction SubSystem: '<S10>/Interpolate  velocities' incorporates:
         *  ActionPort: '<S31>/Action Port'
         */
        _mm_storeu_pd(&rtb_VectorConcatenate_c[i], _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i + 6]),
           _mm_set1_pd(rtb_WhiteNoise_idx_2)), _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i + 3]),
           _mm_set1_pd(rtb_WhiteNoise_idx_1)), _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i]),
           _mm_set1_pd(rtb_WhiteNoise_idx_0)), _mm_set1_pd(0.0)))));

        /* End of Outputs for SubSystem: '<S10>/Interpolate  velocities' */
      }

      /* Outputs for IfAction SubSystem: '<S10>/Interpolate  velocities' incorporates:
       *  ActionPort: '<S31>/Action Port'
       */
      /* Product: '<S35>/Product' incorporates:
       *  Concatenate: '<S35>/Vector Concatenate'
       *  Concatenate: '<S42>/Vector Concatenate'
       */
      for (i = 2; i < 3; i++) {
        rtb_VectorConcatenate_c[i] = (dryden_code_gen_ConstB.VectorConcatenate[i
          + 3] * rtb_WhiteNoise_idx_1 +
          dryden_code_gen_ConstB.VectorConcatenate[i] * rtb_WhiteNoise_idx_0) +
          dryden_code_gen_ConstB.VectorConcatenate[i + 6] * rtb_WhiteNoise_idx_2;
      }

      /* End of Product: '<S35>/Product' */

      /* Sum: '<S31>/Sum3' incorporates:
       *  Merge: '<S34>/Merge'
       *  Product: '<S31>/Product1'
       *  Sum: '<S31>/Sum2'
       */
      dryden_code_gen_Y.uvw_ptr[0] = (dryden_code_gen_B.w1_p[1] -
        rtb_VectorConcatenate_c[0]) * dryden_code_gen_ConstB.Sum1 /
        dryden_code_gen_ConstB.Sum + rtb_VectorConcatenate_c[0];
      dryden_code_gen_Y.uvw_ptr[1] = (dryden_code_gen_B.w1[1] -
        rtb_VectorConcatenate_c[1]) * dryden_code_gen_ConstB.Sum1 /
        dryden_code_gen_ConstB.Sum + rtb_VectorConcatenate_c[1];
      dryden_code_gen_Y.uvw_ptr[2] = (dryden_code_gen_B.LwgV1[1] -
        rtb_VectorConcatenate_c[2]) * dryden_code_gen_ConstB.Sum1 /
        dryden_code_gen_ConstB.Sum + rtb_VectorConcatenate_c[2];

      /* End of Outputs for SubSystem: '<S10>/Interpolate  velocities' */
      break;
    }

    /* End of If: '<S10>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */

    /* Outport: '<Root>/uvw_ptr' incorporates:
     *  Merge: '<S34>/Merge'
     *  UnitConversion: '<S1>/Unit Conversion'
     */
    /* Unit Conversion - from: ft/s to: m/s
       Expression: output = (0.3048*input) + (0) */
    dryden_code_gen_Y.uvw_ptr[0] *= 0.3048;
    dryden_code_gen_Y.uvw_ptr[1] *= 0.3048;
    dryden_code_gen_Y.uvw_ptr[2] *= 0.3048;

    /* Outputs for Enabled SubSystem: '<S4>/Hpgw' incorporates:
     *  EnablePort: '<S15>/Enable'
     */
    if (rtsiIsModeUpdateTimeStep(&dryden_code_gen_M->solverInfo) &&
        (!dryden_code_gen_DW.Hpgw_MODE)) {
      /* InitializeConditions for Integrator: '<S15>/pgw_p' */
      dryden_code_gen_X.pgw_p_CSTATE[0] = 0.0;
      dryden_code_gen_X.pgw_p_CSTATE[1] = 0.0;
      dryden_code_gen_DW.Hpgw_MODE = true;
    }

    /* End of Outputs for SubSystem: '<S4>/Hpgw' */
    if (rtsiIsModeUpdateTimeStep(&dryden_code_gen_M->solverInfo) &&
        (!dryden_code_gen_DW.Hqgw_MODE)) {
      /* InitializeConditions for Integrator: '<S16>/qgw_p' */
      dryden_code_gen_X.qgw_p_CSTATE[0] = 0.0;
      dryden_code_gen_X.qgw_p_CSTATE[1] = 0.0;
      dryden_code_gen_DW.Hqgw_MODE = true;
    }
  }

  /* End of Outputs for SubSystem: '<S4>/Hqgw' */

  /* Outputs for Enabled SubSystem: '<S4>/Hpgw' incorporates:
   *  EnablePort: '<S15>/Enable'
   */
  if (dryden_code_gen_DW.Hpgw_MODE) {
    if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
      /* Product: '<S15>/Lug//V1' */
      dryden_code_gen_B.LugV1_k[0] = dryden_code_gen_ConstB.w2[0] *
        rtb_WhiteNoise_idx_3;
      dryden_code_gen_B.LugV1_k[1] = dryden_code_gen_ConstB.w2[1] *
        rtb_WhiteNoise_idx_3;
    }

    /* Product: '<S15>/w' incorporates:
     *  Integrator: '<S15>/pgw_p'
     *  Sum: '<S15>/Sum'
     */
    dryden_code_gen_B.w_g[0] = (dryden_code_gen_B.LugV1_k[0] -
      dryden_code_gen_X.pgw_p_CSTATE[0]) * dryden_code_gen_ConstB.w3;
    dryden_code_gen_B.w_g[1] = (dryden_code_gen_B.LugV1_k[1] -
      dryden_code_gen_X.pgw_p_CSTATE[1]) * dryden_code_gen_ConstB.w3;

    /* Product: '<S15>/sigma_w' incorporates:
     *  Integrator: '<S15>/pgw_p'
     */
    dryden_code_gen_B.sigma_w[0] = dryden_code_gen_ConstB.sigma_wg *
      dryden_code_gen_X.pgw_p_CSTATE[0];
    dryden_code_gen_B.sigma_w[1] =
      dryden_code_gen_ConstB.MediumHighAltitudeIntensity *
      dryden_code_gen_X.pgw_p_CSTATE[1];
  }

  /* End of Outputs for SubSystem: '<S4>/Hpgw' */

  /* Outputs for Enabled SubSystem: '<S4>/Hqgw' incorporates:
   *  EnablePort: '<S16>/Enable'
   */
  if (dryden_code_gen_DW.Hqgw_MODE) {
    /* Product: '<S16>/w' */
    rtb_WhiteNoise_idx_3 = dryden_code_gen_ConstB.pi4 /
      dryden_code_gen_ConstB.UnitConversion_a5;

    /* Product: '<S16>/w' incorporates:
     *  Integrator: '<S16>/qgw_p'
     *  Product: '<S16>/wg//V'
     *  Sum: '<S16>/Sum'
     */
    dryden_code_gen_B.w_p[0] = (dryden_code_gen_B.LwgV1[0] /
      dryden_code_gen_ConstB.UnitConversion_c - dryden_code_gen_X.qgw_p_CSTATE[0])
      * rtb_WhiteNoise_idx_3;
    dryden_code_gen_B.w_p[1] = (dryden_code_gen_B.LwgV1[1] /
      dryden_code_gen_ConstB.UnitConversion_c - dryden_code_gen_X.qgw_p_CSTATE[1])
      * rtb_WhiteNoise_idx_3;
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
    /* Product: '<S17>/w' */
    rtb_WhiteNoise_idx_3 = dryden_code_gen_ConstB.pi3 /
      dryden_code_gen_ConstB.UnitConversion_a5;

    /* Product: '<S17>/w' incorporates:
     *  Integrator: '<S17>/rgw_p'
     *  Product: '<S17>/vg//V'
     *  Sum: '<S17>/Sum'
     */
    dryden_code_gen_B.w_ic[0] = (dryden_code_gen_B.w1[0] /
      dryden_code_gen_ConstB.UnitConversion_c - dryden_code_gen_X.rgw_p_CSTATE[0])
      * rtb_WhiteNoise_idx_3;
    dryden_code_gen_B.w_ic[1] = (dryden_code_gen_B.w1[1] /
      dryden_code_gen_ConstB.UnitConversion_c - dryden_code_gen_X.rgw_p_CSTATE[1])
      * rtb_WhiteNoise_idx_3;
  }

  /* End of Outputs for SubSystem: '<S4>/Hrgw' */
  if (rtmIsMajorTimeStep(dryden_code_gen_M)) {
    /* If: '<S9>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
    if (rtsiIsModeUpdateTimeStep(&dryden_code_gen_M->solverInfo)) {
      if (dryden_code_gen_ConstB.UnitConversion_a <= 1000.0) {
        dryden_code_gen_DW.ifHeightMaxlowaltitudeelseifH_i = 0;
      } else if (dryden_code_gen_ConstB.UnitConversion_a >= 2000.0) {
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

      /* Sum: '<S30>/Sum' incorporates:
       *  Product: '<S30>/Product1'
       *  Product: '<S30>/Product2'
       */
      rtb_VectorConcatenate_c[0] = dryden_code_gen_B.sigma_w[0] *
        dryden_code_gen_ConstB.TrigonometricFunction1_o2 -
        dryden_code_gen_ConstB.TrigonometricFunction1_o1 *
        dryden_code_gen_B.w_p[0];

      /* Sum: '<S30>/Sum1' incorporates:
       *  Product: '<S30>/Product1'
       *  Product: '<S30>/Product2'
       */
      rtb_VectorConcatenate_c[1] =
        dryden_code_gen_ConstB.TrigonometricFunction1_o1 *
        dryden_code_gen_B.sigma_w[0] + dryden_code_gen_B.w_p[0] *
        dryden_code_gen_ConstB.TrigonometricFunction1_o2;

      /* End of Outputs for SubSystem: '<S9>/Low altitude  rates' */
      for (i = 0; i <= 0; i += 2) {
        /* Outputs for IfAction SubSystem: '<S9>/Low altitude  rates' incorporates:
         *  ActionPort: '<S24>/Action Port'
         */
        _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_set1_pd(0.0));
        tmp = _mm_loadu_pd(&dryden_code_gen_Y.pqr_ptr[i]);
        _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i]),
           _mm_set1_pd(rtb_VectorConcatenate_c[0])), tmp));
        tmp = _mm_loadu_pd(&dryden_code_gen_Y.pqr_ptr[i]);
        _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i + 3]),
           _mm_set1_pd(rtb_VectorConcatenate_c[1])), tmp));
        tmp = _mm_loadu_pd(&dryden_code_gen_Y.pqr_ptr[i]);
        _mm_storeu_pd(&dryden_code_gen_Y.pqr_ptr[i], _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i + 6]),
           _mm_set1_pd(rtb_VectorConcatenate_c[2])), tmp));

        /* End of Outputs for SubSystem: '<S9>/Low altitude  rates' */
      }

      /* Outputs for IfAction SubSystem: '<S9>/Low altitude  rates' incorporates:
       *  ActionPort: '<S24>/Action Port'
       */
      /* Reshape: '<S29>/Reshape1' incorporates:
       *  Concatenate: '<S29>/Vector Concatenate'
       *  Concatenate: '<S42>/Vector Concatenate'
       *  Product: '<S29>/Product'
       */
      for (i = 2; i < 3; i++) {
        dryden_code_gen_Y.pqr_ptr[i] = 0.0;
        dryden_code_gen_Y.pqr_ptr[i] +=
          dryden_code_gen_ConstB.VectorConcatenate[i] * rtb_VectorConcatenate_c
          [0];
        dryden_code_gen_Y.pqr_ptr[i] +=
          dryden_code_gen_ConstB.VectorConcatenate[i + 3] *
          rtb_VectorConcatenate_c[1];
        dryden_code_gen_Y.pqr_ptr[i] +=
          dryden_code_gen_ConstB.VectorConcatenate[i + 6] *
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
      /* Sum: '<S28>/Sum' incorporates:
       *  Product: '<S28>/Product1'
       *  Product: '<S28>/Product2'
       */
      rtb_WhiteNoise_idx_0 = dryden_code_gen_B.sigma_w[0] *
        dryden_code_gen_ConstB.TrigonometricFunction_o2_e -
        dryden_code_gen_ConstB.TrigonometricFunction_o1_px *
        dryden_code_gen_B.w_p[0];

      /* Sum: '<S28>/Sum1' incorporates:
       *  Product: '<S28>/Product1'
       *  Product: '<S28>/Product2'
       */
      rtb_WhiteNoise_idx_1 = dryden_code_gen_ConstB.TrigonometricFunction_o1_px *
        dryden_code_gen_B.sigma_w[0] + dryden_code_gen_B.w_p[0] *
        dryden_code_gen_ConstB.TrigonometricFunction_o2_e;

      /* SignalConversion generated from: '<S27>/Vector Concatenate' */
      rtb_WhiteNoise_idx_2 = dryden_code_gen_B.w_ic[0];

      /* End of Outputs for SubSystem: '<S9>/Interpolate  rates' */
      for (i = 0; i <= 0; i += 2) {
        /* Outputs for IfAction SubSystem: '<S9>/Interpolate  rates' incorporates:
         *  ActionPort: '<S23>/Action Port'
         */
        _mm_storeu_pd(&rtb_VectorConcatenate_c[i], _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i + 6]),
           _mm_set1_pd(rtb_WhiteNoise_idx_2)), _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i + 3]),
           _mm_set1_pd(rtb_WhiteNoise_idx_1)), _mm_add_pd(_mm_mul_pd
          (_mm_loadu_pd(&dryden_code_gen_ConstB.VectorConcatenate[i]),
           _mm_set1_pd(rtb_WhiteNoise_idx_0)), _mm_set1_pd(0.0)))));

        /* End of Outputs for SubSystem: '<S9>/Interpolate  rates' */
      }

      /* Outputs for IfAction SubSystem: '<S9>/Interpolate  rates' incorporates:
       *  ActionPort: '<S23>/Action Port'
       */
      /* Product: '<S27>/Product' incorporates:
       *  Concatenate: '<S27>/Vector Concatenate'
       *  Concatenate: '<S42>/Vector Concatenate'
       */
      for (i = 2; i < 3; i++) {
        rtb_VectorConcatenate_c[i] = (dryden_code_gen_ConstB.VectorConcatenate[i
          + 3] * rtb_WhiteNoise_idx_1 +
          dryden_code_gen_ConstB.VectorConcatenate[i] * rtb_WhiteNoise_idx_0) +
          dryden_code_gen_ConstB.VectorConcatenate[i + 6] * rtb_WhiteNoise_idx_2;
      }

      /* End of Product: '<S27>/Product' */

      /* Sum: '<S23>/Sum3' incorporates:
       *  Product: '<S23>/Product1'
       *  Sum: '<S23>/Sum2'
       */
      dryden_code_gen_Y.pqr_ptr[0] = (dryden_code_gen_B.sigma_w[1] -
        rtb_VectorConcatenate_c[0]) * dryden_code_gen_ConstB.Sum1_g /
        dryden_code_gen_ConstB.Sum_n + rtb_VectorConcatenate_c[0];
      dryden_code_gen_Y.pqr_ptr[1] = (dryden_code_gen_B.w_p[1] -
        rtb_VectorConcatenate_c[1]) * dryden_code_gen_ConstB.Sum1_g /
        dryden_code_gen_ConstB.Sum_n + rtb_VectorConcatenate_c[1];
      dryden_code_gen_Y.pqr_ptr[2] = (dryden_code_gen_B.w_ic[1] -
        rtb_VectorConcatenate_c[2]) * dryden_code_gen_ConstB.Sum1_g /
        dryden_code_gen_ConstB.Sum_n + rtb_VectorConcatenate_c[2];

      /* End of Outputs for SubSystem: '<S9>/Interpolate  rates' */
      break;
    }

    /* End of If: '<S9>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  }

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
  dryden_code_gen_M->intgData.y = dryden_code_gen_M->odeY;
  dryden_code_gen_M->intgData.f[0] = dryden_code_gen_M->odeF[0];
  dryden_code_gen_M->intgData.f[1] = dryden_code_gen_M->odeF[1];
  dryden_code_gen_M->intgData.f[2] = dryden_code_gen_M->odeF[2];
  dryden_code_gen_M->intgData.f[3] = dryden_code_gen_M->odeF[3];
  dryden_code_gen_M->contStates = ((X_dryden_code_gen_T *) &dryden_code_gen_X);
  rtsiSetSolverData(&dryden_code_gen_M->solverInfo, (void *)
                    &dryden_code_gen_M->intgData);
  rtsiSetIsMinorTimeStepWithModeChange(&dryden_code_gen_M->solverInfo, false);
  rtsiSetSolverName(&dryden_code_gen_M->solverInfo,"ode4");
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
