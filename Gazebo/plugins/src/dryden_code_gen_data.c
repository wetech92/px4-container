/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: dryden_code_gen_data.c
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

/* Invariant block signals (default storage) */
const ConstB_dryden_code_gen_T dryden_code_gen_ConstB = {
  0.78539816339744828,                 /* '<S3>/Unit Conversion' */
  19.685039370078737,                  /* '<S13>/Unit Conversion' */
  1.9685039370078738,                  /* '<S22>/sigma_wg ' */
  32.808398950131227,                  /* '<S7>/Unit Conversion' */
  1749.9999999999998,                  /* '<S41>/Unit Conversion' */
  0.0,                   /* '<S21>/PreLook-Up Index Search  (prob of exceed)' */

  { 1.7724538509055159, 1.7724538509055159, 1.7724538509055159,
    1.7724538509055159 },              /* '<S14>/Sqrt' */
  0.31622776601683794,                 /* '<S14>/Sqrt1' */

  { 5.6049912163979281, 5.6049912163979281, 5.6049912163979281,
    5.6049912163979281 },              /* '<S14>/Divide' */
  1000.0,                              /* '<S31>/Sum' */
  1000.0,                              /* '<S23>/Sum' */
  1.7320508075688772,                  /* '<S20>/sqrt' */
  0.02393893602035423,                 /* '<S15>/w4' */
  0.53684734216761054,                 /* '<S15>/u^1//6' */
  2U                     /* '<S21>/PreLook-Up Index Search  (prob of exceed)' */
};

/* Constant parameters (default storage) */
const ConstP_dryden_code_gen_T dryden_code_gen_ConstP = {
  /* Expression: h_vec
   * Referenced by: '<S21>/PreLook-Up Index Search  (altitude)'
   */
  { 500.0, 1750.0, 3750.0, 7500.0, 15000.0, 25000.0, 35000.0, 45000.0, 55000.0,
    65000.0, 75000.0, 80000.0 },

  /* Expression: sigma_vec'
   * Referenced by: '<S21>/Medium//High Altitude Intensity'
   */
  { 3.2, 2.2, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.2, 3.6, 3.3,
    1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.6, 6.9, 7.4, 6.7, 4.6, 2.7,
    0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 8.6, 9.6, 10.6, 10.1, 8.0, 6.6, 5.0, 4.2, 2.7,
    0.0, 0.0, 0.0, 11.8, 13.0, 16.0, 15.1, 11.6, 9.7, 8.1, 8.2, 7.9, 4.9, 3.2,
    2.1, 15.6, 17.6, 23.0, 23.6, 22.1, 20.0, 16.0, 15.1, 12.1, 7.9, 6.2, 5.1,
    18.7, 21.5, 28.4, 30.2, 30.7, 31.0, 25.2, 23.1, 17.5, 10.7, 8.4, 7.2 },

  /* Computed Parameter: MediumHighAltitudeIntensity_max
   * Referenced by: '<S21>/Medium//High Altitude Intensity'
   */
  { 11U, 6U }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */