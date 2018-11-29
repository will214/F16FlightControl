/*
 * SS_F16_Block_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "SS_F16_Block".
 *
 * Model version              : 1.83
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C source code generated on : Sun Nov 25 16:30:51 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "SS_F16_Block.h"
#include "SS_F16_Block_private.h"

/* Block parameters (default storage) */
P_SS_F16_Block_T SS_F16_Block_P = {
  /* Variable: A
   * Referenced by: '<Root>/State-Space'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.00010799293058414518, 2.0764354657803074E-6, -4.7658563676409425E-23,
    -7.9261874213685373E-22, 1.8011346804342971E-20, 4.4956159840888016E-21, 0.0,
    0.0, 0.0, 0.0, -8.2098323102723774E-5, 0.0, 0.0, -38.929316866019882, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.064144690084976821, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -2.636675050113225E-9, 0.0, 499.99999999057667, 0.0, 0.0, 0.0,
    -32.169999999391692, -8.2358437717929536E-14, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 499.99999999166658, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99999999999972566, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.01327713196721058, -0.00025528622617455182,
    2.9296780627041204E-21, 9.74480790707947E-20, -2.2143952121093648E-18,
    -5.5271105592769882E-19, 0.0, 0.0, 0.0, 0.0, -0.04178722897973907, 0.0,
    -2.636675050113225E-9, 0.0, -499.99999999024703, 0.0, 0.0, 0.0,
    -7.32588065590331, -0.63976289270895537, 0.0, -8.4091617553138209E-16,
    -1.5679193372020985, 3.5048354230690963E-18, 0.0, 0.0, 0.0, 0.0,
    1162.7672901184353, -415.39440146984685, 0.0, 499.99999999166658, 0.0, 0.0,
    0.0, 0.0, 7.3255115047696121E-16, 0.0, -0.20220303812121418,
    -22.921887220844113, 0.0, 6.0052035247721873, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.99999999999999989, 0.0, 0.0, 0.0, 0.0, 0.0782722976893603,
    -2.2542147178795009, 0.0, -0.040397614801989522, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.99999999999999989, 0.0, -1.1964964211660098,
    0.93781063577324686, 0.0, 0.0, -0.87906116362025766, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.078095700325048187, 0.0, 1.0030448336985041, 0.0,
    0.0, -0.991939483881862, 0.54083800178867814, 0.0, -0.31464048806595835, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0015652406964063643,
    -2.4447713672624231E-7, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.073970431180274382, -0.0013566981877241797,
    0.0, 0.0, -0.11365790765428867, 0.0, 0.0, -20.2, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00017235182379367412,
    -0.46228913379791686, 0.0, -0.024372898821110971, 0.0, 0.0, -20.2, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00050583892411825945,
    0.056861906142393732, 0.0, -0.04687037737055693, 0.0, 0.0, 0.0, -20.2, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -7.3529411764705879, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.147058823529411, -7.25 },

  /* Variable: B
   * Referenced by: '<Root>/State-Space'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 20.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 20.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.2, 0.0, 0.0 },

  /* Variable: C
   * Referenced by: '<Root>/State-Space'
   */
  { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 8.3393093906567562E-7, -7.4063783919000469E-22,
    -3.2432222001807735E-5, 1.8601334632403296E-6, -0.00609443385480135,
    -0.048197247300393209, 0.0, 0.0, 0.0, 57.295779513082323, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.00012394659155261434, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 57.295779513082323, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.00012394659162805229, 0.0, -9.6796975870018013E-6, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 57.295779513082323, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.00010252718459792228, 9.1057315307387919E-20, 0.0039873618503135574,
    0.00094678941074957093, 0.74927684727628519, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 57.295779513082323, 0.0, 0.0, 0.0, 0.0, 0.547084294494093, 0.0,
    9.9297818190708345, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    57.295779513082323, 0.0, 0.0, 0.0, 0.0, -3.142336018692101, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 57.295779513082323, 0.0,
    0.0, 0.0, 0.0064285441042904127, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 57.295779513082323, 0.0, 0.038171353208132912, 0.0,
    0.96641539622521744, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 57.295779513082323, 0.0, 0.078089844111481566, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    4.8797370362047827E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0039336489358807016, 0.0,
    0.020840761697278259, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0026784332658928659, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00786098906132684,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Variable: DisAil_1
   * Referenced by: '<S1>/Step'
   */
  0.0,

  /* Variable: DisAil_2
   * Referenced by: '<S1>/Step1'
   */
  -0.0,

  /* Variable: DisAil_3
   * Referenced by: '<S1>/Step2'
   */
  0.0,

  /* Variable: DisEle_1
   * Referenced by: '<S2>/Step'
   */
  2.0,

  /* Variable: DisEle_2
   * Referenced by: '<S2>/Step1'
   */
  -4.0,

  /* Variable: DisEle_3
   * Referenced by: '<S2>/Step2'
   */
  2.0,

  /* Variable: DisRud_1
   * Referenced by: '<S3>/Step'
   */
  0.0,

  /* Variable: DisRud_2
   * Referenced by: '<S3>/Step1'
   */
  -0.0,

  /* Variable: DisRud_3
   * Referenced by: '<S3>/Step2'
   */
  0.0,

  /* Variable: thrust
   * Referenced by: '<Root>/thrust1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/State-Space'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S2>/Step'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S2>/Step'
   */
  0.0,

  /* Expression: 3
   * Referenced by: '<S2>/Step1'
   */
  3.0,

  /* Expression: 0
   * Referenced by: '<S2>/Step1'
   */
  0.0,

  /* Expression: 5
   * Referenced by: '<S2>/Step2'
   */
  5.0,

  /* Expression: 0
   * Referenced by: '<S2>/Step2'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S1>/Step'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S1>/Step'
   */
  0.0,

  /* Expression: 3
   * Referenced by: '<S1>/Step1'
   */
  3.0,

  /* Expression: 0
   * Referenced by: '<S1>/Step1'
   */
  0.0,

  /* Expression: 5
   * Referenced by: '<S1>/Step2'
   */
  5.0,

  /* Expression: 0
   * Referenced by: '<S1>/Step2'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S3>/Step'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S3>/Step'
   */
  0.0,

  /* Expression: 1.5
   * Referenced by: '<S3>/Step1'
   */
  1.5,

  /* Expression: 0
   * Referenced by: '<S3>/Step1'
   */
  0.0,

  /* Expression: 2
   * Referenced by: '<S3>/Step2'
   */
  2.0,

  /* Expression: 0
   * Referenced by: '<S3>/Step2'
   */
  0.0,

  /* Expression: [trim_thrust; trim_control(1); trim_control(2); trim_control(3)]
   * Referenced by: '<Root>/Constant1'
   */
  { 2120.6214478151878, -2.4606861722940634, 2.9128669308682263E-16,
    2.7966120657901467E-15 }
};