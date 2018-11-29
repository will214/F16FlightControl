/*
 * SS_F16_Block.c
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

/* Block signals (default storage) */
B_SS_F16_Block_T SS_F16_Block_B;

/* Continuous states */
X_SS_F16_Block_T SS_F16_Block_X;

/* Block states (default storage) */
DW_SS_F16_Block_T SS_F16_Block_DW;

/* Real-time model */
RT_MODEL_SS_F16_Block_T SS_F16_Block_M_;
RT_MODEL_SS_F16_Block_T *const SS_F16_Block_M = &SS_F16_Block_M_;

/*
 * This function updates continuous states using the ODE5 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE5_A[6] = {
    1.0/5.0, 3.0/10.0, 4.0/5.0, 8.0/9.0, 1.0, 1.0
  };

  static const real_T rt_ODE5_B[6][6] = {
    { 1.0/5.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

    { 3.0/40.0, 9.0/40.0, 0.0, 0.0, 0.0, 0.0 },

    { 44.0/45.0, -56.0/15.0, 32.0/9.0, 0.0, 0.0, 0.0 },

    { 19372.0/6561.0, -25360.0/2187.0, 64448.0/6561.0, -212.0/729.0, 0.0, 0.0 },

    { 9017.0/3168.0, -355.0/33.0, 46732.0/5247.0, 49.0/176.0, -5103.0/18656.0,
      0.0 },

    { 35.0/384.0, 0.0, 500.0/1113.0, 125.0/192.0, -2187.0/6784.0, 11.0/84.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE5_IntgData *id = (ODE5_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T *f4 = id->f[4];
  real_T *f5 = id->f[5];
  real_T hB[6];
  int_T i;
  int_T nXc = 18;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  SS_F16_Block_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE5_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[0]);
  rtsiSetdX(si, f1);
  SS_F16_Block_output();
  SS_F16_Block_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE5_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[1]);
  rtsiSetdX(si, f2);
  SS_F16_Block_output();
  SS_F16_Block_derivatives();

  /* f(:,4) = feval(odefile, t + hA(3), y + f*hB(:,3), args(:)(*)); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE5_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[2]);
  rtsiSetdX(si, f3);
  SS_F16_Block_output();
  SS_F16_Block_derivatives();

  /* f(:,5) = feval(odefile, t + hA(4), y + f*hB(:,4), args(:)(*)); */
  for (i = 0; i <= 3; i++) {
    hB[i] = h * rt_ODE5_B[3][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2] +
                   f3[i]*hB[3]);
  }

  rtsiSetT(si, t + h*rt_ODE5_A[3]);
  rtsiSetdX(si, f4);
  SS_F16_Block_output();
  SS_F16_Block_derivatives();

  /* f(:,6) = feval(odefile, t + hA(5), y + f*hB(:,5), args(:)(*)); */
  for (i = 0; i <= 4; i++) {
    hB[i] = h * rt_ODE5_B[4][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2] +
                   f3[i]*hB[3] + f4[i]*hB[4]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f5);
  SS_F16_Block_output();
  SS_F16_Block_derivatives();

  /* tnew = t + hA(6);
     ynew = y + f*hB(:,6); */
  for (i = 0; i <= 5; i++) {
    hB[i] = h * rt_ODE5_B[5][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2] +
                   f3[i]*hB[3] + f4[i]*hB[4] + f5[i]*hB[5]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model output function */
void SS_F16_Block_output(void)
{
  int_T iy;
  int_T ci;
  real_T currentTime;
  real_T currentTime_0;
  real_T currentTime_1;
  real_T currentTime_tmp;
  if (rtmIsMajorTimeStep(SS_F16_Block_M)) {
    /* set solver stop time */
    if (!(SS_F16_Block_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&SS_F16_Block_M->solverInfo,
                            ((SS_F16_Block_M->Timing.clockTickH0 + 1) *
        SS_F16_Block_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&SS_F16_Block_M->solverInfo,
                            ((SS_F16_Block_M->Timing.clockTick0 + 1) *
        SS_F16_Block_M->Timing.stepSize0 + SS_F16_Block_M->Timing.clockTickH0 *
        SS_F16_Block_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(SS_F16_Block_M)) {
    SS_F16_Block_M->Timing.t[0] = rtsiGetT(&SS_F16_Block_M->solverInfo);
  }

  /* StateSpace: '<Root>/State-Space' */
  for (iy = 0; iy < 18; iy++) {
    SS_F16_Block_B.StateSpace[iy] = 0.0;
    for (ci = 0; ci < 18; ci++) {
      SS_F16_Block_B.StateSpace[iy] += SS_F16_Block_P.C[ci * 18 + iy] *
        SS_F16_Block_X.StateSpace_CSTATE[ci];
    }
  }

  /* End of StateSpace: '<Root>/State-Space' */
  if (rtmIsMajorTimeStep(SS_F16_Block_M)) {
  }

  /* Step: '<S2>/Step' incorporates:
   *  Step: '<S1>/Step'
   *  Step: '<S1>/Step1'
   *  Step: '<S1>/Step2'
   *  Step: '<S2>/Step1'
   *  Step: '<S2>/Step2'
   *  Step: '<S3>/Step'
   *  Step: '<S3>/Step1'
   *  Step: '<S3>/Step2'
   */
  currentTime_tmp = SS_F16_Block_M->Timing.t[0];

  /* SignalConversion: '<Root>/TmpSignal ConversionAtState-SpaceInport1' incorporates:
   *  Constant: '<Root>/thrust1'
   */
  SS_F16_Block_B.TmpSignalConversionAtStateSpace[0] = SS_F16_Block_P.thrust;

  /* Step: '<S2>/Step' */
  if (currentTime_tmp < SS_F16_Block_P.Step_Time) {
    currentTime = SS_F16_Block_P.Step_Y0;
  } else {
    currentTime = SS_F16_Block_P.DisEle_1;
  }

  /* Step: '<S2>/Step1' */
  if (currentTime_tmp < SS_F16_Block_P.Step1_Time) {
    currentTime_0 = SS_F16_Block_P.Step1_Y0;
  } else {
    currentTime_0 = SS_F16_Block_P.DisEle_2;
  }

  /* Step: '<S2>/Step2' */
  if (currentTime_tmp < SS_F16_Block_P.Step2_Time) {
    currentTime_1 = SS_F16_Block_P.Step2_Y0;
  } else {
    currentTime_1 = SS_F16_Block_P.DisEle_3;
  }

  /* SignalConversion: '<Root>/TmpSignal ConversionAtState-SpaceInport1' incorporates:
   *  Sum: '<S2>/Sum'
   */
  SS_F16_Block_B.TmpSignalConversionAtStateSpace[1] = (currentTime +
    currentTime_0) + currentTime_1;

  /* Step: '<S1>/Step' */
  if (currentTime_tmp < SS_F16_Block_P.Step_Time_p) {
    currentTime = SS_F16_Block_P.Step_Y0_g;
  } else {
    currentTime = SS_F16_Block_P.DisAil_1;
  }

  /* Step: '<S1>/Step1' */
  if (currentTime_tmp < SS_F16_Block_P.Step1_Time_h) {
    currentTime_0 = SS_F16_Block_P.Step1_Y0_m;
  } else {
    currentTime_0 = SS_F16_Block_P.DisAil_2;
  }

  /* Step: '<S1>/Step2' */
  if (currentTime_tmp < SS_F16_Block_P.Step2_Time_i) {
    currentTime_1 = SS_F16_Block_P.Step2_Y0_m;
  } else {
    currentTime_1 = SS_F16_Block_P.DisAil_3;
  }

  /* SignalConversion: '<Root>/TmpSignal ConversionAtState-SpaceInport1' incorporates:
   *  Sum: '<S1>/Sum'
   */
  SS_F16_Block_B.TmpSignalConversionAtStateSpace[2] = (currentTime +
    currentTime_0) + currentTime_1;

  /* Step: '<S3>/Step' */
  if (currentTime_tmp < SS_F16_Block_P.Step_Time_e) {
    currentTime = SS_F16_Block_P.Step_Y0_gd;
  } else {
    currentTime = SS_F16_Block_P.DisRud_1;
  }

  /* Step: '<S3>/Step1' */
  if (currentTime_tmp < SS_F16_Block_P.Step1_Time_e) {
    currentTime_0 = SS_F16_Block_P.Step1_Y0_h;
  } else {
    currentTime_0 = SS_F16_Block_P.DisRud_2;
  }

  /* Step: '<S3>/Step2' */
  if (currentTime_tmp < SS_F16_Block_P.Step2_Time_m) {
    currentTime_1 = SS_F16_Block_P.Step2_Y0_j;
  } else {
    currentTime_1 = SS_F16_Block_P.DisRud_3;
  }

  /* SignalConversion: '<Root>/TmpSignal ConversionAtState-SpaceInport1' incorporates:
   *  Sum: '<S3>/Sum'
   */
  SS_F16_Block_B.TmpSignalConversionAtStateSpace[3] = (currentTime +
    currentTime_0) + currentTime_1;

  /* Sum: '<Root>/Sum1' incorporates:
   *  Constant: '<Root>/Constant1'
   */
  SS_F16_Block_B.Sum1[0] = SS_F16_Block_B.TmpSignalConversionAtStateSpace[0] +
    SS_F16_Block_P.Constant1_Value[0];
  SS_F16_Block_B.Sum1[1] = SS_F16_Block_B.TmpSignalConversionAtStateSpace[1] +
    SS_F16_Block_P.Constant1_Value[1];
  SS_F16_Block_B.Sum1[2] = SS_F16_Block_B.TmpSignalConversionAtStateSpace[2] +
    SS_F16_Block_P.Constant1_Value[2];
  SS_F16_Block_B.Sum1[3] = SS_F16_Block_B.TmpSignalConversionAtStateSpace[3] +
    SS_F16_Block_P.Constant1_Value[3];
  if (rtmIsMajorTimeStep(SS_F16_Block_M)) {
    /* ToWorkspace: '<Root>/To Workspace1' */
    if (rtmIsMajorTimeStep(SS_F16_Block_M)) {
      rt_UpdateLogVar((LogVar *)(LogVar*)
                      (SS_F16_Block_DW.ToWorkspace1_PWORK.LoggedData),
                      &SS_F16_Block_B.Sum1[0], 0);
    }

    /* ToWorkspace: '<Root>/To Workspace2' */
    if (rtmIsMajorTimeStep(SS_F16_Block_M)) {
      rt_UpdateLogVar((LogVar *)(LogVar*)
                      (SS_F16_Block_DW.ToWorkspace2_PWORK.LoggedData),
                      &SS_F16_Block_B.StateSpace[0], 0);
    }
  }

  /* Clock: '<Root>/Clock' */
  SS_F16_Block_B.Clock = SS_F16_Block_M->Timing.t[0];
  if (rtmIsMajorTimeStep(SS_F16_Block_M)) {
    /* ToWorkspace: '<Root>/To Workspace' */
    if (rtmIsMajorTimeStep(SS_F16_Block_M)) {
      rt_UpdateLogVar((LogVar *)(LogVar*)
                      (SS_F16_Block_DW.ToWorkspace_PWORK.LoggedData),
                      &SS_F16_Block_B.Clock, 0);
    }
  }
}

/* Model update function */
void SS_F16_Block_update(void)
{
  if (rtmIsMajorTimeStep(SS_F16_Block_M)) {
    rt_ertODEUpdateContinuousStates(&SS_F16_Block_M->solverInfo);
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++SS_F16_Block_M->Timing.clockTick0)) {
    ++SS_F16_Block_M->Timing.clockTickH0;
  }

  SS_F16_Block_M->Timing.t[0] = rtsiGetSolverStopTime
    (&SS_F16_Block_M->solverInfo);

  {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++SS_F16_Block_M->Timing.clockTick1)) {
      ++SS_F16_Block_M->Timing.clockTickH1;
    }

    SS_F16_Block_M->Timing.t[1] = SS_F16_Block_M->Timing.clockTick1 *
      SS_F16_Block_M->Timing.stepSize1 + SS_F16_Block_M->Timing.clockTickH1 *
      SS_F16_Block_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Derivatives for root system: '<Root>' */
void SS_F16_Block_derivatives(void)
{
  int_T is;
  int_T ci;
  XDot_SS_F16_Block_T *_rtXdot;
  _rtXdot = ((XDot_SS_F16_Block_T *) SS_F16_Block_M->derivs);

  /* Derivatives for StateSpace: '<Root>/State-Space' */
  for (is = 0; is < 18; is++) {
    _rtXdot->StateSpace_CSTATE[is] = 0.0;
    for (ci = 0; ci < 18; ci++) {
      _rtXdot->StateSpace_CSTATE[is] += SS_F16_Block_P.A[ci * 18 + is] *
        SS_F16_Block_X.StateSpace_CSTATE[ci];
    }

    _rtXdot->StateSpace_CSTATE[is] += SS_F16_Block_P.B[is] *
      SS_F16_Block_B.TmpSignalConversionAtStateSpace[0];
    _rtXdot->StateSpace_CSTATE[is] += SS_F16_Block_P.B[18 + is] *
      SS_F16_Block_B.TmpSignalConversionAtStateSpace[1];
    _rtXdot->StateSpace_CSTATE[is] += SS_F16_Block_P.B[36 + is] *
      SS_F16_Block_B.TmpSignalConversionAtStateSpace[2];
    _rtXdot->StateSpace_CSTATE[is] += SS_F16_Block_P.B[54 + is] *
      SS_F16_Block_B.TmpSignalConversionAtStateSpace[3];
  }

  /* End of Derivatives for StateSpace: '<Root>/State-Space' */
}

/* Model initialize function */
void SS_F16_Block_initialize(void)
{
  /* Start for ToWorkspace: '<Root>/To Workspace1' */
  {
    int_T dimensions[1] = { 4 };

    SS_F16_Block_DW.ToWorkspace1_PWORK.LoggedData = rt_CreateLogVar(
      SS_F16_Block_M->rtwLogInfo,
      0.0,
      rtmGetTFinal(SS_F16_Block_M),
      SS_F16_Block_M->Timing.stepSize0,
      (&rtmGetErrorStatus(SS_F16_Block_M)),
      "controls",
      SS_DOUBLE,
      0,
      0,
      0,
      4,
      1,
      dimensions,
      NO_LOGVALDIMS,
      (NULL),
      (NULL),
      0,
      100,
      0.001,
      1);
    if (SS_F16_Block_DW.ToWorkspace1_PWORK.LoggedData == (NULL))
      return;
  }

  /* Start for ToWorkspace: '<Root>/To Workspace2' */
  {
    int_T dimensions[1] = { 18 };

    SS_F16_Block_DW.ToWorkspace2_PWORK.LoggedData = rt_CreateLogVar(
      SS_F16_Block_M->rtwLogInfo,
      0.0,
      rtmGetTFinal(SS_F16_Block_M),
      SS_F16_Block_M->Timing.stepSize0,
      (&rtmGetErrorStatus(SS_F16_Block_M)),
      "simout",
      SS_DOUBLE,
      0,
      0,
      0,
      18,
      1,
      dimensions,
      NO_LOGVALDIMS,
      (NULL),
      (NULL),
      0,
      100,
      0.001,
      1);
    if (SS_F16_Block_DW.ToWorkspace2_PWORK.LoggedData == (NULL))
      return;
  }

  /* Start for ToWorkspace: '<Root>/To Workspace' */
  {
    int_T dimensions[1] = { 1 };

    SS_F16_Block_DW.ToWorkspace_PWORK.LoggedData = rt_CreateLogVar(
      SS_F16_Block_M->rtwLogInfo,
      0.0,
      rtmGetTFinal(SS_F16_Block_M),
      SS_F16_Block_M->Timing.stepSize0,
      (&rtmGetErrorStatus(SS_F16_Block_M)),
      "T",
      SS_DOUBLE,
      0,
      0,
      0,
      1,
      1,
      dimensions,
      NO_LOGVALDIMS,
      (NULL),
      (NULL),
      0,
      100,
      0.001,
      1);
    if (SS_F16_Block_DW.ToWorkspace_PWORK.LoggedData == (NULL))
      return;
  }

  {
    int_T is;

    /* InitializeConditions for StateSpace: '<Root>/State-Space' */
    for (is = 0; is < 18; is++) {
      SS_F16_Block_X.StateSpace_CSTATE[is] =
        SS_F16_Block_P.StateSpace_InitialCondition;
    }

    /* End of InitializeConditions for StateSpace: '<Root>/State-Space' */
  }
}

/* Model terminate function */
void SS_F16_Block_terminate(void)
{
  /* (no terminate code required) */
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/

/* Solver interface called by GRT_Main */
#ifndef USE_GENERATED_SOLVER

void rt_ODECreateIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEDestroyIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEUpdateContinuousStates(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

#endif

void MdlOutputs(int_T tid)
{
  SS_F16_Block_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  SS_F16_Block_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  SS_F16_Block_initialize();
}

void MdlTerminate(void)
{
  SS_F16_Block_terminate();
}

/* Registration function */
RT_MODEL_SS_F16_Block_T *SS_F16_Block(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)SS_F16_Block_M, 0,
                sizeof(RT_MODEL_SS_F16_Block_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&SS_F16_Block_M->solverInfo,
                          &SS_F16_Block_M->Timing.simTimeStep);
    rtsiSetTPtr(&SS_F16_Block_M->solverInfo, &rtmGetTPtr(SS_F16_Block_M));
    rtsiSetStepSizePtr(&SS_F16_Block_M->solverInfo,
                       &SS_F16_Block_M->Timing.stepSize0);
    rtsiSetdXPtr(&SS_F16_Block_M->solverInfo, &SS_F16_Block_M->derivs);
    rtsiSetContStatesPtr(&SS_F16_Block_M->solverInfo, (real_T **)
                         &SS_F16_Block_M->contStates);
    rtsiSetNumContStatesPtr(&SS_F16_Block_M->solverInfo,
      &SS_F16_Block_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&SS_F16_Block_M->solverInfo,
      &SS_F16_Block_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&SS_F16_Block_M->solverInfo,
      &SS_F16_Block_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&SS_F16_Block_M->solverInfo,
      &SS_F16_Block_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&SS_F16_Block_M->solverInfo, (&rtmGetErrorStatus
      (SS_F16_Block_M)));
    rtsiSetRTModelPtr(&SS_F16_Block_M->solverInfo, SS_F16_Block_M);
  }

  rtsiSetSimTimeStep(&SS_F16_Block_M->solverInfo, MAJOR_TIME_STEP);
  SS_F16_Block_M->intgData.y = SS_F16_Block_M->odeY;
  SS_F16_Block_M->intgData.f[0] = SS_F16_Block_M->odeF[0];
  SS_F16_Block_M->intgData.f[1] = SS_F16_Block_M->odeF[1];
  SS_F16_Block_M->intgData.f[2] = SS_F16_Block_M->odeF[2];
  SS_F16_Block_M->intgData.f[3] = SS_F16_Block_M->odeF[3];
  SS_F16_Block_M->intgData.f[4] = SS_F16_Block_M->odeF[4];
  SS_F16_Block_M->intgData.f[5] = SS_F16_Block_M->odeF[5];
  SS_F16_Block_M->contStates = ((real_T *) &SS_F16_Block_X);
  rtsiSetSolverData(&SS_F16_Block_M->solverInfo, (void *)
                    &SS_F16_Block_M->intgData);
  rtsiSetSolverName(&SS_F16_Block_M->solverInfo,"ode5");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = SS_F16_Block_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    SS_F16_Block_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    SS_F16_Block_M->Timing.sampleTimes =
      (&SS_F16_Block_M->Timing.sampleTimesArray[0]);
    SS_F16_Block_M->Timing.offsetTimes =
      (&SS_F16_Block_M->Timing.offsetTimesArray[0]);

    /* task periods */
    SS_F16_Block_M->Timing.sampleTimes[0] = (0.0);
    SS_F16_Block_M->Timing.sampleTimes[1] = (0.001);

    /* task offsets */
    SS_F16_Block_M->Timing.offsetTimes[0] = (0.0);
    SS_F16_Block_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(SS_F16_Block_M, &SS_F16_Block_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = SS_F16_Block_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    SS_F16_Block_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(SS_F16_Block_M, 30.0);
  SS_F16_Block_M->Timing.stepSize0 = 0.001;
  SS_F16_Block_M->Timing.stepSize1 = 0.001;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    SS_F16_Block_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(SS_F16_Block_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(SS_F16_Block_M->rtwLogInfo, (NULL));
    rtliSetLogT(SS_F16_Block_M->rtwLogInfo, "tout");
    rtliSetLogX(SS_F16_Block_M->rtwLogInfo, "");
    rtliSetLogXFinal(SS_F16_Block_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(SS_F16_Block_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(SS_F16_Block_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(SS_F16_Block_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(SS_F16_Block_M->rtwLogInfo, 1);
    rtliSetLogY(SS_F16_Block_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(SS_F16_Block_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(SS_F16_Block_M->rtwLogInfo, (NULL));
  }

  SS_F16_Block_M->solverInfoPtr = (&SS_F16_Block_M->solverInfo);
  SS_F16_Block_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&SS_F16_Block_M->solverInfo, 0.001);
  rtsiSetSolverMode(&SS_F16_Block_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  SS_F16_Block_M->blockIO = ((void *) &SS_F16_Block_B);
  (void) memset(((void *) &SS_F16_Block_B), 0,
                sizeof(B_SS_F16_Block_T));

  /* parameters */
  SS_F16_Block_M->defaultParam = ((real_T *)&SS_F16_Block_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &SS_F16_Block_X;
    SS_F16_Block_M->contStates = (x);
    (void) memset((void *)&SS_F16_Block_X, 0,
                  sizeof(X_SS_F16_Block_T));
  }

  /* states (dwork) */
  SS_F16_Block_M->dwork = ((void *) &SS_F16_Block_DW);
  (void) memset((void *)&SS_F16_Block_DW, 0,
                sizeof(DW_SS_F16_Block_T));

  /* Initialize Sizes */
  SS_F16_Block_M->Sizes.numContStates = (18);/* Number of continuous states */
  SS_F16_Block_M->Sizes.numPeriodicContStates = (0);/* Number of periodic continuous states */
  SS_F16_Block_M->Sizes.numY = (0);    /* Number of model outputs */
  SS_F16_Block_M->Sizes.numU = (0);    /* Number of model inputs */
  SS_F16_Block_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  SS_F16_Block_M->Sizes.numSampTimes = (2);/* Number of sample times */
  SS_F16_Block_M->Sizes.numBlocks = (27);/* Number of blocks */
  SS_F16_Block_M->Sizes.numBlockIO = (4);/* Number of block outputs */
  SS_F16_Block_M->Sizes.numBlockPrms = (753);/* Sum of parameter "widths" */
  return SS_F16_Block_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
