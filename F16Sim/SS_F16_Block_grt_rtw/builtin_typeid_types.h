/*
 * builtin_typeid_types.h
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

#ifndef BUILTIN_TYPEID_TYPES_H
#define BUILTIN_TYPEID_TYPES_H
#include "rtwtypes.h"
#ifndef BUILTIN_TYPEID_TYPES
#define BUILTIN_TYPEID_TYPES

/* Enumeration of built-in data types */
typedef enum {
  SS_DOUBLE = 0,                       /* real_T    */
  SS_SINGLE = 1,                       /* real32_T  */
  SS_INT8 = 2,                         /* int8_T    */
  SS_UINT8 = 3,                        /* uint8_T   */
  SS_INT16 = 4,                        /* int16_T   */
  SS_UINT16 = 5,                       /* uint16_T  */
  SS_INT32 = 6,                        /* int32_T   */
  SS_UINT32 = 7,                       /* uint32_T  */
  SS_BOOLEAN = 8                       /* boolean_T */
} BuiltInDTypeId;

#define SS_NUM_BUILT_IN_DTYPE          ((int_T)SS_BOOLEAN+1)

/* Enumeration for MAT-file logging code */
typedef int_T DTypeId;

#endif                                 /* BUILTIN_TYPEID_TYPES */
#endif                                 /* BUILTIN_TYPEID_TYPES_H */
