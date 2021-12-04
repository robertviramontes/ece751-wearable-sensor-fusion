/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: hr_processing_emxutil.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 04-Dec-2021 17:22:10
 */

#ifndef HR_PROCESSING_EMXUTIL_H
#define HR_PROCESSING_EMXUTIL_H

/* Include Files */
#include "hr_processing_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);

extern void emxFree_real_T(emxArray_real_T **pEmxArray);

extern void emxInit_real_T(emxArray_real_T **pEmxArray);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for hr_processing_emxutil.h
 *
 * [EOF]
 */
