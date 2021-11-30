/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: hr_processing.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 29-Nov-2021 22:32:31
 */

#ifndef HR_PROCESSING_H
#define HR_PROCESSING_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern double hr_processing(const double ppg_data[200],
                            const double mic_data[200]);

extern void hr_processing_initialize(void);

extern void hr_processing_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for hr_processing.h
 *
 * [EOF]
 */
