/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: hr_processing.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 06-Dec-2021 17:32:44
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
extern double hr_processing(const float ir_vals[150], double ir_length,
                            const float red_vals[150], double red_length,
                            const float sound_vals[750], double sound_length,
                            double Fs_ppg, double Fs_sound,
                            double previous_bpm);

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
