/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: hr_processing.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 04-Dec-2021 16:28:53
 */

#ifndef HR_PROCESSING_H
#define HR_PROCESSING_H

/* Include Files */
#include "hr_processing_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern double hr_processing(const double ir_vals_data[],
                            const int ir_vals_size[1], double ir_length,
                            const double red_vals_data[],
                            const int red_vals_size[1], double red_length,
                            const emxArray_real_T *sound_vals,
                            double sound_length, double Fs_ppg, double Fs_sound,
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
