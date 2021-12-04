/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: hr_processing.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 04-Dec-2021 17:22:10
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
extern unsigned short
hr_processing(const unsigned int ir_vals[250], double ir_length,
              const unsigned int red_vals[250], double red_length,
              const int sound_vals[1250], double sound_length, double Fs_ppg,
              double Fs_sound, unsigned short previous_bpm);

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
