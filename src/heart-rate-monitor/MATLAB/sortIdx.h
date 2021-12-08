/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sortIdx.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 07-Dec-2021 17:22:13
 */

#ifndef SORTIDX_H
#define SORTIDX_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void merge_block(int idx_data[], int x_data[], int offset, int n,
                 int preSortLevel, int iwork_data[], int xwork_data[]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for sortIdx.h
 *
 * [EOF]
 */
