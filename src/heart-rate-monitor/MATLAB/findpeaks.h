/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: findpeaks.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 07-Dec-2021 17:22:13
 */

#ifndef FINDPEAKS_H
#define FINDPEAKS_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_findPeaksSeparatedByMoreThanM(const float y[200], const double x[200],
                                     const int iPk_data[], int iPk_size,
                                     int idx_data[], int *idx_size);

void findExtents(const float y[200], const double x[200], int iPk_data[],
                 int *iPk_size, const int iFin_data[], int iFin_size,
                 const int iInf_data[], int iInf_size,
                 const int iInflect_data[], int iInflect_size,
                 double bPk_data[], int *bPk_size, double bxPk_data[],
                 int bxPk_size[2], double byPk_data[], int byPk_size[2],
                 double wxPk_data[], int wxPk_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for findpeaks.h
 *
 * [EOF]
 */
