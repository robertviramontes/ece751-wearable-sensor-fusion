/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: findpeaks.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 07-Dec-2021 17:22:13
 */

/* Include Files */
#include "findpeaks.h"
#include "eml_setop.h"
#include "find.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void getLeftBase(const float yTemp[200], const int iPeak_data[],
                        int iPeak_size, const int iFinite_data[],
                        int iFinite_size, const int iInflect_data[],
                        int iBase_data[], int *iBase_size, int iSaddle_data[],
                        int *iSaddle_size);

/* Function Definitions */
/*
 * Arguments    : const float yTemp[200]
 *                const int iPeak_data[]
 *                int iPeak_size
 *                const int iFinite_data[]
 *                int iFinite_size
 *                const int iInflect_data[]
 *                int iBase_data[]
 *                int *iBase_size
 *                int iSaddle_data[]
 *                int *iSaddle_size
 * Return Type  : void
 */
static void getLeftBase(const float yTemp[200], const int iPeak_data[],
                        int iPeak_size, const int iFinite_data[],
                        int iFinite_size, const int iInflect_data[],
                        int iBase_data[], int *iBase_size, int iSaddle_data[],
                        int *iSaddle_size)
{
  double peak_data[200];
  double valley_data[200];
  float p;
  float v;
  int iValley_data[200];
  int i;
  int isv;
  int iv;
  int j;
  int k;
  int n;
  n = (unsigned char)iPeak_size;
  *iBase_size = (unsigned char)iPeak_size;
  if (0 <= n - 1) {
    memset(&iBase_data[0], 0, n * sizeof(int));
  }
  *iSaddle_size = (unsigned char)iPeak_size;
  if (0 <= n - 1) {
    memset(&iSaddle_data[0], 0, n * sizeof(int));
  }
  n = (unsigned char)iFinite_size;
  if (0 <= n - 1) {
    memset(&peak_data[0], 0, n * sizeof(double));
  }
  if (0 <= n - 1) {
    memset(&valley_data[0], 0, n * sizeof(double));
  }
  if (0 <= n - 1) {
    memset(&iValley_data[0], 0, n * sizeof(int));
  }
  n = -1;
  i = 0;
  j = 0;
  k = 0;
  v = rtNaNF;
  iv = 1;
  while (k + 1 <= iPeak_size) {
    while (iInflect_data[i] != iFinite_data[j]) {
      v = yTemp[iInflect_data[i] - 1];
      iv = iInflect_data[i];
      if (rtIsNaNF(yTemp[iInflect_data[i] - 1])) {
        n = -1;
      } else {
        while ((n + 1 > 0) && (valley_data[n] > v)) {
          n--;
        }
      }
      i++;
    }
    p = yTemp[iInflect_data[i] - 1];
    while ((n + 1 > 0) && (peak_data[n] < p)) {
      if (valley_data[n] < v) {
        v = (float)valley_data[n];
        iv = iValley_data[n];
      }
      n--;
    }
    isv = iv;
    while ((n + 1 > 0) && (peak_data[n] <= p)) {
      if (valley_data[n] < v) {
        v = (float)valley_data[n];
        iv = iValley_data[n];
      }
      n--;
    }
    n++;
    peak_data[n] = yTemp[iInflect_data[i] - 1];
    valley_data[n] = v;
    iValley_data[n] = iv;
    if (iInflect_data[i] == iPeak_data[k]) {
      iBase_data[k] = iv;
      iSaddle_data[k] = isv;
      k++;
    }
    i++;
    j++;
  }
}

/*
 * Arguments    : const float y[200]
 *                const double x[200]
 *                const int iPk_data[]
 *                int iPk_size
 *                int idx_data[]
 *                int *idx_size
 * Return Type  : void
 */
void c_findPeaksSeparatedByMoreThanM(const float y[200], const double x[200],
                                     const int iPk_data[], int iPk_size,
                                     int idx_data[], int *idx_size)
{
  double locs_temp_data[400];
  double d;
  double d1;
  float f;
  int iwork_data[400];
  int sortIdx_data[400];
  int b_i;
  int i;
  int i2;
  int j;
  int k;
  int kEnd;
  int p;
  int pEnd;
  int q;
  int qEnd;
  int sortIdx_size_tmp;
  short b_tmp_data[400];
  bool idelete_data[400];
  bool tmp_data[400];
  if (iPk_size == 0) {
    *idx_size = 0;
  } else {
    sortIdx_size_tmp = (short)iPk_size;
    if (0 <= sortIdx_size_tmp - 1) {
      memset(&sortIdx_data[0], 0, sortIdx_size_tmp * sizeof(int));
    }
    i = iPk_size - 1;
    for (k = 1; k <= i; k += 2) {
      f = y[iPk_data[k - 1] - 1];
      if ((f >= y[iPk_data[k] - 1]) || rtIsNaNF(f)) {
        sortIdx_data[k - 1] = k;
        sortIdx_data[k] = k + 1;
      } else {
        sortIdx_data[k - 1] = k + 1;
        sortIdx_data[k] = k;
      }
    }
    if ((iPk_size & 1) != 0) {
      sortIdx_data[iPk_size - 1] = iPk_size;
    }
    b_i = 2;
    while (b_i < iPk_size) {
      i2 = b_i << 1;
      j = 1;
      for (pEnd = b_i + 1; pEnd < iPk_size + 1; pEnd = qEnd + b_i) {
        p = j - 1;
        q = pEnd;
        qEnd = j + i2;
        if (qEnd > iPk_size + 1) {
          qEnd = iPk_size + 1;
        }
        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          f = y[iPk_data[sortIdx_data[p] - 1] - 1];
          i = sortIdx_data[q - 1];
          if ((f >= y[iPk_data[i - 1] - 1]) || rtIsNaNF(f)) {
            iwork_data[k] = sortIdx_data[p];
            p++;
            if (p + 1 == pEnd) {
              while (q < qEnd) {
                k++;
                iwork_data[k] = sortIdx_data[q - 1];
                q++;
              }
            }
          } else {
            iwork_data[k] = i;
            q++;
            if (q == qEnd) {
              while (p + 1 < pEnd) {
                k++;
                iwork_data[k] = sortIdx_data[p];
                p++;
              }
            }
          }
          k++;
        }
        for (k = 0; k < kEnd; k++) {
          sortIdx_data[(j + k) - 1] = iwork_data[k];
        }
        j = qEnd;
      }
      b_i = i2;
    }
    for (i = 0; i < sortIdx_size_tmp; i++) {
      locs_temp_data[i] = x[iPk_data[sortIdx_data[i] - 1] - 1];
    }
    if (0 <= sortIdx_size_tmp - 1) {
      memset(&idelete_data[0], 0, sortIdx_size_tmp * sizeof(bool));
    }
    for (b_i = 0; b_i < sortIdx_size_tmp; b_i++) {
      if (!idelete_data[b_i]) {
        i = sortIdx_data[b_i];
        for (i2 = 0; i2 < sortIdx_size_tmp; i2++) {
          d = x[iPk_data[i - 1] - 1];
          d1 = locs_temp_data[i2];
          tmp_data[i2] = ((d1 >= d - 0.3) && (d1 <= d + 0.3));
        }
        for (i = 0; i < sortIdx_size_tmp; i++) {
          idelete_data[i] = (idelete_data[i] || tmp_data[i]);
        }
        idelete_data[b_i] = false;
      }
    }
    i2 = (short)iPk_size - 1;
    *idx_size = 0;
    j = 0;
    for (b_i = 0; b_i <= i2; b_i++) {
      if (!idelete_data[b_i]) {
        (*idx_size)++;
        b_tmp_data[j] = (short)(b_i + 1);
        j++;
      }
    }
    for (i = 0; i < *idx_size; i++) {
      idx_data[i] = sortIdx_data[b_tmp_data[i] - 1];
    }
    sort(idx_data, idx_size);
  }
}

/*
 * Arguments    : const float y[200]
 *                const double x[200]
 *                int iPk_data[]
 *                int *iPk_size
 *                const int iFin_data[]
 *                int iFin_size
 *                const int iInf_data[]
 *                int iInf_size
 *                const int iInflect_data[]
 *                int iInflect_size
 *                double bPk_data[]
 *                int *bPk_size
 *                double bxPk_data[]
 *                int bxPk_size[2]
 *                double byPk_data[]
 *                int byPk_size[2]
 *                double wxPk_data[]
 *                int wxPk_size[2]
 * Return Type  : void
 */
void findExtents(const float y[200], const double x[200], int iPk_data[],
                 int *iPk_size, const int iFin_data[], int iFin_size,
                 const int iInf_data[], int iInf_size,
                 const int iInflect_data[], int iInflect_size,
                 double bPk_data[], int *bPk_size, double bxPk_data[],
                 int bxPk_size[2], double byPk_data[], int byPk_size[2],
                 double wxPk_data[], int wxPk_size[2])
{
  double b_bxPk_data[800];
  double c_bPk_data[400];
  double w_data[400];
  double xc_tmp;
  float b_bPk_data[200];
  float base_data[200];
  float yFinite[200];
  float b_xc_tmp;
  float refHeight;
  float refHeight_tmp;
  int c_iPk_data[400];
  int ii_data[400];
  int b_iPk_data[200];
  int b_x_data[200];
  int iInfL_data[200];
  int iInfR_data[200];
  int iLeftBase_data[200];
  int iLeftSaddle_data[200];
  int iRightBase_data[200];
  int iRightSaddle_data[200];
  int idx_data[200];
  int x_data[200];
  int i;
  int iLeftBase_size;
  int iPk_tmp;
  int iRightSaddle_size;
  int md2;
  int nx;
  int w_size;
  int xtmp;
  bool tmp_data[400];
  bool yFinite_data[200];
  memcpy(&yFinite[0], &y[0], 200U * sizeof(float));
  for (xtmp = 0; xtmp < iInf_size; xtmp++) {
    yFinite[iInf_data[xtmp] - 1] = rtNaNF;
  }
  nx = *iPk_size;
  if (0 <= nx - 1) {
    memcpy(&b_iPk_data[0], &iPk_data[0], nx * sizeof(int));
  }
  getLeftBase(yFinite, iPk_data, *iPk_size, iFin_data, iFin_size, iInflect_data,
              iLeftBase_data, &iLeftBase_size, iLeftSaddle_data, &md2);
  nx = *iPk_size - 1;
  md2 = *iPk_size >> 1;
  for (i = 0; i < md2; i++) {
    xtmp = b_iPk_data[i];
    iPk_tmp = nx - i;
    b_iPk_data[i] = b_iPk_data[iPk_tmp];
    b_iPk_data[iPk_tmp] = xtmp;
  }
  if (0 <= iFin_size - 1) {
    memcpy(&x_data[0], &iFin_data[0], iFin_size * sizeof(int));
  }
  md2 = iFin_size >> 1;
  for (i = 0; i < md2; i++) {
    xtmp = x_data[i];
    nx = (iFin_size - i) - 1;
    x_data[i] = x_data[nx];
    x_data[nx] = xtmp;
  }
  if (0 <= iInflect_size - 1) {
    memcpy(&b_x_data[0], &iInflect_data[0], iInflect_size * sizeof(int));
  }
  md2 = iInflect_size >> 1;
  for (i = 0; i < md2; i++) {
    xtmp = b_x_data[i];
    nx = (iInflect_size - i) - 1;
    b_x_data[i] = b_x_data[nx];
    b_x_data[nx] = xtmp;
  }
  getLeftBase(yFinite, b_iPk_data, *iPk_size, x_data, iFin_size, b_x_data,
              iRightBase_data, &iPk_tmp, iRightSaddle_data, &iRightSaddle_size);
  md2 = iPk_tmp >> 1;
  for (i = 0; i < md2; i++) {
    xtmp = iRightBase_data[i];
    nx = (iPk_tmp - i) - 1;
    iRightBase_data[i] = iRightBase_data[nx];
    iRightBase_data[nx] = xtmp;
  }
  md2 = iRightSaddle_size >> 1;
  for (i = 0; i < md2; i++) {
    xtmp = iRightSaddle_data[i];
    nx = (iRightSaddle_size - i) - 1;
    iRightSaddle_data[i] = iRightSaddle_data[nx];
    iRightSaddle_data[nx] = xtmp;
  }
  if (iLeftBase_size <= iPk_tmp) {
    nx = (unsigned char)iLeftBase_size;
  } else {
    nx = (unsigned char)iPk_tmp;
  }
  for (iPk_tmp = 0; iPk_tmp < nx; iPk_tmp++) {
    b_bPk_data[iPk_tmp] = fmaxf(yFinite[iLeftBase_data[iPk_tmp] - 1],
                                yFinite[iRightBase_data[iPk_tmp] - 1]);
  }
  for (xtmp = 0; xtmp < *iPk_size; xtmp++) {
    yFinite_data[xtmp] =
        (yFinite[iPk_data[xtmp] - 1] - b_bPk_data[xtmp] >= 0.0F);
  }
  eml_find(yFinite_data, *iPk_size, ii_data, &nx);
  for (xtmp = 0; xtmp < *iPk_size; xtmp++) {
    yFinite_data[xtmp] =
        (yFinite[iPk_data[xtmp] - 1] - b_bPk_data[xtmp] >= 0.0F);
  }
  eml_find(yFinite_data, *iPk_size, c_iPk_data, &iPk_tmp);
  w_size = iPk_tmp;
  for (xtmp = 0; xtmp < iPk_tmp; xtmp++) {
    w_data[xtmp] = c_iPk_data[xtmp];
  }
  for (xtmp = 0; xtmp < *iPk_size; xtmp++) {
    yFinite_data[xtmp] =
        (yFinite[iPk_data[xtmp] - 1] - b_bPk_data[xtmp] >= 0.0F);
  }
  eml_find(yFinite_data, *iPk_size, c_iPk_data, &iPk_tmp);
  for (xtmp = 0; xtmp < w_size; xtmp++) {
    b_iPk_data[xtmp] = iPk_data[(int)w_data[xtmp] - 1];
  }
  for (xtmp = 0; xtmp < w_size; xtmp++) {
    base_data[xtmp] = b_bPk_data[(int)w_data[xtmp] - 1];
  }
  if (0 <= w_size - 1) {
    memcpy(&b_bPk_data[0], &base_data[0], w_size * sizeof(float));
  }
  for (xtmp = 0; xtmp < w_size; xtmp++) {
    iRightBase_data[xtmp] = iLeftSaddle_data[(int)w_data[xtmp] - 1];
  }
  md2 = w_size;
  if (0 <= w_size - 1) {
    memcpy(&iLeftSaddle_data[0], &iRightBase_data[0], w_size * sizeof(int));
  }
  for (xtmp = 0; xtmp < w_size; xtmp++) {
    iRightBase_data[xtmp] = iRightSaddle_data[(int)w_data[xtmp] - 1];
  }
  iRightSaddle_size = w_size;
  if (0 <= w_size - 1) {
    memcpy(&iRightSaddle_data[0], &iRightBase_data[0], w_size * sizeof(int));
  }
  for (xtmp = 0; xtmp < nx; xtmp++) {
    c_iPk_data[xtmp] = iPk_data[ii_data[xtmp] - 1];
  }
  *iPk_size = nx;
  if (0 <= nx - 1) {
    memcpy(&iPk_data[0], &c_iPk_data[0], nx * sizeof(int));
  }
  if (iPk_tmp == 0) {
    md2 = 0;
    iRightSaddle_size = 0;
  }
  nx = w_size << 1;
  if (0 <= nx - 1) {
    memset(&w_data[0], 0, nx * sizeof(double));
  }
  for (i = 0; i < w_size; i++) {
    refHeight_tmp = yFinite[b_iPk_data[i] - 1] + base_data[i];
    refHeight = refHeight_tmp / 2.0F;
    nx = iPk_data[i];
    while ((nx >= iLeftSaddle_data[i]) && (yFinite[nx - 1] > refHeight)) {
      nx--;
    }
    xtmp = iLeftSaddle_data[i];
    if (nx < xtmp) {
      w_data[i] = x[xtmp - 1];
    } else {
      xc_tmp = x[nx - 1];
      b_xc_tmp = yFinite[nx - 1];
      refHeight_tmp = (float)xc_tmp + (float)(x[nx] - xc_tmp) *
                                          (0.5F * refHeight_tmp - b_xc_tmp) /
                                          (yFinite[nx] - b_xc_tmp);
      if (rtIsNaNF(refHeight_tmp)) {
        if (rtIsInfF(base_data[i])) {
          refHeight_tmp = (float)(0.5 * (xc_tmp + x[nx]));
        } else {
          refHeight_tmp = (float)x[nx];
        }
      }
      w_data[i] = refHeight_tmp;
    }
    nx = iPk_data[i] - 1;
    while ((nx + 1 <= iRightSaddle_data[i]) && (yFinite[nx] > refHeight)) {
      nx++;
    }
    xtmp = iRightSaddle_data[i];
    if (nx + 1 > xtmp) {
      w_data[i + w_size] = x[xtmp - 1];
    } else {
      xc_tmp = x[nx - 1];
      refHeight_tmp = (float)x[nx] +
                      (float)(xc_tmp - x[nx]) *
                          (0.5F * (yFinite[b_iPk_data[i] - 1] + base_data[i]) -
                           yFinite[nx]) /
                          (yFinite[nx - 1] - yFinite[nx]);
      if (rtIsNaNF(refHeight_tmp)) {
        if (rtIsInfF(base_data[i])) {
          refHeight_tmp = (float)(0.5 * (x[nx] + xc_tmp));
        } else {
          refHeight_tmp = (float)xc_tmp;
        }
      }
      w_data[i + w_size] = refHeight_tmp;
    }
  }
  nx = *iPk_size - 1;
  if (0 <= nx) {
    memcpy(&idx_data[0], &iPk_data[0], (nx + 1) * sizeof(int));
  }
  do_vectors(idx_data, *iPk_size, iInf_data, iInf_size, iPk_data, iPk_size,
             iLeftBase_data, &iLeftBase_size, x_data, &nx);
  b_do_vectors(iPk_data, *iPk_size, b_iPk_data, w_size, b_x_data, &iPk_tmp,
               iLeftBase_data, &iLeftBase_size, x_data, &nx);
  if (0 <= iLeftBase_size - 1) {
    memcpy(&idx_data[0], &iLeftBase_data[0], iLeftBase_size * sizeof(int));
  }
  b_do_vectors(iPk_data, *iPk_size, iInf_data, iInf_size, b_x_data, &iPk_tmp,
               iLeftBase_data, &iLeftBase_size, x_data, &nx);
  if (0 <= iLeftBase_size - 1) {
    memcpy(&iRightBase_data[0], &iLeftBase_data[0],
           iLeftBase_size * sizeof(int));
  }
  nx = (short)*iPk_size;
  *bPk_size = (short)*iPk_size;
  if (0 <= nx - 1) {
    memset(&bPk_data[0], 0, nx * sizeof(double));
  }
  for (xtmp = 0; xtmp < w_size; xtmp++) {
    bPk_data[idx_data[xtmp] - 1] = b_bPk_data[xtmp];
  }
  for (xtmp = 0; xtmp < iLeftBase_size; xtmp++) {
    bPk_data[iLeftBase_data[xtmp] - 1] = 0.0;
  }
  for (xtmp = 0; xtmp < iInf_size; xtmp++) {
    iLeftBase_data[xtmp] = iInf_data[xtmp] - 1;
  }
  for (iPk_tmp = 0; iPk_tmp < iInf_size; iPk_tmp++) {
    xtmp = iLeftBase_data[iPk_tmp];
    if (1 < xtmp) {
      iInfL_data[iPk_tmp] = xtmp;
    } else {
      iInfL_data[iPk_tmp] = 1;
    }
  }
  for (xtmp = 0; xtmp < iInf_size; xtmp++) {
    iLeftBase_data[xtmp] = iInf_data[xtmp] + 1;
  }
  for (iPk_tmp = 0; iPk_tmp < iInf_size; iPk_tmp++) {
    xtmp = iLeftBase_data[iPk_tmp];
    if (xtmp > 200) {
      iInfR_data[iPk_tmp] = 200;
    } else {
      iInfR_data[iPk_tmp] = xtmp;
    }
  }
  bxPk_size[0] = *iPk_size;
  bxPk_size[1] = 2;
  nx = *iPk_size << 1;
  if (0 <= nx - 1) {
    memset(&bxPk_data[0], 0, nx * sizeof(double));
  }
  for (xtmp = 0; xtmp < md2; xtmp++) {
    bxPk_data[idx_data[xtmp] - 1] = x[iLeftSaddle_data[xtmp] - 1];
  }
  for (xtmp = 0; xtmp < iRightSaddle_size; xtmp++) {
    bxPk_data[(idx_data[xtmp] + bxPk_size[0]) - 1] =
        x[iRightSaddle_data[xtmp] - 1];
  }
  for (xtmp = 0; xtmp < iInf_size; xtmp++) {
    bxPk_data[iRightBase_data[xtmp] - 1] =
        0.5 * (x[iInf_data[xtmp] - 1] + x[iInfL_data[xtmp] - 1]);
  }
  for (xtmp = 0; xtmp < iInf_size; xtmp++) {
    bxPk_data[(iRightBase_data[xtmp] + bxPk_size[0]) - 1] =
        0.5 * (x[iInf_data[xtmp] - 1] + x[iInfR_data[xtmp] - 1]);
  }
  byPk_size[0] = *iPk_size;
  byPk_size[1] = 2;
  if (0 <= nx - 1) {
    memset(&byPk_data[0], 0, nx * sizeof(double));
  }
  for (xtmp = 0; xtmp < md2; xtmp++) {
    byPk_data[idx_data[xtmp] - 1] = y[iLeftSaddle_data[xtmp] - 1];
  }
  for (xtmp = 0; xtmp < iRightSaddle_size; xtmp++) {
    byPk_data[(idx_data[xtmp] + byPk_size[0]) - 1] =
        y[iRightSaddle_data[xtmp] - 1];
  }
  for (xtmp = 0; xtmp < iInf_size; xtmp++) {
    byPk_data[iRightBase_data[xtmp] - 1] = y[iInfL_data[xtmp] - 1];
  }
  for (xtmp = 0; xtmp < iInf_size; xtmp++) {
    byPk_data[(iRightBase_data[xtmp] + byPk_size[0]) - 1] =
        y[iInfR_data[xtmp] - 1];
  }
  wxPk_size[0] = *iPk_size;
  wxPk_size[1] = 2;
  if (0 <= nx - 1) {
    memset(&wxPk_data[0], 0, nx * sizeof(double));
  }
  for (xtmp = 0; xtmp < 2; xtmp++) {
    for (nx = 0; nx < w_size; nx++) {
      wxPk_data[(idx_data[nx] + wxPk_size[0] * xtmp) - 1] =
          w_data[nx + w_size * xtmp];
    }
  }
  for (xtmp = 0; xtmp < iInf_size; xtmp++) {
    wxPk_data[iRightBase_data[xtmp] - 1] =
        0.5 * (x[iInf_data[xtmp] - 1] + x[iInfL_data[xtmp] - 1]);
  }
  for (xtmp = 0; xtmp < iInf_size; xtmp++) {
    wxPk_data[(iRightBase_data[xtmp] + wxPk_size[0]) - 1] =
        0.5 * (x[iInf_data[xtmp] - 1] + x[iInfR_data[xtmp] - 1]);
  }
  if (*iPk_size != 0) {
    w_size = (short)wxPk_size[0];
    if ((short)wxPk_size[0] != 0) {
      nx = wxPk_size[0];
      for (iPk_tmp = 0; iPk_tmp < nx; iPk_tmp++) {
        w_data[iPk_tmp] = wxPk_data[iPk_tmp + nx] - wxPk_data[iPk_tmp];
      }
    }
    for (xtmp = 0; xtmp < w_size; xtmp++) {
      xc_tmp = w_data[xtmp];
      tmp_data[xtmp] = ((0.1 <= xc_tmp) && (xc_tmp <= rtInf));
    }
    eml_find(tmp_data, (short)wxPk_size[0], c_iPk_data, &iPk_tmp);
    for (xtmp = 0; xtmp < iPk_tmp; xtmp++) {
      w_data[xtmp] = c_iPk_data[xtmp];
    }
    for (xtmp = 0; xtmp < iPk_tmp; xtmp++) {
      c_iPk_data[xtmp] = iPk_data[(int)w_data[xtmp] - 1];
    }
    *iPk_size = iPk_tmp;
    if (0 <= iPk_tmp - 1) {
      memcpy(&iPk_data[0], &c_iPk_data[0], iPk_tmp * sizeof(int));
    }
    for (xtmp = 0; xtmp < iPk_tmp; xtmp++) {
      c_bPk_data[xtmp] = bPk_data[(int)w_data[xtmp] - 1];
    }
    *bPk_size = iPk_tmp;
    if (0 <= iPk_tmp - 1) {
      memcpy(&bPk_data[0], &c_bPk_data[0], iPk_tmp * sizeof(double));
    }
    for (xtmp = 0; xtmp < 2; xtmp++) {
      for (nx = 0; nx < iPk_tmp; nx++) {
        b_bxPk_data[nx + iPk_tmp * xtmp] =
            bxPk_data[((int)w_data[nx] + bxPk_size[0] * xtmp) - 1];
      }
    }
    bxPk_size[0] = iPk_tmp;
    bxPk_size[1] = 2;
    nx = iPk_tmp * 2;
    if (0 <= nx - 1) {
      memcpy(&bxPk_data[0], &b_bxPk_data[0], nx * sizeof(double));
    }
    for (xtmp = 0; xtmp < 2; xtmp++) {
      for (nx = 0; nx < iPk_tmp; nx++) {
        b_bxPk_data[nx + iPk_tmp * xtmp] =
            byPk_data[((int)w_data[nx] + byPk_size[0] * xtmp) - 1];
      }
    }
    byPk_size[0] = iPk_tmp;
    byPk_size[1] = 2;
    nx = iPk_tmp * 2;
    if (0 <= nx - 1) {
      memcpy(&byPk_data[0], &b_bxPk_data[0], nx * sizeof(double));
    }
    for (xtmp = 0; xtmp < 2; xtmp++) {
      for (nx = 0; nx < iPk_tmp; nx++) {
        b_bxPk_data[nx + iPk_tmp * xtmp] =
            wxPk_data[((int)w_data[nx] + wxPk_size[0] * xtmp) - 1];
      }
    }
    wxPk_size[0] = iPk_tmp;
    wxPk_size[1] = 2;
    nx = iPk_tmp * 2;
    if (0 <= nx - 1) {
      memcpy(&wxPk_data[0], &b_bxPk_data[0], nx * sizeof(double));
    }
  }
}

/*
 * File trailer for findpeaks.c
 *
 * [EOF]
 */
