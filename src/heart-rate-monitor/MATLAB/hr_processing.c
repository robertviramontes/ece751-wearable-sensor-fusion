/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: hr_processing.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 06-Dec-2021 17:32:44
 */

/* Include Files */
#include "hr_processing.h"
#include "hr_processing_emxutil.h"
#include "hr_processing_types.h"
#include "rt_nonfinite.h"
#include "MW_CMSIS_fft.h"
#include "mw_cmsis.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void do_vectors(const int a_data[], int a_size, const int b_data[],
                       int b_size, int c_data[], int *c_size, int ia_data[],
                       int *ia_size, int ib_data[], int *ib_size);

static void fft(const float x[750], creal32_T y[750]);

static void findpeaks(const float Yin[150], const emxArray_real_T *varargin_1,
                      float Ypk_data[], int *Ypk_size, double Xpk_data[],
                      int Xpk_size[2]);

static void merge(int idx_data[], int x_data[], int offset, int np, int nq,
                  int iwork_data[], int xwork_data[]);

static void merge_block(int idx_data[], int x_data[], int offset, int n,
                        int preSortLevel, int iwork_data[], int xwork_data[]);

static float rt_hypotf_snf(float u0, float u1);

static void sort(int x_data[], const int *x_size);

/* Function Definitions */
/*
 * Arguments    : const int a_data[]
 *                int a_size
 *                const int b_data[]
 *                int b_size
 *                int c_data[]
 *                int *c_size
 *                int ia_data[]
 *                int *ia_size
 *                int ib_data[]
 *                int *ib_size
 * Return Type  : void
 */
static void do_vectors(const int a_data[], int a_size, const int b_data[],
                       int b_size, int c_data[], int *c_size, int ia_data[],
                       int *ia_size, int ib_data[], int *ib_size)
{
  int ak;
  int b_ialast;
  int b_iblast;
  int bk;
  int iafirst;
  int ialast;
  int ibfirst;
  int iblast;
  int nc;
  int nia;
  int nib;
  *c_size = a_size + b_size;
  *ia_size = a_size;
  *ib_size = b_size;
  nc = -1;
  nia = -1;
  nib = 0;
  iafirst = 1;
  ialast = 0;
  ibfirst = 0;
  iblast = 0;
  while ((ialast + 1 <= a_size) && (iblast + 1 <= b_size)) {
    b_ialast = ialast + 1;
    ak = a_data[ialast];
    while ((b_ialast < a_size) && (a_data[b_ialast] == ak)) {
      b_ialast++;
    }
    ialast = b_ialast - 1;
    b_iblast = iblast + 1;
    bk = b_data[iblast];
    while ((b_iblast < b_size) && (b_data[b_iblast] == bk)) {
      b_iblast++;
    }
    iblast = b_iblast - 1;
    if (ak == bk) {
      nc++;
      c_data[nc] = ak;
      nia++;
      ia_data[nia] = iafirst;
      ialast = b_ialast;
      iafirst = b_ialast + 1;
      iblast = b_iblast;
      ibfirst = b_iblast;
    } else if (ak < bk) {
      nc++;
      nia++;
      c_data[nc] = ak;
      ia_data[nia] = iafirst;
      ialast = b_ialast;
      iafirst = b_ialast + 1;
    } else {
      nc++;
      nib++;
      c_data[nc] = bk;
      ib_data[nib - 1] = ibfirst + 1;
      iblast = b_iblast;
      ibfirst = b_iblast;
    }
  }
  while (ialast + 1 <= a_size) {
    b_ialast = ialast + 1;
    while ((b_ialast < a_size) && (a_data[b_ialast] == a_data[ialast])) {
      b_ialast++;
    }
    nc++;
    nia++;
    c_data[nc] = a_data[ialast];
    ia_data[nia] = iafirst;
    ialast = b_ialast;
    iafirst = b_ialast + 1;
  }
  while (iblast + 1 <= b_size) {
    b_iblast = iblast + 1;
    while ((b_iblast < b_size) && (b_data[b_iblast] == b_data[iblast])) {
      b_iblast++;
    }
    nc++;
    nib++;
    c_data[nc] = b_data[iblast];
    ib_data[nib - 1] = ibfirst + 1;
    iblast = b_iblast;
    ibfirst = b_iblast;
  }
  if (a_size > 0) {
    if (1 > nia + 1) {
      *ia_size = 0;
    } else {
      *ia_size = nia + 1;
    }
  }
  if (b_size > 0) {
    if (1 > nib) {
      *ib_size = 0;
    } else {
      *ib_size = nib;
    }
  }
  if (*c_size > 0) {
    if (1 > nc + 1) {
      *c_size = 0;
    } else {
      *c_size = nc + 1;
    }
  }
}

/*
 * Arguments    : const float x[750]
 *                creal32_T y[750]
 * Return Type  : void
 */
static void fft(const float x[750], creal32_T y[750])
{
  static creal32_T fy[2048];
  creal32_T fcv[2048];
  creal32_T fv[2048];
  creal32_T wwc[1499];
  float f;
  float f1;
  float nt_im;
  float nt_re;
  int b_y;
  int k;
  int rt;
  rt = 0;
  wwc[749].re = 1.0F;
  wwc[749].im = 0.0F;
  for (k = 0; k < 749; k++) {
    b_y = ((k + 1) << 1) - 1;
    if (1500 - rt <= b_y) {
      rt = (b_y + rt) - 1500;
    } else {
      rt += b_y;
    }
    nt_im = -3.14159274F * (float)rt / 750.0F;
    if (nt_im == 0.0F) {
      nt_re = 1.0F;
      nt_im = 0.0F;
    } else {
      nt_re = cosf(nt_im);
      nt_im = sinf(nt_im);
    }
    wwc[748 - k].re = nt_re;
    wwc[748 - k].im = -nt_im;
  }
  for (k = 748; k >= 0; k--) {
    wwc[k + 750] = wwc[748 - k];
  }
  for (k = 0; k < 750; k++) {
    f = x[k];
    nt_im = wwc[k + 749].re * f;
    nt_re = wwc[k + 749].im * -f;
    y[k].re = nt_im;
    y[k].im = nt_re;
    fy[k].re = nt_im;
    fy[k].im = nt_re;
  }
  memset(&fy[750], 0, 1298U * sizeof(creal32_T));
  mw_cmsis_fft_do_complex(&fy[0], 2048U, 0U);
  memcpy(&fv[0], &wwc[0], 1499U * sizeof(creal32_T));
  memset(&fv[1499], 0, 549U * sizeof(creal32_T));
  mw_cmsis_fft_do_complex(&fv[0], 2048U, 0U);
  mw_arm_cmplx_mult_cmplx_f32(&fy[0], &fv[0], &fcv[0], 2048U);
  memcpy(&fv[0], &fcv[0], 2048U * sizeof(creal32_T));
  mw_cmsis_fft_do_complex(&fv[0], 2048U, 1U);
  for (k = 0; k < 750; k++) {
    f = wwc[k + 749].re;
    nt_re = fv[k + 749].im;
    nt_im = wwc[k + 749].im;
    f1 = fv[k + 749].re;
    y[k].re = f * f1 + nt_im * nt_re;
    y[k].im = f * nt_re - nt_im * f1;
  }
}

/*
 * Arguments    : const float Yin[150]
 *                const emxArray_real_T *varargin_1
 *                float Ypk_data[]
 *                int *Ypk_size
 *                double Xpk_data[]
 *                int Xpk_size[2]
 * Return Type  : void
 */
static void findpeaks(const float Yin[150], const emxArray_real_T *varargin_1,
                      float Ypk_data[], int *Ypk_size, double Xpk_data[],
                      int Xpk_size[2])
{
  double locs_temp_data[300];
  const double *varargin_1_data;
  double d;
  double d1;
  float yk;
  float ykfirst;
  int c_data[300];
  int iwork_data[300];
  int sortIdx_data[300];
  int iFinite_data[150];
  int iInfinite_data[150];
  int iInflect_data[150];
  int iPk_data[150];
  int b_i;
  int b_nPk;
  int c_size;
  int i;
  int k;
  int kEnd;
  int kfirst;
  int nInf;
  int nPk;
  int q;
  int qEnd;
  int sortIdx_size_tmp;
  short b_tmp_data[300];
  char dir;
  char previousdir;
  bool idelete_data[300];
  bool tmp_data[300];
  bool isinfyk;
  bool isinfykfirst;
  varargin_1_data = varargin_1->data;
  nPk = -1;
  nInf = -1;
  dir = 'n';
  kfirst = -1;
  ykfirst = rtInfF;
  isinfykfirst = true;
  for (k = 0; k < 150; k++) {
    yk = Yin[k];
    if (rtIsNaNF(yk)) {
      yk = rtInfF;
      isinfyk = true;
    } else if (rtIsInfF(yk) && (yk > 0.0F)) {
      isinfyk = true;
      nInf++;
      iInfinite_data[nInf] = k + 1;
    } else {
      isinfyk = false;
    }
    if (yk != ykfirst) {
      previousdir = dir;
      if (isinfyk || isinfykfirst) {
        dir = 'n';
      } else if (yk < ykfirst) {
        dir = 'd';
        if (('d' != previousdir) && (previousdir == 'i')) {
          nPk++;
          iFinite_data[nPk] = kfirst + 1;
        }
      } else {
        dir = 'i';
      }
      ykfirst = yk;
      kfirst = k;
      isinfykfirst = isinfyk;
    }
  }
  b_nPk = 0;
  if (1 > nPk + 1) {
    nPk = -1;
  }
  kfirst = nPk + 1;
  for (k = 0; k < kfirst; k++) {
    i = iFinite_data[k];
    ykfirst = Yin[i - 1];
    if ((ykfirst > rtMinusInfF) &&
        (ykfirst - fmaxf(Yin[i - 2], Yin[i]) >= 0.0F)) {
      b_nPk++;
      iPk_data[b_nPk - 1] = i;
    }
  }
  if (1 > b_nPk) {
    b_nPk = 0;
  }
  if (1 > nInf + 1) {
    nInf = -1;
  }
  do_vectors(iPk_data, b_nPk, iInfinite_data, nInf + 1, c_data, &c_size,
             iInflect_data, &kfirst, iFinite_data, &nPk);
  if (c_size == 0) {
    *Ypk_size = 0;
  } else {
    sortIdx_size_tmp = (short)c_size;
    if (0 <= sortIdx_size_tmp - 1) {
      memset(&sortIdx_data[0], 0, sortIdx_size_tmp * sizeof(int));
    }
    i = c_size - 1;
    for (k = 1; k <= i; k += 2) {
      ykfirst = Yin[c_data[k - 1] - 1];
      if ((ykfirst >= Yin[c_data[k] - 1]) || rtIsNaNF(ykfirst)) {
        sortIdx_data[k - 1] = k;
        sortIdx_data[k] = k + 1;
      } else {
        sortIdx_data[k - 1] = k + 1;
        sortIdx_data[k] = k;
      }
    }
    if ((c_size & 1) != 0) {
      sortIdx_data[c_size - 1] = c_size;
    }
    b_i = 2;
    while (b_i < c_size) {
      kfirst = b_i << 1;
      nPk = 1;
      for (b_nPk = b_i + 1; b_nPk < c_size + 1; b_nPk = qEnd + b_i) {
        nInf = nPk - 1;
        q = b_nPk;
        qEnd = nPk + kfirst;
        if (qEnd > c_size + 1) {
          qEnd = c_size + 1;
        }
        k = 0;
        kEnd = qEnd - nPk;
        while (k + 1 <= kEnd) {
          ykfirst = Yin[c_data[sortIdx_data[nInf] - 1] - 1];
          i = sortIdx_data[q - 1];
          if ((ykfirst >= Yin[c_data[i - 1] - 1]) || rtIsNaNF(ykfirst)) {
            iwork_data[k] = sortIdx_data[nInf];
            nInf++;
            if (nInf + 1 == b_nPk) {
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
              while (nInf + 1 < b_nPk) {
                k++;
                iwork_data[k] = sortIdx_data[nInf];
                nInf++;
              }
            }
          }
          k++;
        }
        for (k = 0; k < kEnd; k++) {
          sortIdx_data[(nPk + k) - 1] = iwork_data[k];
        }
        nPk = qEnd;
      }
      b_i = kfirst;
    }
    for (i = 0; i < sortIdx_size_tmp; i++) {
      locs_temp_data[i] = varargin_1_data[c_data[sortIdx_data[i] - 1] - 1];
    }
    if (0 <= sortIdx_size_tmp - 1) {
      memset(&idelete_data[0], 0, sortIdx_size_tmp * sizeof(bool));
    }
    for (b_i = 0; b_i < sortIdx_size_tmp; b_i++) {
      if (!idelete_data[b_i]) {
        i = sortIdx_data[b_i];
        for (kfirst = 0; kfirst < sortIdx_size_tmp; kfirst++) {
          d = varargin_1_data[c_data[i - 1] - 1];
          d1 = locs_temp_data[kfirst];
          tmp_data[kfirst] = ((d1 >= d - 0.3) && (d1 <= d + 0.3));
        }
        for (i = 0; i < sortIdx_size_tmp; i++) {
          idelete_data[i] = (idelete_data[i] || tmp_data[i]);
        }
        idelete_data[b_i] = false;
      }
    }
    kfirst = (short)c_size - 1;
    *Ypk_size = 0;
    nPk = 0;
    for (b_i = 0; b_i <= kfirst; b_i++) {
      if (!idelete_data[b_i]) {
        (*Ypk_size)++;
        b_tmp_data[nPk] = (short)(b_i + 1);
        nPk++;
      }
    }
    for (i = 0; i < *Ypk_size; i++) {
      iwork_data[i] = sortIdx_data[b_tmp_data[i] - 1];
    }
    sort(iwork_data, Ypk_size);
  }
  if (*Ypk_size > 150) {
    *Ypk_size = 150;
  }
  for (i = 0; i < *Ypk_size; i++) {
    Ypk_data[i] = Yin[c_data[iwork_data[i] - 1] - 1];
  }
  Xpk_size[0] = 1;
  Xpk_size[1] = *Ypk_size;
  for (i = 0; i < *Ypk_size; i++) {
    Xpk_data[i] = varargin_1_data[c_data[iwork_data[i] - 1] - 1];
  }
}

/*
 * Arguments    : int idx_data[]
 *                int x_data[]
 *                int offset
 *                int np
 *                int nq
 *                int iwork_data[]
 *                int xwork_data[]
 * Return Type  : void
 */
static void merge(int idx_data[], int x_data[], int offset, int np, int nq,
                  int iwork_data[], int xwork_data[])
{
  int exitg1;
  int iout;
  int j;
  int n_tmp;
  int p;
  int q;
  if (nq != 0) {
    n_tmp = np + nq;
    for (j = 0; j < n_tmp; j++) {
      iout = offset + j;
      iwork_data[j] = idx_data[iout];
      xwork_data[j] = x_data[iout];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (j = p + 1; j <= np; j++) {
            iout = q + j;
            idx_data[iout] = iwork_data[j - 1];
            x_data[iout] = xwork_data[j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

/*
 * Arguments    : int idx_data[]
 *                int x_data[]
 *                int offset
 *                int n
 *                int preSortLevel
 *                int iwork_data[]
 *                int xwork_data[]
 * Return Type  : void
 */
static void merge_block(int idx_data[], int x_data[], int offset, int n,
                        int preSortLevel, int iwork_data[], int xwork_data[])
{
  int bLen;
  int nPairs;
  int nTail;
  int tailOffset;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    if ((nPairs & 1) != 0) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        merge(idx_data, x_data, offset + tailOffset, bLen, nTail - bLen,
              iwork_data, xwork_data);
      }
    }
    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 0; nTail < nPairs; nTail++) {
      merge(idx_data, x_data, offset + nTail * tailOffset, bLen, bLen,
            iwork_data, xwork_data);
    }
    bLen = tailOffset;
  }
  if (n > bLen) {
    merge(idx_data, x_data, offset, bLen, n - bLen, iwork_data, xwork_data);
  }
}

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_hypotf_snf(float u0, float u1)
{
  float a;
  float f;
  float y;
  a = fabsf(u0);
  y = fabsf(u1);
  if (a < y) {
    a /= y;
    mw_arm_sqrt_f32(a * a + 1.0F, &f);
    y *= f;
  } else if (a > y) {
    y /= a;
    mw_arm_sqrt_f32(y * y + 1.0F, &f);
    y = a * f;
  } else if (!rtIsNaNF(y)) {
    y = a * 1.41421354F;
  }
  return y;
}

/*
 * Arguments    : int x_data[]
 *                const int *x_size
 * Return Type  : void
 */
static void sort(int x_data[], const int *x_size)
{
  int b_iwork_data[300];
  int iidx_data[300];
  int iwork_data[300];
  int vwork_data[300];
  int xwork_data[300];
  int xwork[256];
  int x4[4];
  int b;
  int b_j;
  int dim;
  int exitg1;
  int i;
  int i1;
  int i2;
  int i4;
  int j;
  int k;
  int loop_ub_tmp;
  int nLeft;
  int nPairs;
  int nQuartets;
  int vlen;
  int vstride;
  int vwork_size;
  short iwork[256];
  short idx4[4];
  signed char perm[4];
  dim = 0;
  if (*x_size != 1) {
    dim = -1;
  }
  if (dim + 2 <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  vlen = vwork_size - 1;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  for (j = 0; j < vstride; j++) {
    for (k = 0; k <= vlen; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }
    loop_ub_tmp = (short)vwork_size;
    if (0 <= loop_ub_tmp - 1) {
      memset(&iidx_data[0], 0, loop_ub_tmp * sizeof(int));
    }
    if (vwork_size != 0) {
      x4[0] = 0;
      idx4[0] = 0;
      x4[1] = 0;
      idx4[1] = 0;
      x4[2] = 0;
      idx4[2] = 0;
      x4[3] = 0;
      idx4[3] = 0;
      if (0 <= loop_ub_tmp - 1) {
        memset(&iwork_data[0], 0, loop_ub_tmp * sizeof(int));
      }
      if (0 <= vwork_size - 1) {
        memset(&xwork_data[0], 0, vwork_size * sizeof(int));
      }
      nQuartets = vwork_size >> 2;
      for (b_j = 0; b_j < nQuartets; b_j++) {
        i = b_j << 2;
        idx4[0] = (short)(i + 1);
        idx4[1] = (short)(i + 2);
        idx4[2] = (short)(i + 3);
        idx4[3] = (short)(i + 4);
        nPairs = vwork_data[i];
        x4[0] = nPairs;
        dim = vwork_data[i + 1];
        x4[1] = dim;
        i4 = vwork_data[i + 2];
        x4[2] = i4;
        nLeft = vwork_data[i + 3];
        x4[3] = nLeft;
        if (nPairs <= dim) {
          i1 = 1;
          i2 = 2;
        } else {
          i1 = 2;
          i2 = 1;
        }
        if (i4 <= nLeft) {
          dim = 3;
          i4 = 4;
        } else {
          dim = 4;
          i4 = 3;
        }
        nPairs = x4[i1 - 1];
        nLeft = x4[dim - 1];
        if (nPairs <= nLeft) {
          nPairs = x4[i2 - 1];
          if (nPairs <= nLeft) {
            perm[0] = (signed char)i1;
            perm[1] = (signed char)i2;
            perm[2] = (signed char)dim;
            perm[3] = (signed char)i4;
          } else if (nPairs <= x4[i4 - 1]) {
            perm[0] = (signed char)i1;
            perm[1] = (signed char)dim;
            perm[2] = (signed char)i2;
            perm[3] = (signed char)i4;
          } else {
            perm[0] = (signed char)i1;
            perm[1] = (signed char)dim;
            perm[2] = (signed char)i4;
            perm[3] = (signed char)i2;
          }
        } else {
          nLeft = x4[i4 - 1];
          if (nPairs <= nLeft) {
            if (x4[i2 - 1] <= nLeft) {
              perm[0] = (signed char)dim;
              perm[1] = (signed char)i1;
              perm[2] = (signed char)i2;
              perm[3] = (signed char)i4;
            } else {
              perm[0] = (signed char)dim;
              perm[1] = (signed char)i1;
              perm[2] = (signed char)i4;
              perm[3] = (signed char)i2;
            }
          } else {
            perm[0] = (signed char)dim;
            perm[1] = (signed char)i4;
            perm[2] = (signed char)i1;
            perm[3] = (signed char)i2;
          }
        }
        iidx_data[i] = idx4[perm[0] - 1];
        iidx_data[i + 1] = idx4[perm[1] - 1];
        iidx_data[i + 2] = idx4[perm[2] - 1];
        iidx_data[i + 3] = idx4[perm[3] - 1];
        vwork_data[i] = x4[perm[0] - 1];
        vwork_data[i + 1] = x4[perm[1] - 1];
        vwork_data[i + 2] = x4[perm[2] - 1];
        vwork_data[i + 3] = x4[perm[3] - 1];
      }
      i4 = nQuartets << 2;
      nLeft = (vwork_size - i4) - 1;
      if (nLeft + 1 > 0) {
        for (k = 0; k <= nLeft; k++) {
          dim = i4 + k;
          idx4[k] = (short)(dim + 1);
          x4[k] = vwork_data[dim];
        }
        perm[1] = 0;
        perm[2] = 0;
        perm[3] = 0;
        if (nLeft + 1 == 1) {
          perm[0] = 1;
        } else if (nLeft + 1 == 2) {
          if (x4[0] <= x4[1]) {
            perm[0] = 1;
            perm[1] = 2;
          } else {
            perm[0] = 2;
            perm[1] = 1;
          }
        } else if (x4[0] <= x4[1]) {
          if (x4[1] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 2;
            perm[2] = 3;
          } else if (x4[0] <= x4[2]) {
            perm[0] = 1;
            perm[1] = 3;
            perm[2] = 2;
          } else {
            perm[0] = 3;
            perm[1] = 1;
            perm[2] = 2;
          }
        } else if (x4[0] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 1;
          perm[2] = 3;
        } else if (x4[1] <= x4[2]) {
          perm[0] = 2;
          perm[1] = 3;
          perm[2] = 1;
        } else {
          perm[0] = 3;
          perm[1] = 2;
          perm[2] = 1;
        }
        for (k = 0; k <= nLeft; k++) {
          i1 = perm[k] - 1;
          dim = i4 + k;
          iidx_data[dim] = idx4[i1];
          vwork_data[dim] = x4[i1];
        }
      }
      dim = 2;
      if (vwork_size > 1) {
        if (vwork_size >= 256) {
          for (b = 0; b < 6; b++) {
            nQuartets = 1 << (b + 2);
            i = nQuartets << 1;
            nPairs = 256 >> (b + 3);
            for (k = 0; k < nPairs; k++) {
              i4 = k * i;
              for (b_j = 0; b_j < i; b_j++) {
                dim = i4 + b_j;
                iwork[b_j] = (short)iidx_data[dim];
                xwork[b_j] = vwork_data[dim];
              }
              i2 = 0;
              nLeft = nQuartets;
              dim = i4 - 1;
              do {
                exitg1 = 0;
                dim++;
                if (xwork[i2] <= xwork[nLeft]) {
                  iidx_data[dim] = iwork[i2];
                  vwork_data[dim] = xwork[i2];
                  if (i2 + 1 < nQuartets) {
                    i2++;
                  } else {
                    exitg1 = 1;
                  }
                } else {
                  iidx_data[dim] = iwork[nLeft];
                  vwork_data[dim] = xwork[nLeft];
                  if (nLeft + 1 < i) {
                    nLeft++;
                  } else {
                    dim -= i2;
                    for (b_j = i2 + 1; b_j <= nQuartets; b_j++) {
                      i1 = dim + b_j;
                      iidx_data[i1] = iwork[b_j - 1];
                      vwork_data[i1] = xwork[b_j - 1];
                    }
                    exitg1 = 1;
                  }
                }
              } while (exitg1 == 0);
            }
          }
          if (vwork_size - 256 > 0) {
            merge_block(iidx_data, vwork_data, 256, vwork_size - 256, 2,
                        iwork_data, xwork_data);
          }
          dim = 8;
        }
        if (0 <= loop_ub_tmp - 1) {
          memcpy(&b_iwork_data[0], &iwork_data[0], loop_ub_tmp * sizeof(int));
        }
        if (0 <= vwork_size - 1) {
          memcpy(&iwork_data[0], &xwork_data[0], vwork_size * sizeof(int));
        }
        merge_block(iidx_data, vwork_data, 0, vwork_size, dim, b_iwork_data,
                    iwork_data);
      }
    }
    for (k = 0; k <= vlen; k++) {
      x_data[j + k * vstride] = vwork_data[k];
    }
  }
}

/*
 * ir_vals_no_dc = ir_vals - mean(ir_vals);
 *      red_vals_no_dc = red_vals - mean(red_vals);
 *      ppg = ir_vals_no_dc + red_vals_no_dc;
 *      %taking fft of all values together. But they should be taken in chunks
 *      L = ir_length; % can be red length as well
 *      Fs = Fs_ppg;
 *      Y = fft(ppg);
 *      P2 = abs(Y/L);
 *      P1 = P2(1:L/2+1);
 *      P1(2:end-1) = 2*P1(2:end-1);
 *      f = Fs*(0:(L/2))/L;
 *      % stem(f,P1)
 *      % figure
 *
 * Arguments    : const float ir_vals[150]
 *                double ir_length
 *                const float red_vals[150]
 *                double red_length
 *                const float sound_vals[750]
 *                double sound_length
 *                double Fs_ppg
 *                double Fs_sound
 *                double previous_bpm
 * Return Type  : double
 */
double hr_processing(const float ir_vals[150], double ir_length,
                     const float red_vals[150], double red_length,
                     const float sound_vals[750], double sound_length,
                     double Fs_ppg, double Fs_sound, double previous_bpm)
{
  emxArray_real_T *ppg_x;
  creal32_T Y2[750];
  double locs_data[150];
  double y1_data[149];
  double d;
  double delta1;
  double heartbeat;
  double tmp1;
  double y;
  double *ppg_x_data;
  float P12_data[750];
  float P22[750];
  float a__1_data[150];
  float fv[150];
  float ai;
  float im;
  float re;
  int locs_size[2];
  int dimSize;
  int i;
  int i1;
  int i2;
  int k;
  int ppg_x_tmp_tmp;
  int y1_size_idx_1;
  bool exitg1;
  (void)red_length;
  delta1 = ir_length * (1.0 / Fs_ppg);
  emxInit_real_T(&ppg_x);
  if (!(ir_length >= 0.0)) {
    ppg_x->size[0] = 1;
    ppg_x->size[1] = 0;
  } else {
    d = floor(ir_length);
    i = ppg_x->size[0] * ppg_x->size[1];
    ppg_x->size[0] = 1;
    ppg_x->size[1] = (int)d;
    emxEnsureCapacity_real_T(ppg_x, i);
    ppg_x_data = ppg_x->data;
    if ((int)d >= 1) {
      ppg_x_tmp_tmp = (int)d - 1;
      ppg_x_data[(int)floor(ir_length) - 1] = delta1;
      if (ppg_x->size[1] >= 2) {
        ppg_x_data[0] = 0.0;
        if (ppg_x->size[1] >= 3) {
          if ((0.0 == -delta1) && ((int)d > 2)) {
            delta1 /= (double)(int)d - 1.0;
            for (k = 2; k <= ppg_x_tmp_tmp; k++) {
              ppg_x_data[k - 1] = (double)(((k << 1) - (int)d) - 1) * delta1;
            }
            if (((int)d & 1) == 1) {
              ppg_x_data[(int)d >> 1] = 0.0;
            }
          } else if ((delta1 < 0.0) &&
                     (fabs(delta1) > 8.9884656743115785E+307)) {
            delta1 /= (double)ppg_x->size[1] - 1.0;
            i = ppg_x->size[1];
            for (k = 0; k <= i - 3; k++) {
              ppg_x_data[k + 1] = delta1 * ((double)k + 1.0);
            }
          } else {
            delta1 /= (double)ppg_x->size[1] - 1.0;
            i = ppg_x->size[1];
            for (k = 0; k <= i - 3; k++) {
              ppg_x_data[k + 1] = ((double)k + 1.0) * delta1;
            }
          }
        }
      }
    }
  }
  mw_arm_add_f32(&ir_vals[0], &red_vals[0], &fv[0], 150U);
  findpeaks(fv, ppg_x, a__1_data, &ppg_x_tmp_tmp, locs_data, locs_size);
  dimSize = locs_size[1];
  if (locs_size[1] == 0) {
    y1_size_idx_1 = 0;
  } else {
    ppg_x_tmp_tmp = locs_size[1] - 1;
    if (ppg_x_tmp_tmp > 1) {
      ppg_x_tmp_tmp = 1;
    }
    if (ppg_x_tmp_tmp < 1) {
      y1_size_idx_1 = 0;
    } else {
      y1_size_idx_1 = (unsigned char)(locs_size[1] - 1);
      if ((unsigned char)(locs_size[1] - 1) != 0) {
        delta1 = locs_data[0];
        for (ppg_x_tmp_tmp = 2; ppg_x_tmp_tmp <= dimSize; ppg_x_tmp_tmp++) {
          tmp1 = locs_data[ppg_x_tmp_tmp - 1];
          d = tmp1;
          tmp1 -= delta1;
          delta1 = d;
          y1_data[ppg_x_tmp_tmp - 2] = tmp1;
        }
      }
    }
  }
  /*  difference in s */
  ppg_x_tmp_tmp = y1_size_idx_1 - 1;
  for (i = 0; i <= ppg_x_tmp_tmp; i++) {
    y1_data[i] = 60.0 / y1_data[i];
  }
  /*  convert to bpm */
  if (y1_size_idx_1 == 0) {
    y = 0.0;
  } else {
    y = y1_data[0];
    for (k = 2; k <= y1_size_idx_1; k++) {
      y += y1_data[k - 1];
    }
  }
  fft(sound_vals, Y2);
  for (k = 0; k < 750; k++) {
    im = Y2[k].re;
    ai = Y2[k].im;
    if (ai == 0.0F) {
      re = im / (float)sound_length;
      im = 0.0F;
    } else if (im == 0.0F) {
      re = 0.0F;
      im = ai / (float)sound_length;
    } else {
      re = im / (float)sound_length;
      im = ai / (float)sound_length;
    }
    Y2[k].re = re;
    Y2[k].im = im;
    P22[k] = rt_hypotf_snf(re, im);
  }
  d = sound_length / 2.0 + 1.0;
  if (1.0 > d) {
    ppg_x_tmp_tmp = 0;
  } else {
    ppg_x_tmp_tmp = (int)d;
  }
  if (0 <= ppg_x_tmp_tmp - 1) {
    memcpy(&P12_data[0], &P22[0], ppg_x_tmp_tmp * sizeof(float));
  }
  if (2.0 > (double)ppg_x_tmp_tmp - 1.0) {
    i = 0;
    i1 = -1;
    i2 = 0;
  } else {
    i = 1;
    i1 = 0;
    i2 = ppg_x_tmp_tmp - 1;
  }
  ppg_x_tmp_tmp = (i2 - i1) - 1;
  for (i2 = 0; i2 < ppg_x_tmp_tmp; i2++) {
    P12_data[(i1 + i2) + 1] = 2.0F * P22[i + i2];
  }
  delta1 = sound_length / 2.0;
  if (rtIsNaN(delta1)) {
    i = ppg_x->size[0] * ppg_x->size[1];
    ppg_x->size[0] = 1;
    ppg_x->size[1] = 1;
    emxEnsureCapacity_real_T(ppg_x, i);
    ppg_x_data = ppg_x->data;
    ppg_x_data[0] = rtNaN;
  } else if (delta1 < 0.0) {
    ppg_x->size[1] = 0;
  } else if (rtIsInf(delta1) && (0.0 == delta1)) {
    i = ppg_x->size[0] * ppg_x->size[1];
    ppg_x->size[0] = 1;
    ppg_x->size[1] = 1;
    emxEnsureCapacity_real_T(ppg_x, i);
    ppg_x_data = ppg_x->data;
    ppg_x_data[0] = rtNaN;
  } else {
    i = ppg_x->size[0] * ppg_x->size[1];
    ppg_x->size[0] = 1;
    ppg_x_tmp_tmp = (int)floor(delta1);
    ppg_x->size[1] = ppg_x_tmp_tmp + 1;
    emxEnsureCapacity_real_T(ppg_x, i);
    ppg_x_data = ppg_x->data;
    for (i = 0; i <= ppg_x_tmp_tmp; i++) {
      ppg_x_data[i] = i;
    }
  }
  i = ppg_x->size[0] * ppg_x->size[1];
  ppg_x->size[0] = 1;
  emxEnsureCapacity_real_T(ppg_x, i);
  ppg_x_data = ppg_x->data;
  ppg_x_tmp_tmp = ppg_x->size[1] - 1;
  for (i = 0; i <= ppg_x_tmp_tmp; i++) {
    ppg_x_data[i] = Fs_sound * ppg_x_data[i] / sound_length;
  }
  /*  stem(f2,P12) */
  /*      a = 0; */
  /*      b = 0; */
  /*      %finding min and max frequency index (b/w 1 and 4) for ppg */
  /*      for k = 1 : length(f)  */
  /*          a = f(k);  */
  /*          if a >= 1 */
  /*               a = k; % min frequency index */
  /*               break */
  /*          end */
  /*      end */
  /*       */
  /*      for k = 1 : length(f) */
  /*          b = f(k); */
  /*          if b >= 4 */
  /*               b = k; % max frequency index */
  /*               break */
  /*          end */
  /*      end */
  /*   */
  /*      chunk_f = f(1, a:b); */
  /*      P1 = P1'; */
  /*      chunk_p1 = P1(1, a:b); */
  /*      %plot(chunk_f, chunk_p1) */
  /*      [~, idx_p1] = max(chunk_p1) */
  /*      hrtbeat_ppg = chunk_f(idx_p1) * 60; */
  /* finding min and max frequency index (b/w 1 and 4) for sound */
  tmp1 = 0.0;
  delta1 = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= ppg_x->size[1] - 1)) {
    tmp1 = ppg_x_data[k];
    if (ppg_x_data[k] >= 1.0) {
      tmp1 = (double)k + 1.0;
      exitg1 = true;
    } else {
      k++;
    }
  }
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= ppg_x->size[1] - 1)) {
    delta1 = ppg_x_data[k];
    if (ppg_x_data[k] >= 4.0) {
      delta1 = (double)k + 1.0;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (tmp1 > delta1) {
    i = 1;
    i1 = 0;
    i2 = -1;
  } else {
    i = (int)tmp1;
    i1 = (int)tmp1 - 1;
    i2 = (int)delta1 - 1;
  }
  /* plot(chunk_f2, chunk_p12) */
  ppg_x_tmp_tmp = (i2 - i1) + 1;
  if (ppg_x_tmp_tmp <= 2) {
    if (ppg_x_tmp_tmp == 1) {
      dimSize = 1;
    } else if ((P12_data[i1] < P12_data[i2]) ||
               (rtIsNaNF(P12_data[i1]) && (!rtIsNaNF(P12_data[i2])))) {
      dimSize = ppg_x_tmp_tmp;
    } else {
      dimSize = 1;
    }
  } else {
    if (!rtIsNaNF(P12_data[i1])) {
      dimSize = 1;
    } else {
      dimSize = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= ppg_x_tmp_tmp)) {
        if (!rtIsNaNF(P12_data[(i1 + k) - 1])) {
          dimSize = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (dimSize == 0) {
      dimSize = 1;
    } else {
      im = P12_data[(i1 + dimSize) - 1];
      i2 = dimSize + 1;
      for (k = i2; k <= ppg_x_tmp_tmp; k++) {
        re = P12_data[(i1 + k) - 1];
        if (im < re) {
          im = re;
          dimSize = k;
        }
      }
    }
  }
  /* the previous value would be multiplied by 0.3 when running in loop */
  heartbeat = (0.4 * (y / (double)y1_size_idx_1) +
               0.3 * (ppg_x_data[(i + dimSize) - 2] * 60.0)) +
              0.4 * previous_bpm;
  /*  complementary filter */
  emxFree_real_T(&ppg_x);
  return heartbeat;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void hr_processing_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void hr_processing_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for hr_processing.c
 *
 * [EOF]
 */
