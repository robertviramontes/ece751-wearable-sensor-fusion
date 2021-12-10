/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: wavelet_peaks.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 07-Dec-2021 17:22:13
 */

/* Include Files */
#include "wavelet_peaks.h"
#include "findpeaks.h"
#include "rt_nonfinite.h"
#include "mw_cmsis.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * ppg_x = linspace(0, vals_length*(1/Fs), vals_length);
 *
 * Arguments    : const float vals[200]
 *                unsigned int Fs
 * Return Type  : double
 */
double wavelet_peaks(const float vals[200], unsigned int Fs)
{
  static double bxPk_data[800];
  static const float b[8] = {-0.075765714F,  -0.0296355281F, 0.497618675F,
                             0.803738773F,   0.297857791F,   -0.0992195457F,
                             -0.0126039675F, 0.0322231F};
  static const float b_b[8] = {0.0322231F,     -0.0126039675F, -0.0992195457F,
                               0.297857791F,   0.803738773F,   0.497618675F,
                               -0.0296355281F, -0.075765714F};
  static const unsigned char uv[214] = {
      6U,   5U,   4U,   3U,   2U,   1U,   0U,   0U,   1U,   2U,   3U,   4U,
      5U,   6U,   7U,   8U,   9U,   10U,  11U,  12U,  13U,  14U,  15U,  16U,
      17U,  18U,  19U,  20U,  21U,  22U,  23U,  24U,  25U,  26U,  27U,  28U,
      29U,  30U,  31U,  32U,  33U,  34U,  35U,  36U,  37U,  38U,  39U,  40U,
      41U,  42U,  43U,  44U,  45U,  46U,  47U,  48U,  49U,  50U,  51U,  52U,
      53U,  54U,  55U,  56U,  57U,  58U,  59U,  60U,  61U,  62U,  63U,  64U,
      65U,  66U,  67U,  68U,  69U,  70U,  71U,  72U,  73U,  74U,  75U,  76U,
      77U,  78U,  79U,  80U,  81U,  82U,  83U,  84U,  85U,  86U,  87U,  88U,
      89U,  90U,  91U,  92U,  93U,  94U,  95U,  96U,  97U,  98U,  99U,  100U,
      101U, 102U, 103U, 104U, 105U, 106U, 107U, 108U, 109U, 110U, 111U, 112U,
      113U, 114U, 115U, 116U, 117U, 118U, 119U, 120U, 121U, 122U, 123U, 124U,
      125U, 126U, 127U, 128U, 129U, 130U, 131U, 132U, 133U, 134U, 135U, 136U,
      137U, 138U, 139U, 140U, 141U, 142U, 143U, 144U, 145U, 146U, 147U, 148U,
      149U, 150U, 151U, 152U, 153U, 154U, 155U, 156U, 157U, 158U, 159U, 160U,
      161U, 162U, 163U, 164U, 165U, 166U, 167U, 168U, 169U, 170U, 171U, 172U,
      173U, 174U, 175U, 176U, 177U, 178U, 179U, 180U, 181U, 182U, 183U, 184U,
      185U, 186U, 187U, 188U, 189U, 190U, 191U, 192U, 193U, 194U, 195U, 196U,
      197U, 198U, 199U, 199U, 198U, 197U, 196U, 195U, 194U, 193U};
  double byPk_data[800];
  double wxPk_data[800];
  double bPk_data[400];
  double x[200];
  double y1_data[199];
  double tmp2;
  double work_data;
  float b_y1[212];
  float z[207];
  float y[205];
  float fv[200];
  float xrec[200];
  float yk;
  float ykfirst;
  int b_iPk_data[400];
  int idx_data[400];
  int iFinite_data[200];
  int iInfinite_data[200];
  int iInflect_data[200];
  int bxPk_size[2];
  int byPk_size[2];
  int wxPk_size[2];
  int b_i;
  int dimSize;
  int i;
  int kfirst;
  int nInf;
  int nInflect;
  int nPk;
  unsigned char iPk_data[200];
  char dir;
  char previousdir;
  bool isinfyk;
  bool isinfykfirst;
  /*  ppg_sum = ir_vals + red_vals; */
  memset(&z[0], 0, 207U * sizeof(float));
  for (kfirst = 0; kfirst < 8; kfirst++) {
    for (i = 0; i < 207; i++) {
      z[i] += b[7 - kfirst] * vals[uv[kfirst + i]];
    }
  }
  memset(&y[0], 0, 205U * sizeof(float));
  for (i = 0; i < 103; i++) {
    y[((i + 1) << 1) - 2] = z[(i << 1) + 1];
  }
  memset(&b_y1[0], 0, 212U * sizeof(float));
  for (kfirst = 0; kfirst < 8; kfirst++) {
    for (i = 0; i < 205; i++) {
      dimSize = (i - kfirst) + 7;
      b_y1[dimSize] += b_b[7 - kfirst] * y[i];
    }
  }
  memset(&fv[0], 0, 200U * sizeof(float));
  mw_arm_add_f32(&b_y1[6], &fv[0], &xrec[0], 200U);
  nPk = -1;
  nInf = -1;
  nInflect = -1;
  dir = 'n';
  kfirst = 0;
  ykfirst = rtInfF;
  isinfykfirst = true;
  for (dimSize = 0; dimSize < 200; dimSize++) {
    x[dimSize] = (double)dimSize / (double)Fs;
    yk = xrec[dimSize];
    if (rtIsNaNF(yk)) {
      yk = rtInfF;
      isinfyk = true;
    } else if (rtIsInfF(yk) && (yk > 0.0F)) {
      isinfyk = true;
      nInf++;
      iInfinite_data[nInf] = dimSize + 1;
    } else {
      isinfyk = false;
    }
    if (yk != ykfirst) {
      previousdir = dir;
      if (isinfyk || isinfykfirst) {
        dir = 'n';
        if (kfirst >= 1) {
          nInflect++;
          iInflect_data[nInflect] = kfirst;
        }
      } else if (yk < ykfirst) {
        dir = 'd';
        if ('d' != previousdir) {
          nInflect++;
          iInflect_data[nInflect] = kfirst;
          if (previousdir == 'i') {
            nPk++;
            iFinite_data[nPk] = kfirst;
          }
        }
      } else {
        dir = 'i';
        if ('i' != previousdir) {
          nInflect++;
          iInflect_data[nInflect] = kfirst;
        }
      }
      ykfirst = yk;
      kfirst = dimSize + 1;
      isinfykfirst = isinfyk;
    }
  }
  if ((!isinfykfirst) &&
      ((nInflect + 1 == 0) || (iInflect_data[nInflect] < 200))) {
    nInflect++;
    iInflect_data[nInflect] = 200;
  }
  if (1 > nPk + 1) {
    b_i = -1;
  } else {
    b_i = nPk;
  }
  i = b_i + 1;
  kfirst = 0;
  for (dimSize = 0; dimSize < i; dimSize++) {
    b_i = iFinite_data[dimSize];
    if ((xrec[b_i - 1] > rtMinusInfF) &&
        (xrec[b_i - 1] - fmaxf(xrec[b_i - 2], xrec[b_i]) >= 0.0F)) {
      kfirst++;
      iPk_data[kfirst - 1] = (unsigned char)b_i;
    }
  }
  if (1 > kfirst) {
    kfirst = 0;
  }
  for (b_i = 0; b_i < kfirst; b_i++) {
    b_iPk_data[b_i] = iPk_data[b_i];
  }
  if (1 > nPk + 1) {
    nPk = -1;
  }
  if (1 > nInf + 1) {
    nInf = -1;
  }
  if (1 > nInflect + 1) {
    nInflect = -1;
  }
  findExtents(xrec, x, b_iPk_data, &kfirst, iFinite_data, nPk + 1,
              iInfinite_data, nInf + 1, iInflect_data, nInflect + 1, bPk_data,
              &i, bxPk_data, bxPk_size, byPk_data, byPk_size, wxPk_data,
              wxPk_size);
  c_findPeaksSeparatedByMoreThanM(xrec, x, b_iPk_data, kfirst, idx_data, &i);
  if (i > 200) {
    i = 200;
  }
  dimSize = i - 1;
  if (i == 0) {
    nPk = 0;
  } else {
    kfirst = i - 1;
    if (kfirst > 1) {
      kfirst = 1;
    }
    if (kfirst < 1) {
      nPk = 0;
    } else {
      nPk = (short)(i - 1);
      if ((short)(i - 1) != 0) {
        work_data = x[b_iPk_data[idx_data[0] - 1] - 1];
        for (kfirst = 2; kfirst <= dimSize + 1; kfirst++) {
          tmp2 = work_data;
          work_data = x[b_iPk_data[idx_data[kfirst - 1] - 1] - 1];
          y1_data[kfirst - 2] = work_data - tmp2;
        }
      }
    }
  }
  /*  difference in s */
  for (b_i = 0; b_i < nPk; b_i++) {
    y1_data[b_i] = 60.0 / y1_data[b_i];
  }
  /*  convert to bpm */
  if (nPk == 0) {
    work_data = 0.0;
  } else {
    work_data = y1_data[0];
    for (dimSize = 2; dimSize <= nPk; dimSize++) {
      work_data += y1_data[dimSize - 1];
    }
  }
  return work_data / (double)nPk;
}

/*
 * File trailer for wavelet_peaks.c
 *
 * [EOF]
 */
