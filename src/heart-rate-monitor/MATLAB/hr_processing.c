/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: hr_processing.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 04-Dec-2021 16:28:53
 */

/* Include Files */
#include "hr_processing.h"
#include "hr_processing_emxutil.h"
#include "hr_processing_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void binary_expand_op(emxArray_creal_T *Y2, const double ir_vals_data[],
                             const int *ir_vals_size,
                             const double red_vals_data[],
                             const int *red_vals_size);

static void c_FFTImplementationCallback_doH(const emxArray_real_T *x,
                                            emxArray_creal_T *y,
                                            int unsigned_nRows,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab);

static void c_FFTImplementationCallback_dob(const emxArray_real_T *x,
                                            int n2blue, int nfft,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab,
                                            const emxArray_real_T *sintabinv,
                                            emxArray_creal_T *y);

static void c_FFTImplementationCallback_gen(int nRows, bool useRadix2,
                                            emxArray_real_T *costab,
                                            emxArray_real_T *sintab,
                                            emxArray_real_T *sintabinv);

static void c_FFTImplementationCallback_r2b(const emxArray_creal_T *x,
                                            int unsigned_nRows,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab,
                                            emxArray_creal_T *y);

static void d_FFTImplementationCallback_doH(
    const emxArray_real_T *x, emxArray_creal_T *y, int nrowsx, int nRows,
    int nfft, const emxArray_creal_T *wwc, const emxArray_real_T *costab,
    const emxArray_real_T *sintab, const emxArray_real_T *costabinv,
    const emxArray_real_T *sintabinv);

static void d_FFTImplementationCallback_gen(int nRows, emxArray_real_T *costab,
                                            emxArray_real_T *sintab,
                                            emxArray_real_T *sintabinv);

static void d_FFTImplementationCallback_r2b(const emxArray_creal_T *x,
                                            int n1_unsigned,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab,
                                            emxArray_creal_T *y);

static void e_FFTImplementationCallback_r2b(const emxArray_creal_T *x,
                                            int n1_unsigned,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab,
                                            emxArray_creal_T *y);

static void fft(const emxArray_real_T *x, emxArray_creal_T *y);

static void maximum(const emxArray_real_T *x, double *ex, int *idx);

static double mean(const double x_data[], int x_size);

static double rt_hypotd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : emxArray_creal_T *Y2
 *                const double ir_vals_data[]
 *                const int *ir_vals_size
 *                const double red_vals_data[]
 *                const int *red_vals_size
 * Return Type  : void
 */
static void binary_expand_op(emxArray_creal_T *Y2, const double ir_vals_data[],
                             const int *ir_vals_size,
                             const double red_vals_data[],
                             const int *red_vals_size)
{
  emxArray_real_T b_ir_vals_data;
  double c_ir_vals_data[500];
  double d;
  double d1;
  int b_ir_vals_size;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  d = mean(ir_vals_data, *ir_vals_size);
  d1 = mean(red_vals_data, *red_vals_size);
  if (*red_vals_size == 1) {
    b_ir_vals_size = *ir_vals_size;
  } else {
    b_ir_vals_size = *red_vals_size;
  }
  stride_0_0 = (*ir_vals_size != 1);
  stride_1_0 = (*red_vals_size != 1);
  if (*red_vals_size == 1) {
    loop_ub = *ir_vals_size;
  } else {
    loop_ub = *red_vals_size;
  }
  for (i = 0; i < loop_ub; i++) {
    c_ir_vals_data[i] = (ir_vals_data[i * stride_0_0] - d) +
                        (red_vals_data[i * stride_1_0] - d1);
  }
  b_ir_vals_data.data = &c_ir_vals_data[0];
  b_ir_vals_data.size = &b_ir_vals_size;
  b_ir_vals_data.allocatedSize = 500;
  b_ir_vals_data.numDimensions = 1;
  b_ir_vals_data.canFreeData = false;
  fft(&b_ir_vals_data, Y2);
}

/*
 * Arguments    : const emxArray_real_T *x
 *                emxArray_creal_T *y
 *                int unsigned_nRows
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 * Return Type  : void
 */
static void c_FFTImplementationCallback_doH(const emxArray_real_T *x,
                                            emxArray_creal_T *y,
                                            int unsigned_nRows,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab)
{
  emxArray_creal_T *b_y;
  emxArray_creal_T *reconVar1;
  emxArray_creal_T *reconVar2;
  emxArray_real_T *hcostab;
  emxArray_real_T *hsintab;
  creal_T *b_y_data;
  creal_T *reconVar1_data;
  creal_T *reconVar2_data;
  creal_T *y_data;
  const double *costab_data;
  const double *sintab_data;
  const double *x_data;
  double b_y_re_tmp;
  double c_y_re_tmp;
  double d_y_re_tmp;
  double temp2_im;
  double temp2_im_tmp;
  double temp2_re;
  double temp_im;
  double temp_im_tmp;
  double temp_re;
  double temp_re_tmp;
  double y_re_tmp;
  double z_tmp;
  double *hcostab_data;
  double *hsintab_data;
  int bitrevIndex_data[5000];
  int wrapIndex_data[5000];
  int b_j1;
  int hszCostab;
  int i;
  int iDelta;
  int ihi;
  int istart;
  int ju;
  int k;
  int nRows;
  int nRowsD2;
  bool tst;
  sintab_data = sintab->data;
  costab_data = costab->data;
  y_data = y->data;
  x_data = x->data;
  emxInit_real_T(&hcostab, 2);
  emxInit_real_T(&hsintab, 2);
  nRows = unsigned_nRows / 2;
  istart = y->size[0];
  if (istart > nRows) {
    istart = nRows;
  }
  ihi = nRows - 2;
  nRowsD2 = nRows / 2;
  k = nRowsD2 / 2;
  hszCostab = costab->size[1] / 2;
  ju = hcostab->size[0] * hcostab->size[1];
  hcostab->size[1] = (short)hszCostab;
  hcostab->size[0] = 1;
  emxEnsureCapacity_real_T(hcostab, ju);
  hcostab_data = hcostab->data;
  ju = hsintab->size[0] * hsintab->size[1];
  hsintab->size[1] = (short)hszCostab;
  hsintab->size[0] = 1;
  emxEnsureCapacity_real_T(hsintab, ju);
  hsintab_data = hsintab->data;
  for (i = 0; i < hszCostab; i++) {
    iDelta = ((i + 1) << 1) - 2;
    hcostab_data[i] = costab_data[iDelta];
    hsintab_data[i] = sintab_data[iDelta];
  }
  emxInit_creal_T(&reconVar1);
  emxInit_creal_T(&reconVar2);
  ju = reconVar1->size[0];
  reconVar1->size[0] = nRows;
  emxEnsureCapacity_creal_T(reconVar1, ju);
  reconVar1_data = reconVar1->data;
  ju = reconVar2->size[0];
  reconVar2->size[0] = nRows;
  emxEnsureCapacity_creal_T(reconVar2, ju);
  reconVar2_data = reconVar2->data;
  for (i = 0; i < nRows; i++) {
    temp_re = sintab_data[i];
    temp2_re = costab_data[i];
    reconVar1_data[i].re = temp_re + 1.0;
    reconVar1_data[i].im = -temp2_re;
    reconVar2_data[i].re = 1.0 - temp_re;
    reconVar2_data[i].im = temp2_re;
    if (i + 1 != 1) {
      wrapIndex_data[i] = (nRows - i) + 1;
    } else {
      wrapIndex_data[0] = 1;
    }
  }
  z_tmp = (double)unsigned_nRows / 2.0;
  ju = 0;
  hszCostab = 1;
  iDelta = (int)z_tmp;
  if (0 <= iDelta - 1) {
    memset(&bitrevIndex_data[0], 0, iDelta * sizeof(int));
  }
  for (b_j1 = 0; b_j1 <= istart - 2; b_j1++) {
    bitrevIndex_data[b_j1] = hszCostab;
    iDelta = (int)z_tmp;
    tst = true;
    while (tst) {
      iDelta >>= 1;
      ju ^= iDelta;
      tst = ((ju & iDelta) == 0);
    }
    hszCostab = ju + 1;
  }
  bitrevIndex_data[istart - 1] = hszCostab;
  if ((x->size[0] & 1) == 0) {
    tst = true;
    istart = x->size[0];
  } else if (x->size[0] >= unsigned_nRows) {
    tst = true;
    istart = unsigned_nRows;
  } else {
    tst = false;
    istart = x->size[0] - 1;
  }
  if (istart <= unsigned_nRows) {
    iDelta = istart;
  } else {
    iDelta = unsigned_nRows;
  }
  temp_re = (double)iDelta / 2.0;
  if (istart > unsigned_nRows) {
    istart = unsigned_nRows;
  }
  ju = (int)((double)istart / 2.0);
  for (i = 0; i < ju; i++) {
    hszCostab = i << 1;
    iDelta = bitrevIndex_data[i];
    y_data[iDelta - 1].re = x_data[hszCostab];
    y_data[iDelta - 1].im = x_data[hszCostab + 1];
  }
  if (!tst) {
    ju = bitrevIndex_data[(int)temp_re] - 1;
    y_data[ju].re = x_data[(int)temp_re << 1];
    y_data[ju].im = 0.0;
  }
  emxInit_creal_T(&b_y);
  ju = b_y->size[0];
  b_y->size[0] = y->size[0];
  emxEnsureCapacity_creal_T(b_y, ju);
  b_y_data = b_y->data;
  iDelta = y->size[0];
  for (ju = 0; ju < iDelta; ju++) {
    b_y_data[ju] = y_data[ju];
  }
  if (nRows > 1) {
    for (i = 0; i <= ihi; i += 2) {
      temp_re_tmp = b_y_data[i + 1].re;
      temp_im_tmp = b_y_data[i + 1].im;
      temp_im = b_y_data[i].re;
      temp_re = b_y_data[i].im;
      b_y_data[i + 1].re = temp_im - temp_re_tmp;
      b_y_data[i + 1].im = temp_re - temp_im_tmp;
      b_y_data[i].re = temp_im + temp_re_tmp;
      b_y_data[i].im = temp_re + temp_im_tmp;
    }
  }
  iDelta = 2;
  hszCostab = 4;
  ju = ((k - 1) << 2) + 1;
  while (k > 0) {
    for (i = 0; i < ju; i += hszCostab) {
      b_j1 = i + iDelta;
      temp_re = b_y_data[b_j1].re;
      temp_im = b_y_data[b_j1].im;
      b_y_data[b_j1].re = b_y_data[i].re - temp_re;
      b_y_data[b_j1].im = b_y_data[i].im - temp_im;
      b_y_data[i].re += temp_re;
      b_y_data[i].im += temp_im;
    }
    istart = 1;
    for (nRows = k; nRows < nRowsD2; nRows += k) {
      temp2_re = hcostab_data[nRows];
      temp2_im = hsintab_data[nRows];
      i = istart;
      ihi = istart + ju;
      while (i < ihi) {
        b_j1 = i + iDelta;
        temp_re_tmp = b_y_data[b_j1].im;
        y_re_tmp = b_y_data[b_j1].re;
        temp_re = temp2_re * y_re_tmp - temp2_im * temp_re_tmp;
        temp_im = temp2_re * temp_re_tmp + temp2_im * y_re_tmp;
        b_y_data[b_j1].re = b_y_data[i].re - temp_re;
        b_y_data[b_j1].im = b_y_data[i].im - temp_im;
        b_y_data[i].re += temp_re;
        b_y_data[i].im += temp_im;
        i += hszCostab;
      }
      istart++;
    }
    k /= 2;
    iDelta = hszCostab;
    hszCostab += hszCostab;
    ju -= iDelta;
  }
  emxFree_real_T(&hsintab);
  emxFree_real_T(&hcostab);
  ju = y->size[0];
  y->size[0] = b_y->size[0];
  emxEnsureCapacity_creal_T(y, ju);
  y_data = y->data;
  iDelta = b_y->size[0];
  for (ju = 0; ju < iDelta; ju++) {
    y_data[ju] = b_y_data[ju];
  }
  iDelta = (int)z_tmp / 2;
  y_data[0].re = 0.5 * ((b_y_data[0].re * reconVar1_data[0].re -
                         b_y_data[0].im * reconVar1_data[0].im) +
                        (b_y_data[0].re * reconVar2_data[0].re -
                         -b_y_data[0].im * reconVar2_data[0].im));
  y_data[0].im = 0.5 * ((b_y_data[0].re * reconVar1_data[0].im +
                         b_y_data[0].im * reconVar1_data[0].re) +
                        (b_y_data[0].re * reconVar2_data[0].im +
                         -b_y_data[0].im * reconVar2_data[0].re));
  y_data[(int)z_tmp].re = 0.5 * ((b_y_data[0].re * reconVar2_data[0].re -
                                  b_y_data[0].im * reconVar2_data[0].im) +
                                 (b_y_data[0].re * reconVar1_data[0].re -
                                  -b_y_data[0].im * reconVar1_data[0].im));
  y_data[(int)z_tmp].im = 0.5 * ((b_y_data[0].re * reconVar2_data[0].im +
                                  b_y_data[0].im * reconVar2_data[0].re) +
                                 (b_y_data[0].re * reconVar1_data[0].im +
                                  -b_y_data[0].im * reconVar1_data[0].re));
  emxFree_creal_T(&b_y);
  for (i = 2; i <= iDelta; i++) {
    temp_re_tmp = y_data[i - 1].re;
    temp_im_tmp = y_data[i - 1].im;
    ju = wrapIndex_data[i - 1];
    temp2_im = y_data[ju - 1].re;
    temp2_im_tmp = y_data[ju - 1].im;
    y_re_tmp = reconVar1_data[i - 1].im;
    b_y_re_tmp = reconVar1_data[i - 1].re;
    c_y_re_tmp = reconVar2_data[i - 1].im;
    d_y_re_tmp = reconVar2_data[i - 1].re;
    y_data[i - 1].re =
        0.5 * ((temp_re_tmp * b_y_re_tmp - temp_im_tmp * y_re_tmp) +
               (temp2_im * d_y_re_tmp - -temp2_im_tmp * c_y_re_tmp));
    y_data[i - 1].im =
        0.5 * ((temp_re_tmp * y_re_tmp + temp_im_tmp * b_y_re_tmp) +
               (temp2_im * c_y_re_tmp + -temp2_im_tmp * d_y_re_tmp));
    hszCostab = ((int)z_tmp + i) - 1;
    y_data[hszCostab].re =
        0.5 * ((temp_re_tmp * d_y_re_tmp - temp_im_tmp * c_y_re_tmp) +
               (temp2_im * b_y_re_tmp - -temp2_im_tmp * y_re_tmp));
    y_data[hszCostab].im =
        0.5 * ((temp_re_tmp * c_y_re_tmp + temp_im_tmp * d_y_re_tmp) +
               (temp2_im * y_re_tmp + -temp2_im_tmp * b_y_re_tmp));
    temp_im = reconVar1_data[ju - 1].im;
    temp_re = reconVar1_data[ju - 1].re;
    y_re_tmp = reconVar2_data[ju - 1].im;
    temp2_re = reconVar2_data[ju - 1].re;
    y_data[ju - 1].re =
        0.5 * ((temp2_im * temp_re - temp2_im_tmp * temp_im) +
               (temp_re_tmp * temp2_re - -temp_im_tmp * y_re_tmp));
    y_data[ju - 1].im =
        0.5 * ((temp2_im * temp_im + temp2_im_tmp * temp_re) +
               (temp_re_tmp * y_re_tmp + -temp_im_tmp * temp2_re));
    ju = (ju + (int)z_tmp) - 1;
    y_data[ju].re = 0.5 * ((temp2_im * temp2_re - temp2_im_tmp * y_re_tmp) +
                           (temp_re_tmp * temp_re - -temp_im_tmp * temp_im));
    y_data[ju].im = 0.5 * ((temp2_im * y_re_tmp + temp2_im_tmp * temp2_re) +
                           (temp_re_tmp * temp_im + -temp_im_tmp * temp_re));
  }
  if (iDelta != 0) {
    temp_re_tmp = y_data[iDelta].re;
    temp_im_tmp = y_data[iDelta].im;
    y_re_tmp = reconVar1_data[iDelta].im;
    b_y_re_tmp = reconVar1_data[iDelta].re;
    c_y_re_tmp = reconVar2_data[iDelta].im;
    d_y_re_tmp = reconVar2_data[iDelta].re;
    temp_re = temp_re_tmp * d_y_re_tmp;
    temp2_re = temp_re_tmp * b_y_re_tmp;
    y_data[iDelta].re = 0.5 * ((temp2_re - temp_im_tmp * y_re_tmp) +
                               (temp_re - -temp_im_tmp * c_y_re_tmp));
    temp2_im = temp_re_tmp * c_y_re_tmp;
    temp_im = temp_re_tmp * y_re_tmp;
    y_data[iDelta].im = 0.5 * ((temp_im + temp_im_tmp * b_y_re_tmp) +
                               (temp2_im + -temp_im_tmp * d_y_re_tmp));
    ju = (int)z_tmp + iDelta;
    y_data[ju].re = 0.5 * ((temp_re - temp_im_tmp * c_y_re_tmp) +
                           (temp2_re - -temp_im_tmp * y_re_tmp));
    y_data[ju].im = 0.5 * ((temp2_im + temp_im_tmp * d_y_re_tmp) +
                           (temp_im + -temp_im_tmp * b_y_re_tmp));
  }
  emxFree_creal_T(&reconVar2);
  emxFree_creal_T(&reconVar1);
}

/*
 * Arguments    : const emxArray_real_T *x
 *                int n2blue
 *                int nfft
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                const emxArray_real_T *sintabinv
 *                emxArray_creal_T *y
 * Return Type  : void
 */
static void c_FFTImplementationCallback_dob(const emxArray_real_T *x,
                                            int n2blue, int nfft,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab,
                                            const emxArray_real_T *sintabinv,
                                            emxArray_creal_T *y)
{
  emxArray_creal_T *b_fv;
  emxArray_creal_T *fv;
  emxArray_creal_T *wwc;
  creal_T *b_fv_data;
  creal_T *fv_data;
  creal_T *wwc_data;
  creal_T *y_data;
  const double *costab_data;
  const double *sintab_data;
  const double *x_data;
  double b_nt_re_tmp;
  double nt_im;
  double nt_re;
  double twid_im;
  double twid_re;
  int i;
  int ihi;
  int j;
  int k;
  int minNrowsNx;
  int nInt2;
  int nInt2m1;
  int nRowsD2;
  int nt_re_tmp;
  int rt;
  bool tst;
  sintab_data = sintab->data;
  costab_data = costab->data;
  x_data = x->data;
  emxInit_creal_T(&wwc);
  if ((nfft != 1) && ((nfft & 1) == 0)) {
    j = nfft / 2;
    nInt2m1 = (j + j) - 1;
    ihi = wwc->size[0];
    wwc->size[0] = nInt2m1;
    emxEnsureCapacity_creal_T(wwc, ihi);
    wwc_data = wwc->data;
    rt = 0;
    wwc_data[j - 1].re = 1.0;
    wwc_data[j - 1].im = 0.0;
    nInt2 = j << 1;
    for (k = 0; k <= j - 2; k++) {
      minNrowsNx = ((k + 1) << 1) - 1;
      if (nInt2 - rt <= minNrowsNx) {
        rt += minNrowsNx - nInt2;
      } else {
        rt += minNrowsNx;
      }
      nt_im = -3.1415926535897931 * (double)rt / (double)j;
      if (nt_im == 0.0) {
        nt_re = 1.0;
        nt_im = 0.0;
      } else {
        nt_re = cos(nt_im);
        nt_im = sin(nt_im);
      }
      ihi = (j - k) - 2;
      wwc_data[ihi].re = nt_re;
      wwc_data[ihi].im = -nt_im;
    }
    ihi = nInt2m1 - 1;
    for (k = ihi; k >= j; k--) {
      wwc_data[k] = wwc_data[(nInt2m1 - k) - 1];
    }
  } else {
    nInt2m1 = (nfft + nfft) - 1;
    ihi = wwc->size[0];
    wwc->size[0] = nInt2m1;
    emxEnsureCapacity_creal_T(wwc, ihi);
    wwc_data = wwc->data;
    rt = 0;
    wwc_data[nfft - 1].re = 1.0;
    wwc_data[nfft - 1].im = 0.0;
    nInt2 = nfft << 1;
    for (k = 0; k <= nfft - 2; k++) {
      minNrowsNx = ((k + 1) << 1) - 1;
      if (nInt2 - rt <= minNrowsNx) {
        rt += minNrowsNx - nInt2;
      } else {
        rt += minNrowsNx;
      }
      nt_im = -3.1415926535897931 * (double)rt / (double)nfft;
      if (nt_im == 0.0) {
        nt_re = 1.0;
        nt_im = 0.0;
      } else {
        nt_re = cos(nt_im);
        nt_im = sin(nt_im);
      }
      ihi = (nfft - k) - 2;
      wwc_data[ihi].re = nt_re;
      wwc_data[ihi].im = -nt_im;
    }
    ihi = nInt2m1 - 1;
    for (k = ihi; k >= nfft; k--) {
      wwc_data[k] = wwc_data[(nInt2m1 - k) - 1];
    }
  }
  ihi = y->size[0];
  y->size[0] = nfft;
  emxEnsureCapacity_creal_T(y, ihi);
  y_data = y->data;
  if (nfft > x->size[0]) {
    ihi = y->size[0];
    y->size[0] = nfft;
    emxEnsureCapacity_creal_T(y, ihi);
    y_data = y->data;
    for (ihi = 0; ihi < nfft; ihi++) {
      y_data[ihi].re = 0.0;
      y_data[ihi].im = 0.0;
    }
  }
  emxInit_creal_T(&fv);
  emxInit_creal_T(&b_fv);
  if ((n2blue != 1) && ((nfft & 1) == 0)) {
    d_FFTImplementationCallback_doH(x, y, x->size[0], nfft, n2blue, wwc, costab,
                                    sintab, costab, sintabinv);
  } else {
    minNrowsNx = x->size[0];
    if (nfft <= minNrowsNx) {
      minNrowsNx = nfft;
    }
    for (k = 0; k < minNrowsNx; k++) {
      nt_re_tmp = (nfft + k) - 1;
      y_data[k].re = wwc_data[nt_re_tmp].re * x_data[k];
      y_data[k].im = wwc_data[nt_re_tmp].im * -x_data[k];
    }
    ihi = minNrowsNx + 1;
    for (k = ihi; k <= nfft; k++) {
      y_data[k - 1].re = 0.0;
      y_data[k - 1].im = 0.0;
    }
    ihi = fv->size[0];
    fv->size[0] = n2blue;
    emxEnsureCapacity_creal_T(fv, ihi);
    fv_data = fv->data;
    if (n2blue > y->size[0]) {
      ihi = fv->size[0];
      fv->size[0] = n2blue;
      emxEnsureCapacity_creal_T(fv, ihi);
      fv_data = fv->data;
      for (ihi = 0; ihi < n2blue; ihi++) {
        fv_data[ihi].re = 0.0;
        fv_data[ihi].im = 0.0;
      }
    }
    nInt2m1 = y->size[0];
    if (nInt2m1 > n2blue) {
      nInt2m1 = n2blue;
    }
    rt = n2blue - 2;
    nRowsD2 = n2blue / 2;
    k = nRowsD2 / 2;
    minNrowsNx = 0;
    nInt2 = 0;
    for (i = 0; i <= nInt2m1 - 2; i++) {
      fv_data[minNrowsNx] = y_data[i];
      minNrowsNx = n2blue;
      tst = true;
      while (tst) {
        minNrowsNx >>= 1;
        nInt2 ^= minNrowsNx;
        tst = ((nInt2 & minNrowsNx) == 0);
      }
      minNrowsNx = nInt2;
    }
    fv_data[minNrowsNx] = y_data[nInt2m1 - 1];
    if (n2blue > 1) {
      for (i = 0; i <= rt; i += 2) {
        b_nt_re_tmp = fv_data[i + 1].re;
        nt_re = fv_data[i + 1].im;
        twid_im = fv_data[i].re;
        nt_im = fv_data[i].im;
        fv_data[i + 1].re = twid_im - b_nt_re_tmp;
        fv_data[i + 1].im = nt_im - nt_re;
        fv_data[i].re = twid_im + b_nt_re_tmp;
        fv_data[i].im = nt_im + nt_re;
      }
    }
    minNrowsNx = 2;
    nInt2m1 = 4;
    rt = ((k - 1) << 2) + 1;
    while (k > 0) {
      for (i = 0; i < rt; i += nInt2m1) {
        nt_re_tmp = i + minNrowsNx;
        nt_re = fv_data[nt_re_tmp].re;
        nt_im = fv_data[nt_re_tmp].im;
        fv_data[nt_re_tmp].re = fv_data[i].re - nt_re;
        fv_data[nt_re_tmp].im = fv_data[i].im - nt_im;
        fv_data[i].re += nt_re;
        fv_data[i].im += nt_im;
      }
      nInt2 = 1;
      for (j = k; j < nRowsD2; j += k) {
        twid_re = costab_data[j];
        twid_im = sintab_data[j];
        i = nInt2;
        ihi = nInt2 + rt;
        while (i < ihi) {
          nt_re_tmp = i + minNrowsNx;
          b_nt_re_tmp = fv_data[nt_re_tmp].im;
          nt_im = fv_data[nt_re_tmp].re;
          nt_re = twid_re * nt_im - twid_im * b_nt_re_tmp;
          nt_im = twid_re * b_nt_re_tmp + twid_im * nt_im;
          fv_data[nt_re_tmp].re = fv_data[i].re - nt_re;
          fv_data[nt_re_tmp].im = fv_data[i].im - nt_im;
          fv_data[i].re += nt_re;
          fv_data[i].im += nt_im;
          i += nInt2m1;
        }
        nInt2++;
      }
      k /= 2;
      minNrowsNx = nInt2m1;
      nInt2m1 += nInt2m1;
      rt -= minNrowsNx;
    }
    d_FFTImplementationCallback_r2b(wwc, n2blue, costab, sintab, b_fv);
    ihi = b_fv->size[0];
    b_fv->size[0] = fv->size[0];
    emxEnsureCapacity_creal_T(b_fv, ihi);
    b_fv_data = b_fv->data;
    minNrowsNx = fv->size[0];
    for (ihi = 0; ihi < minNrowsNx; ihi++) {
      twid_im = fv_data[ihi].re;
      nt_im = b_fv_data[ihi].im;
      nt_re = fv_data[ihi].im;
      twid_re = b_fv_data[ihi].re;
      b_fv_data[ihi].re = twid_im * twid_re - nt_re * nt_im;
      b_fv_data[ihi].im = twid_im * nt_im + nt_re * twid_re;
    }
    e_FFTImplementationCallback_r2b(b_fv, n2blue, costab, sintabinv, fv);
    fv_data = fv->data;
    ihi = wwc->size[0];
    for (k = nfft; k <= ihi; k++) {
      nt_im = wwc_data[k - 1].re;
      nt_re = wwc_data[k - 1].im;
      minNrowsNx = k - nfft;
      y_data[minNrowsNx].re =
          nt_im * fv_data[k - 1].re + nt_re * fv_data[k - 1].im;
      y_data[minNrowsNx].im =
          nt_im * fv_data[k - 1].im - nt_re * fv_data[k - 1].re;
    }
  }
  emxFree_creal_T(&b_fv);
  emxFree_creal_T(&fv);
  emxFree_creal_T(&wwc);
}

/*
 * Arguments    : int nRows
 *                bool useRadix2
 *                emxArray_real_T *costab
 *                emxArray_real_T *sintab
 *                emxArray_real_T *sintabinv
 * Return Type  : void
 */
static void c_FFTImplementationCallback_gen(int nRows, bool useRadix2,
                                            emxArray_real_T *costab,
                                            emxArray_real_T *sintab,
                                            emxArray_real_T *sintabinv)
{
  emxArray_real_T *costab1q;
  double e;
  double *costab1q_data;
  double *costab_data;
  double *sintab_data;
  double *sintabinv_data;
  int i;
  int k;
  int n;
  int nd2;
  emxInit_real_T(&costab1q, 2);
  e = 6.2831853071795862 / (double)nRows;
  n = nRows / 2 / 2;
  i = costab1q->size[0] * costab1q->size[1];
  costab1q->size[1] = n + 1;
  costab1q->size[0] = 1;
  emxEnsureCapacity_real_T(costab1q, i);
  costab1q_data = costab1q->data;
  costab1q_data[0] = 1.0;
  nd2 = n / 2 - 1;
  for (k = 0; k <= nd2; k++) {
    costab1q_data[k + 1] = cos(e * ((double)k + 1.0));
  }
  i = nd2 + 2;
  nd2 = n - 1;
  for (k = i; k <= nd2; k++) {
    costab1q_data[k] = sin(e * (double)(n - k));
  }
  costab1q_data[n] = 0.0;
  if (!useRadix2) {
    n = costab1q->size[1] - 1;
    nd2 = (costab1q->size[1] - 1) << 1;
    i = costab->size[0] * costab->size[1];
    costab->size[1] = (short)(nd2 + 1);
    costab->size[0] = 1;
    emxEnsureCapacity_real_T(costab, i);
    costab_data = costab->data;
    i = sintab->size[0] * sintab->size[1];
    sintab->size[1] = (short)(nd2 + 1);
    sintab->size[0] = 1;
    emxEnsureCapacity_real_T(sintab, i);
    sintab_data = sintab->data;
    costab_data[0] = 1.0;
    sintab_data[0] = 0.0;
    i = sintabinv->size[0] * sintabinv->size[1];
    sintabinv->size[1] = (short)(nd2 + 1);
    sintabinv->size[0] = 1;
    emxEnsureCapacity_real_T(sintabinv, i);
    sintabinv_data = sintabinv->data;
    for (k = 0; k < n; k++) {
      sintabinv_data[k + 1] = costab1q_data[(n - k) - 1];
    }
    i = costab1q->size[1];
    for (k = i; k <= nd2; k++) {
      sintabinv_data[k] = costab1q_data[k - n];
    }
    for (k = 0; k < n; k++) {
      costab_data[k + 1] = costab1q_data[k + 1];
      sintab_data[k + 1] = -costab1q_data[(n - k) - 1];
    }
    i = costab1q->size[1];
    for (k = i; k <= nd2; k++) {
      costab_data[k] = -costab1q_data[nd2 - k];
      sintab_data[k] = -costab1q_data[k - n];
    }
  } else {
    n = costab1q->size[1] - 1;
    nd2 = (costab1q->size[1] - 1) << 1;
    i = costab->size[0] * costab->size[1];
    costab->size[1] = (short)(nd2 + 1);
    costab->size[0] = 1;
    emxEnsureCapacity_real_T(costab, i);
    costab_data = costab->data;
    i = sintab->size[0] * sintab->size[1];
    sintab->size[1] = (short)(nd2 + 1);
    sintab->size[0] = 1;
    emxEnsureCapacity_real_T(sintab, i);
    sintab_data = sintab->data;
    costab_data[0] = 1.0;
    sintab_data[0] = 0.0;
    for (k = 0; k < n; k++) {
      costab_data[k + 1] = costab1q_data[k + 1];
      sintab_data[k + 1] = -costab1q_data[(n - k) - 1];
    }
    i = costab1q->size[1];
    for (k = i; k <= nd2; k++) {
      costab_data[k] = -costab1q_data[nd2 - k];
      sintab_data[k] = -costab1q_data[k - n];
    }
    sintabinv->size[1] = 0;
    sintabinv->size[0] = 1;
  }
  emxFree_real_T(&costab1q);
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                int unsigned_nRows
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
static void c_FFTImplementationCallback_r2b(const emxArray_creal_T *x,
                                            int unsigned_nRows,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab,
                                            emxArray_creal_T *y)
{
  emxArray_creal_T *b_y;
  const creal_T *x_data;
  creal_T *b_y_data;
  creal_T *y_data;
  const double *costab_data;
  const double *sintab_data;
  double temp_im;
  double temp_re;
  double temp_re_tmp;
  double twid_im;
  double twid_re;
  int b_temp_re_tmp;
  int i;
  int iDelta2;
  int ihi;
  int istart;
  int iy;
  int j;
  int ju;
  int k;
  int nRowsD2;
  bool tst;
  sintab_data = sintab->data;
  costab_data = costab->data;
  x_data = x->data;
  emxInit_creal_T(&b_y);
  ju = b_y->size[0];
  b_y->size[0] = unsigned_nRows;
  emxEnsureCapacity_creal_T(b_y, ju);
  y_data = b_y->data;
  if (unsigned_nRows > x->size[0]) {
    ju = b_y->size[0];
    b_y->size[0] = unsigned_nRows;
    emxEnsureCapacity_creal_T(b_y, ju);
    y_data = b_y->data;
    for (ju = 0; ju < unsigned_nRows; ju++) {
      y_data[ju].re = 0.0;
      y_data[ju].im = 0.0;
    }
  }
  ju = y->size[0];
  y->size[0] = b_y->size[0];
  emxEnsureCapacity_creal_T(y, ju);
  b_y_data = y->data;
  iy = b_y->size[0];
  for (ju = 0; ju < iy; ju++) {
    b_y_data[ju] = y_data[ju];
  }
  iDelta2 = x->size[0];
  if (iDelta2 > unsigned_nRows) {
    iDelta2 = unsigned_nRows;
  }
  istart = unsigned_nRows - 2;
  nRowsD2 = unsigned_nRows / 2;
  k = nRowsD2 / 2;
  iy = 0;
  ju = 0;
  for (i = 0; i <= iDelta2 - 2; i++) {
    b_y_data[iy] = x_data[i];
    iy = unsigned_nRows;
    tst = true;
    while (tst) {
      iy >>= 1;
      ju ^= iy;
      tst = ((ju & iy) == 0);
    }
    iy = ju;
  }
  b_y_data[iy] = x_data[iDelta2 - 1];
  ju = b_y->size[0];
  b_y->size[0] = y->size[0];
  emxEnsureCapacity_creal_T(b_y, ju);
  y_data = b_y->data;
  iy = y->size[0];
  for (ju = 0; ju < iy; ju++) {
    y_data[ju] = b_y_data[ju];
  }
  if (unsigned_nRows > 1) {
    for (i = 0; i <= istart; i += 2) {
      temp_re_tmp = y_data[i + 1].re;
      temp_im = y_data[i + 1].im;
      temp_re = y_data[i].re;
      twid_re = y_data[i].im;
      y_data[i + 1].re = temp_re - temp_re_tmp;
      y_data[i + 1].im = twid_re - temp_im;
      y_data[i].re = temp_re + temp_re_tmp;
      y_data[i].im = twid_re + temp_im;
    }
  }
  iy = 2;
  iDelta2 = 4;
  ju = ((k - 1) << 2) + 1;
  while (k > 0) {
    for (i = 0; i < ju; i += iDelta2) {
      b_temp_re_tmp = i + iy;
      temp_re = y_data[b_temp_re_tmp].re;
      temp_im = y_data[b_temp_re_tmp].im;
      y_data[b_temp_re_tmp].re = y_data[i].re - temp_re;
      y_data[b_temp_re_tmp].im = y_data[i].im - temp_im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }
    istart = 1;
    for (j = k; j < nRowsD2; j += k) {
      twid_re = costab_data[j];
      twid_im = sintab_data[j];
      i = istart;
      ihi = istart + ju;
      while (i < ihi) {
        b_temp_re_tmp = i + iy;
        temp_re_tmp = y_data[b_temp_re_tmp].im;
        temp_im = y_data[b_temp_re_tmp].re;
        temp_re = twid_re * temp_im - twid_im * temp_re_tmp;
        temp_im = twid_re * temp_re_tmp + twid_im * temp_im;
        y_data[b_temp_re_tmp].re = y_data[i].re - temp_re;
        y_data[b_temp_re_tmp].im = y_data[i].im - temp_im;
        y_data[i].re += temp_re;
        y_data[i].im += temp_im;
        i += iDelta2;
      }
      istart++;
    }
    k /= 2;
    iy = iDelta2;
    iDelta2 += iDelta2;
    ju -= iy;
  }
  ju = y->size[0];
  y->size[0] = b_y->size[0];
  emxEnsureCapacity_creal_T(y, ju);
  b_y_data = y->data;
  iy = b_y->size[0];
  for (ju = 0; ju < iy; ju++) {
    b_y_data[ju] = y_data[ju];
  }
  emxFree_creal_T(&b_y);
}

/*
 * Arguments    : const emxArray_real_T *x
 *                emxArray_creal_T *y
 *                int nrowsx
 *                int nRows
 *                int nfft
 *                const emxArray_creal_T *wwc
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                const emxArray_real_T *costabinv
 *                const emxArray_real_T *sintabinv
 * Return Type  : void
 */
static void d_FFTImplementationCallback_doH(
    const emxArray_real_T *x, emxArray_creal_T *y, int nrowsx, int nRows,
    int nfft, const emxArray_creal_T *wwc, const emxArray_real_T *costab,
    const emxArray_real_T *sintab, const emxArray_real_T *costabinv,
    const emxArray_real_T *sintabinv)
{
  emxArray_creal_T *fv;
  emxArray_creal_T *r;
  emxArray_creal_T *reconVar1;
  emxArray_creal_T *reconVar2;
  emxArray_creal_T *ytmp;
  emxArray_real_T *a__1;
  emxArray_real_T *costable;
  emxArray_real_T *hcostabinv;
  emxArray_real_T *hsintab;
  emxArray_real_T *hsintabinv;
  emxArray_real_T *sintable;
  const creal_T *wwc_data;
  creal_T *fv_data;
  creal_T *r1;
  creal_T *reconVar1_data;
  creal_T *reconVar2_data;
  creal_T *y_data;
  creal_T *ytmp_data;
  cuint8_T b_y_data[10000];
  const double *costab_data;
  const double *costabinv_data;
  const double *sintab_data;
  const double *sintabinv_data;
  const double *x_data;
  double a_im;
  double a_re;
  double b_im;
  double b_re;
  double b_ytmp_re_tmp;
  double ytmp_im_tmp;
  double ytmp_re_tmp;
  double z_tmp;
  double *a__1_data;
  double *costable_data;
  double *hcostabinv_data;
  double *hsintab_data;
  double *hsintabinv_data;
  double *sintable_data;
  int wrapIndex_data[5000];
  int a__1_tmp;
  int b_i;
  int hnRows;
  int hszCostab;
  int i;
  int u0;
  bool nxeven;
  sintabinv_data = sintabinv->data;
  costabinv_data = costabinv->data;
  sintab_data = sintab->data;
  costab_data = costab->data;
  wwc_data = wwc->data;
  y_data = y->data;
  x_data = x->data;
  hnRows = nRows / 2;
  if ((hnRows > nrowsx) && (0 <= hnRows - 1)) {
    memset(&b_y_data[0], 0, hnRows * sizeof(cuint8_T));
  }
  emxInit_creal_T(&ytmp);
  i = ytmp->size[0];
  ytmp->size[0] = hnRows;
  emxEnsureCapacity_creal_T(ytmp, i);
  ytmp_data = ytmp->data;
  for (i = 0; i < hnRows; i++) {
    ytmp_data[i].re = 0.0;
    ytmp_data[i].im = b_y_data[i].im;
  }
  if ((x->size[0] & 1) == 0) {
    nxeven = true;
    u0 = x->size[0];
  } else if (x->size[0] >= nRows) {
    nxeven = true;
    u0 = nRows;
  } else {
    nxeven = false;
    u0 = x->size[0] - 1;
  }
  emxInit_real_T(&a__1, 2);
  emxInit_real_T(&costable, 2);
  emxInit_real_T(&sintable, 2);
  emxInit_real_T(&hsintab, 2);
  emxInit_real_T(&hcostabinv, 2);
  emxInit_real_T(&hsintabinv, 2);
  if (u0 > nRows) {
    u0 = nRows;
  }
  d_FFTImplementationCallback_gen(nRows << 1, costable, sintable, a__1);
  sintable_data = sintable->data;
  costable_data = costable->data;
  hszCostab = costab->size[1] / 2;
  i = a__1->size[0] * a__1->size[1];
  a__1->size[1] = (short)hszCostab;
  a__1->size[0] = 1;
  emxEnsureCapacity_real_T(a__1, i);
  a__1_data = a__1->data;
  i = hsintab->size[0] * hsintab->size[1];
  hsintab->size[1] = (short)hszCostab;
  hsintab->size[0] = 1;
  emxEnsureCapacity_real_T(hsintab, i);
  hsintab_data = hsintab->data;
  i = hcostabinv->size[0] * hcostabinv->size[1];
  hcostabinv->size[1] = (short)hszCostab;
  hcostabinv->size[0] = 1;
  emxEnsureCapacity_real_T(hcostabinv, i);
  hcostabinv_data = hcostabinv->data;
  i = hsintabinv->size[0] * hsintabinv->size[1];
  hsintabinv->size[1] = (short)hszCostab;
  hsintabinv->size[0] = 1;
  emxEnsureCapacity_real_T(hsintabinv, i);
  hsintabinv_data = hsintabinv->data;
  for (b_i = 0; b_i < hszCostab; b_i++) {
    a__1_tmp = ((b_i + 1) << 1) - 2;
    a__1_data[b_i] = costab_data[a__1_tmp];
    hsintab_data[b_i] = sintab_data[a__1_tmp];
    hcostabinv_data[b_i] = costabinv_data[a__1_tmp];
    hsintabinv_data[b_i] = sintabinv_data[a__1_tmp];
  }
  emxInit_creal_T(&reconVar1);
  emxInit_creal_T(&reconVar2);
  i = reconVar1->size[0];
  reconVar1->size[0] = hnRows;
  emxEnsureCapacity_creal_T(reconVar1, i);
  reconVar1_data = reconVar1->data;
  i = reconVar2->size[0];
  reconVar2->size[0] = hnRows;
  emxEnsureCapacity_creal_T(reconVar2, i);
  reconVar2_data = reconVar2->data;
  for (b_i = 0; b_i < hnRows; b_i++) {
    i = b_i << 1;
    z_tmp = sintable_data[i];
    a_re = costable_data[i];
    reconVar1_data[b_i].re = z_tmp + 1.0;
    reconVar1_data[b_i].im = -a_re;
    reconVar2_data[b_i].re = 1.0 - z_tmp;
    reconVar2_data[b_i].im = a_re;
    if (b_i + 1 != 1) {
      wrapIndex_data[b_i] = (hnRows - b_i) + 1;
    } else {
      wrapIndex_data[0] = 1;
    }
  }
  emxFree_real_T(&sintable);
  emxFree_real_T(&costable);
  z_tmp = (double)u0 / 2.0;
  i = (int)((double)u0 / 2.0);
  for (b_i = 0; b_i < i; b_i++) {
    hszCostab = (hnRows + b_i) - 1;
    a_re = wwc_data[hszCostab].re;
    a_im = wwc_data[hszCostab].im;
    a__1_tmp = b_i << 1;
    b_re = x_data[a__1_tmp];
    b_im = x_data[a__1_tmp + 1];
    ytmp_data[b_i].re = a_re * b_re + a_im * b_im;
    ytmp_data[b_i].im = a_re * b_im - a_im * b_re;
  }
  if (!nxeven) {
    hszCostab = (hnRows + (int)z_tmp) - 1;
    a_re = wwc_data[hszCostab].re;
    a_im = wwc_data[hszCostab].im;
    b_re = x_data[(int)z_tmp << 1];
    ytmp_data[(int)z_tmp].re = a_re * b_re + a_im * 0.0;
    ytmp_data[(int)z_tmp].im = a_re * 0.0 - a_im * b_re;
    if ((int)z_tmp + 2 <= hnRows) {
      i = (int)((double)u0 / 2.0) + 2;
      for (b_i = i; b_i <= hnRows; b_i++) {
        ytmp_data[b_i - 1].re = 0.0;
        ytmp_data[b_i - 1].im = 0.0;
      }
    }
  } else if ((int)z_tmp + 1 <= hnRows) {
    i = (int)((double)u0 / 2.0) + 1;
    for (b_i = i; b_i <= hnRows; b_i++) {
      ytmp_data[b_i - 1].re = 0.0;
      ytmp_data[b_i - 1].im = 0.0;
    }
  }
  emxInit_creal_T(&fv);
  emxInit_creal_T(&r);
  a__1_tmp = (int)((double)nfft / 2.0);
  c_FFTImplementationCallback_r2b(ytmp, a__1_tmp, a__1, hsintab, fv);
  fv_data = fv->data;
  d_FFTImplementationCallback_r2b(wwc, a__1_tmp, a__1, hsintab, r);
  r1 = r->data;
  hszCostab = fv->size[0];
  emxFree_real_T(&hsintab);
  emxFree_real_T(&a__1);
  for (i = 0; i < hszCostab; i++) {
    z_tmp = fv_data[i].re;
    a_re = r1[i].im;
    a_im = fv_data[i].im;
    b_re = r1[i].re;
    fv_data[i].re = z_tmp * b_re - a_im * a_re;
    fv_data[i].im = z_tmp * a_re + a_im * b_re;
  }
  e_FFTImplementationCallback_r2b(fv, a__1_tmp, hcostabinv, hsintabinv, r);
  r1 = r->data;
  i = fv->size[0];
  fv->size[0] = r->size[0];
  emxEnsureCapacity_creal_T(fv, i);
  fv_data = fv->data;
  hszCostab = r->size[0];
  emxFree_real_T(&hsintabinv);
  emxFree_real_T(&hcostabinv);
  for (i = 0; i < hszCostab; i++) {
    fv_data[i] = r1[i];
  }
  emxFree_creal_T(&r);
  i = wwc->size[0];
  for (hszCostab = hnRows; hszCostab <= i; hszCostab++) {
    z_tmp = wwc_data[hszCostab - 1].re;
    a_re = fv_data[hszCostab - 1].im;
    a_im = wwc_data[hszCostab - 1].im;
    b_re = fv_data[hszCostab - 1].re;
    a__1_tmp = hszCostab - hnRows;
    ytmp_data[a__1_tmp].re = z_tmp * b_re + a_im * a_re;
    ytmp_data[a__1_tmp].im = z_tmp * a_re - a_im * b_re;
  }
  emxFree_creal_T(&fv);
  for (b_i = 0; b_i < hnRows; b_i++) {
    i = wrapIndex_data[b_i];
    z_tmp = ytmp_data[b_i].re;
    a_re = reconVar1_data[b_i].im;
    a_im = ytmp_data[b_i].im;
    b_re = reconVar1_data[b_i].re;
    b_im = ytmp_data[i - 1].re;
    ytmp_im_tmp = -ytmp_data[i - 1].im;
    ytmp_re_tmp = reconVar2_data[b_i].im;
    b_ytmp_re_tmp = reconVar2_data[b_i].re;
    y_data[b_i].re = 0.5 * ((z_tmp * b_re - a_im * a_re) +
                            (b_im * b_ytmp_re_tmp - ytmp_im_tmp * ytmp_re_tmp));
    y_data[b_i].im = 0.5 * ((z_tmp * a_re + a_im * b_re) +
                            (b_im * ytmp_re_tmp + ytmp_im_tmp * b_ytmp_re_tmp));
    i = hnRows + b_i;
    y_data[i].re = 0.5 * ((z_tmp * b_ytmp_re_tmp - a_im * ytmp_re_tmp) +
                          (b_im * b_re - ytmp_im_tmp * a_re));
    y_data[i].im = 0.5 * ((z_tmp * ytmp_re_tmp + a_im * b_ytmp_re_tmp) +
                          (b_im * a_re + ytmp_im_tmp * b_re));
  }
  emxFree_creal_T(&reconVar2);
  emxFree_creal_T(&reconVar1);
  emxFree_creal_T(&ytmp);
}

/*
 * Arguments    : int nRows
 *                emxArray_real_T *costab
 *                emxArray_real_T *sintab
 *                emxArray_real_T *sintabinv
 * Return Type  : void
 */
static void d_FFTImplementationCallback_gen(int nRows, emxArray_real_T *costab,
                                            emxArray_real_T *sintab,
                                            emxArray_real_T *sintabinv)
{
  emxArray_real_T *b_costab;
  emxArray_real_T *b_sintab;
  emxArray_real_T *b_sintabinv;
  emxArray_real_T *costab1q;
  double e;
  double *b_costab_data;
  double *costab_data;
  double *sintab_data;
  double *sintabinv_data;
  int i;
  int k;
  int n;
  int nd2;
  emxInit_real_T(&costab1q, 2);
  e = 6.2831853071795862 / (double)nRows;
  n = nRows / 2 / 2;
  i = costab1q->size[0] * costab1q->size[1];
  costab1q->size[1] = n + 1;
  costab1q->size[0] = 1;
  emxEnsureCapacity_real_T(costab1q, i);
  costab_data = costab1q->data;
  costab_data[0] = 1.0;
  nd2 = n / 2 - 1;
  for (k = 0; k <= nd2; k++) {
    costab_data[k + 1] = cos(e * ((double)k + 1.0));
  }
  i = nd2 + 2;
  nd2 = n - 1;
  for (k = i; k <= nd2; k++) {
    costab_data[k] = sin(e * (double)(n - k));
  }
  emxInit_real_T(&b_costab, 2);
  emxInit_real_T(&b_sintab, 2);
  emxInit_real_T(&b_sintabinv, 2);
  costab_data[n] = 0.0;
  n = costab1q->size[1] - 1;
  nd2 = (costab1q->size[1] - 1) << 1;
  i = b_costab->size[0] * b_costab->size[1];
  b_costab->size[1] = (short)(nd2 + 1);
  b_costab->size[0] = 1;
  emxEnsureCapacity_real_T(b_costab, i);
  b_costab_data = b_costab->data;
  i = b_sintab->size[0] * b_sintab->size[1];
  b_sintab->size[1] = (short)(nd2 + 1);
  b_sintab->size[0] = 1;
  emxEnsureCapacity_real_T(b_sintab, i);
  sintab_data = b_sintab->data;
  b_costab_data[0] = 1.0;
  sintab_data[0] = 0.0;
  i = b_sintabinv->size[0] * b_sintabinv->size[1];
  b_sintabinv->size[1] = (short)(nd2 + 1);
  b_sintabinv->size[0] = 1;
  emxEnsureCapacity_real_T(b_sintabinv, i);
  sintabinv_data = b_sintabinv->data;
  for (k = 0; k < n; k++) {
    sintabinv_data[k + 1] = costab_data[(n - k) - 1];
  }
  i = costab1q->size[1];
  for (k = i; k <= nd2; k++) {
    sintabinv_data[k] = costab_data[k - n];
  }
  for (k = 0; k < n; k++) {
    b_costab_data[k + 1] = costab_data[k + 1];
    sintab_data[k + 1] = -costab_data[(n - k) - 1];
  }
  i = costab1q->size[1];
  for (k = i; k <= nd2; k++) {
    b_costab_data[k] = -costab_data[nd2 - k];
    sintab_data[k] = -costab_data[k - n];
  }
  emxFree_real_T(&costab1q);
  i = costab->size[0] * costab->size[1];
  costab->size[1] = b_costab->size[1];
  costab->size[0] = 1;
  emxEnsureCapacity_real_T(costab, i);
  costab_data = costab->data;
  nd2 = b_costab->size[1];
  for (i = 0; i < nd2; i++) {
    costab_data[i] = b_costab_data[i];
  }
  emxFree_real_T(&b_costab);
  i = sintab->size[0] * sintab->size[1];
  sintab->size[1] = b_sintab->size[1];
  sintab->size[0] = 1;
  emxEnsureCapacity_real_T(sintab, i);
  costab_data = sintab->data;
  nd2 = b_sintab->size[1];
  for (i = 0; i < nd2; i++) {
    costab_data[i] = sintab_data[i];
  }
  emxFree_real_T(&b_sintab);
  i = sintabinv->size[0] * sintabinv->size[1];
  sintabinv->size[1] = b_sintabinv->size[1];
  sintabinv->size[0] = 1;
  emxEnsureCapacity_real_T(sintabinv, i);
  costab_data = sintabinv->data;
  nd2 = b_sintabinv->size[1];
  for (i = 0; i < nd2; i++) {
    costab_data[i] = sintabinv_data[i];
  }
  emxFree_real_T(&b_sintabinv);
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                int n1_unsigned
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
static void d_FFTImplementationCallback_r2b(const emxArray_creal_T *x,
                                            int n1_unsigned,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab,
                                            emxArray_creal_T *y)
{
  const creal_T *x_data;
  creal_T *y_data;
  const double *costab_data;
  const double *sintab_data;
  double temp_im;
  double temp_re;
  double temp_re_tmp;
  double twid_im;
  double twid_re;
  int b_temp_re_tmp;
  int i;
  int iDelta2;
  int iheight;
  int ihi;
  int iy;
  int j;
  int ju;
  int k;
  int nRowsD2;
  bool tst;
  sintab_data = sintab->data;
  costab_data = costab->data;
  x_data = x->data;
  iy = y->size[0];
  y->size[0] = n1_unsigned;
  emxEnsureCapacity_creal_T(y, iy);
  y_data = y->data;
  if (n1_unsigned > x->size[0]) {
    iy = y->size[0];
    y->size[0] = n1_unsigned;
    emxEnsureCapacity_creal_T(y, iy);
    y_data = y->data;
    for (iy = 0; iy < n1_unsigned; iy++) {
      y_data[iy].re = 0.0;
      y_data[iy].im = 0.0;
    }
  }
  iDelta2 = x->size[0];
  if (iDelta2 > n1_unsigned) {
    iDelta2 = n1_unsigned;
  }
  iheight = n1_unsigned - 2;
  nRowsD2 = n1_unsigned / 2;
  k = nRowsD2 / 2;
  iy = 0;
  ju = 0;
  for (i = 0; i <= iDelta2 - 2; i++) {
    y_data[iy] = x_data[i];
    iy = n1_unsigned;
    tst = true;
    while (tst) {
      iy >>= 1;
      ju ^= iy;
      tst = ((ju & iy) == 0);
    }
    iy = ju;
  }
  y_data[iy] = x_data[iDelta2 - 1];
  if (n1_unsigned > 1) {
    for (i = 0; i <= iheight; i += 2) {
      temp_re_tmp = y_data[i + 1].re;
      temp_im = y_data[i + 1].im;
      temp_re = y_data[i].re;
      twid_re = y_data[i].im;
      y_data[i + 1].re = temp_re - temp_re_tmp;
      y_data[i + 1].im = twid_re - temp_im;
      y_data[i].re = temp_re + temp_re_tmp;
      y_data[i].im = twid_re + temp_im;
    }
  }
  iy = 2;
  iDelta2 = 4;
  iheight = ((k - 1) << 2) + 1;
  while (k > 0) {
    for (i = 0; i < iheight; i += iDelta2) {
      b_temp_re_tmp = i + iy;
      temp_re = y_data[b_temp_re_tmp].re;
      temp_im = y_data[b_temp_re_tmp].im;
      y_data[b_temp_re_tmp].re = y_data[i].re - temp_re;
      y_data[b_temp_re_tmp].im = y_data[i].im - temp_im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }
    ju = 1;
    for (j = k; j < nRowsD2; j += k) {
      twid_re = costab_data[j];
      twid_im = sintab_data[j];
      i = ju;
      ihi = ju + iheight;
      while (i < ihi) {
        b_temp_re_tmp = i + iy;
        temp_re_tmp = y_data[b_temp_re_tmp].im;
        temp_im = y_data[b_temp_re_tmp].re;
        temp_re = twid_re * temp_im - twid_im * temp_re_tmp;
        temp_im = twid_re * temp_re_tmp + twid_im * temp_im;
        y_data[b_temp_re_tmp].re = y_data[i].re - temp_re;
        y_data[b_temp_re_tmp].im = y_data[i].im - temp_im;
        y_data[i].re += temp_re;
        y_data[i].im += temp_im;
        i += iDelta2;
      }
      ju++;
    }
    k /= 2;
    iy = iDelta2;
    iDelta2 += iDelta2;
    iheight -= iy;
  }
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                int n1_unsigned
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
static void e_FFTImplementationCallback_r2b(const emxArray_creal_T *x,
                                            int n1_unsigned,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab,
                                            emxArray_creal_T *y)
{
  const creal_T *x_data;
  creal_T *y_data;
  const double *costab_data;
  const double *sintab_data;
  double temp_im;
  double temp_re;
  double temp_re_tmp;
  double twid_im;
  double twid_re;
  int b_temp_re_tmp;
  int i;
  int iDelta2;
  int iheight;
  int ihi;
  int iy;
  int j;
  int ju;
  int k;
  int nRowsD2;
  bool tst;
  sintab_data = sintab->data;
  costab_data = costab->data;
  x_data = x->data;
  iDelta2 = y->size[0];
  y->size[0] = n1_unsigned;
  emxEnsureCapacity_creal_T(y, iDelta2);
  y_data = y->data;
  if (n1_unsigned > x->size[0]) {
    iDelta2 = y->size[0];
    y->size[0] = n1_unsigned;
    emxEnsureCapacity_creal_T(y, iDelta2);
    y_data = y->data;
    for (iDelta2 = 0; iDelta2 < n1_unsigned; iDelta2++) {
      y_data[iDelta2].re = 0.0;
      y_data[iDelta2].im = 0.0;
    }
  }
  iDelta2 = x->size[0];
  if (iDelta2 > n1_unsigned) {
    iDelta2 = n1_unsigned;
  }
  iheight = n1_unsigned - 2;
  nRowsD2 = n1_unsigned / 2;
  k = nRowsD2 / 2;
  iy = 0;
  ju = 0;
  for (i = 0; i <= iDelta2 - 2; i++) {
    y_data[iy] = x_data[i];
    iy = n1_unsigned;
    tst = true;
    while (tst) {
      iy >>= 1;
      ju ^= iy;
      tst = ((ju & iy) == 0);
    }
    iy = ju;
  }
  y_data[iy] = x_data[iDelta2 - 1];
  if (n1_unsigned > 1) {
    for (i = 0; i <= iheight; i += 2) {
      temp_re_tmp = y_data[i + 1].re;
      temp_im = y_data[i + 1].im;
      temp_re = y_data[i].re;
      twid_re = y_data[i].im;
      y_data[i + 1].re = temp_re - temp_re_tmp;
      y_data[i + 1].im = twid_re - temp_im;
      y_data[i].re = temp_re + temp_re_tmp;
      y_data[i].im = twid_re + temp_im;
    }
  }
  iy = 2;
  iDelta2 = 4;
  iheight = ((k - 1) << 2) + 1;
  while (k > 0) {
    for (i = 0; i < iheight; i += iDelta2) {
      b_temp_re_tmp = i + iy;
      temp_re = y_data[b_temp_re_tmp].re;
      temp_im = y_data[b_temp_re_tmp].im;
      y_data[b_temp_re_tmp].re = y_data[i].re - temp_re;
      y_data[b_temp_re_tmp].im = y_data[i].im - temp_im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }
    ju = 1;
    for (j = k; j < nRowsD2; j += k) {
      twid_re = costab_data[j];
      twid_im = sintab_data[j];
      i = ju;
      ihi = ju + iheight;
      while (i < ihi) {
        b_temp_re_tmp = i + iy;
        temp_re_tmp = y_data[b_temp_re_tmp].im;
        temp_im = y_data[b_temp_re_tmp].re;
        temp_re = twid_re * temp_im - twid_im * temp_re_tmp;
        temp_im = twid_re * temp_re_tmp + twid_im * temp_im;
        y_data[b_temp_re_tmp].re = y_data[i].re - temp_re;
        y_data[b_temp_re_tmp].im = y_data[i].im - temp_im;
        y_data[i].re += temp_re;
        y_data[i].im += temp_im;
        i += iDelta2;
      }
      ju++;
    }
    k /= 2;
    iy = iDelta2;
    iDelta2 += iDelta2;
    iheight -= iy;
  }
  if (y->size[0] > 1) {
    temp_im = 1.0 / (double)y->size[0];
    iy = y->size[0];
    for (iDelta2 = 0; iDelta2 < iy; iDelta2++) {
      y_data[iDelta2].re *= temp_im;
      y_data[iDelta2].im *= temp_im;
    }
  }
}

/*
 * Arguments    : const emxArray_real_T *x
 *                emxArray_creal_T *y
 * Return Type  : void
 */
static void fft(const emxArray_real_T *x, emxArray_creal_T *y)
{
  emxArray_creal_T *b_y;
  emxArray_real_T *costab;
  emxArray_real_T *sintab;
  emxArray_real_T *sintabinv;
  creal_T *b_y_data;
  creal_T *y_data;
  const double *x_data;
  int N2blue;
  int k;
  int pmax;
  int pmin;
  int pow2p;
  bool exitg1;
  bool useRadix2;
  x_data = x->data;
  if (x->size[0] == 0) {
    y->size[0] = 0;
  } else {
    useRadix2 = ((x->size[0] & (x->size[0] - 1)) == 0);
    N2blue = 1;
    if (useRadix2) {
      pmax = x->size[0];
    } else {
      N2blue = (x->size[0] + x->size[0]) - 1;
      pmax = 31;
      if (N2blue <= 1) {
        pmax = 0;
      } else {
        pmin = 0;
        exitg1 = false;
        while ((!exitg1) && (pmax - pmin > 1)) {
          k = (pmin + pmax) >> 1;
          pow2p = 1 << k;
          if (pow2p == N2blue) {
            pmax = k;
            exitg1 = true;
          } else if (pow2p > N2blue) {
            pmax = k;
          } else {
            pmin = k;
          }
        }
      }
      N2blue = 1 << pmax;
      pmax = N2blue;
    }
    emxInit_real_T(&costab, 2);
    emxInit_real_T(&sintab, 2);
    emxInit_real_T(&sintabinv, 2);
    c_FFTImplementationCallback_gen(pmax, useRadix2, costab, sintab, sintabinv);
    if (useRadix2) {
      pmax = y->size[0];
      y->size[0] = (short)x->size[0];
      emxEnsureCapacity_creal_T(y, pmax);
      y_data = y->data;
      if (x->size[0] != 1) {
        c_FFTImplementationCallback_doH(x, y, x->size[0], costab, sintab);
      } else {
        emxInit_creal_T(&b_y);
        y_data[0].re = x_data[0];
        y_data[0].im = 0.0;
        pmax = b_y->size[0];
        b_y->size[0] = (short)x->size[0];
        emxEnsureCapacity_creal_T(b_y, pmax);
        b_y_data = b_y->data;
        N2blue = (short)x->size[0];
        for (pmax = 0; pmax < N2blue; pmax++) {
          b_y_data[pmax] = y_data[pmax];
        }
        pmax = y->size[0];
        y->size[0] = b_y->size[0];
        emxEnsureCapacity_creal_T(y, pmax);
        y_data = y->data;
        N2blue = b_y->size[0];
        for (pmax = 0; pmax < N2blue; pmax++) {
          y_data[pmax] = b_y_data[pmax];
        }
        emxFree_creal_T(&b_y);
      }
    } else {
      c_FFTImplementationCallback_dob(x, N2blue, x->size[0], costab, sintab,
                                      sintabinv, y);
    }
    emxFree_real_T(&sintabinv);
    emxFree_real_T(&sintab);
    emxFree_real_T(&costab);
  }
}

/*
 * Arguments    : const emxArray_real_T *x
 *                double *ex
 *                int *idx
 * Return Type  : void
 */
static void maximum(const emxArray_real_T *x, double *ex, int *idx)
{
  const double *x_data;
  double d;
  int i;
  int k;
  int last;
  bool exitg1;
  x_data = x->data;
  last = x->size[1];
  if (x->size[1] <= 2) {
    if (x->size[1] == 1) {
      *ex = x_data[0];
      *idx = 1;
    } else if ((x_data[0] < x_data[x->size[1] - 1]) ||
               (rtIsNaN(x_data[0]) && (!rtIsNaN(x_data[x->size[1] - 1])))) {
      *ex = x_data[x->size[1] - 1];
      *idx = x->size[1];
    } else {
      *ex = x_data[0];
      *idx = 1;
    }
  } else {
    if (!rtIsNaN(x_data[0])) {
      *idx = 1;
    } else {
      *idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!rtIsNaN(x_data[k - 1])) {
          *idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (*idx == 0) {
      *ex = x_data[0];
      *idx = 1;
    } else {
      *ex = x_data[*idx - 1];
      i = *idx + 1;
      for (k = i; k <= last; k++) {
        d = x_data[k - 1];
        if (*ex < d) {
          *ex = d;
          *idx = k;
        }
      }
    }
  }
}

/*
 * Arguments    : const double x_data[]
 *                int x_size
 * Return Type  : double
 */
static double mean(const double x_data[], int x_size)
{
  double y;
  int k;
  if (x_size == 0) {
    y = 0.0;
  } else {
    y = x_data[0];
    for (k = 2; k <= x_size; k++) {
      if (x_size >= 2) {
        y += x_data[k - 1];
      }
    }
  }
  y /= (double)x_size;
  return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * sqrt(y * y + 1.0);
  } else if (!rtIsNaN(y)) {
    y = a * 1.4142135623730951;
  }
  return y;
}

/*
 * Arguments    : const double ir_vals_data[]
 *                const int ir_vals_size[1]
 *                double ir_length
 *                const double red_vals_data[]
 *                const int red_vals_size[1]
 *                double red_length
 *                const emxArray_real_T *sound_vals
 *                double sound_length
 *                double Fs_ppg
 *                double Fs_sound
 *                double previous_bpm
 * Return Type  : double
 */
double hr_processing(const double ir_vals_data[], const int ir_vals_size[1],
                     double ir_length, const double red_vals_data[],
                     const int red_vals_size[1], double red_length,
                     const emxArray_real_T *sound_vals, double sound_length,
                     double Fs_ppg, double Fs_sound, double previous_bpm)
{
  emxArray_creal_T *Y2;
  emxArray_real_T P1_data;
  emxArray_real_T c_P1_data;
  emxArray_real_T *P12;
  emxArray_real_T *P22;
  emxArray_real_T *b_P12;
  emxArray_real_T *f;
  emxArray_real_T *f2;
  emxArray_real_T *r;
  creal_T Y_data[500];
  creal_T *Y2_data;
  double b_P1_data[500];
  double b_P22_data[499];
  double a;
  double ar;
  double ex;
  double heartbeat;
  double *P12_data;
  double *P22_data;
  double *b_P12_data;
  double *f_data;
  int iv[2];
  int P1_size;
  int Y_size;
  int i;
  int i1;
  int i2;
  int i3;
  int loop_ub;
  bool exitg1;
  (void)red_length;
  /* taking fft of all values together. But they should be taken in chunks */
  /*  can be red length as well */
  emxInit_creal_T(&Y2);
  if (ir_vals_size[0] == red_vals_size[0]) {
    ar = mean(ir_vals_data, ir_vals_size[0]);
    a = mean(red_vals_data, red_vals_size[0]);
    P1_size = ir_vals_size[0];
    loop_ub = ir_vals_size[0];
    for (i = 0; i < loop_ub; i++) {
      b_P1_data[i] = (ir_vals_data[i] - ar) + (red_vals_data[i] - a);
    }
    P1_data.data = &b_P1_data[0];
    P1_data.size = &P1_size;
    P1_data.allocatedSize = 500;
    P1_data.numDimensions = 1;
    P1_data.canFreeData = false;
    fft(&P1_data, Y2);
    Y2_data = Y2->data;
  } else {
    binary_expand_op(Y2, ir_vals_data, &ir_vals_size[0], red_vals_data,
                     &red_vals_size[0]);
    Y2_data = Y2->data;
  }
  Y_size = Y2->size[0];
  loop_ub = Y2->size[0];
  for (i = 0; i < loop_ub; i++) {
    Y_data[i] = Y2_data[i];
  }
  i = Y2->size[0];
  Y2->size[0] = Y_size;
  emxEnsureCapacity_creal_T(Y2, i);
  Y2_data = Y2->data;
  for (i = 0; i < Y_size; i++) {
    ar = Y_data[i].re;
    a = Y_data[i].im;
    if (a == 0.0) {
      Y2_data[i].re = ar / ir_length;
      Y2_data[i].im = 0.0;
    } else if (ar == 0.0) {
      Y2_data[i].re = 0.0;
      Y2_data[i].im = a / ir_length;
    } else {
      Y2_data[i].re = ar / ir_length;
      Y2_data[i].im = a / ir_length;
    }
  }
  emxInit_real_T(&P22, 1);
  i = P22->size[0];
  P22->size[0] = (short)Y2->size[0];
  emxEnsureCapacity_real_T(P22, i);
  P22_data = P22->data;
  if (Y2->size[0] != 0) {
    i = (short)Y2->size[0];
    for (Y_size = 0; Y_size < i; Y_size++) {
      P22_data[Y_size] = rt_hypotd_snf(Y2_data[Y_size].re, Y2_data[Y_size].im);
    }
  }
  ar = ir_length / 2.0 + 1.0;
  if (1.0 > ar) {
    loop_ub = 0;
  } else {
    loop_ub = (int)ar;
  }
  P1_size = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    b_P1_data[i] = P22_data[i];
  }
  if (2.0 > (double)loop_ub - 1.0) {
    i = 0;
    i1 = 0;
    i2 = 0;
    i3 = 0;
  } else {
    i = 1;
    i1 = loop_ub - 1;
    i2 = 1;
    i3 = loop_ub - 1;
  }
  Y_size = i1 - i;
  for (i1 = 0; i1 < Y_size; i1++) {
    b_P22_data[i1] = P22_data[i + i1];
  }
  emxInit_real_T(&r, 1);
  i = r->size[0];
  r->size[0] = Y_size;
  emxEnsureCapacity_real_T(r, i);
  P22_data = r->data;
  for (i = 0; i < Y_size; i++) {
    P22_data[i] = 2.0 * b_P22_data[i];
  }
  Y_size = i3 - i2;
  for (i = 0; i < Y_size; i++) {
    b_P1_data[i2 + i] = P22_data[i];
  }
  emxFree_real_T(&r);
  emxInit_real_T(&f, 2);
  ar = ir_length / 2.0;
  if (rtIsNaN(ar)) {
    i = f->size[0] * f->size[1];
    f->size[1] = 1;
    f->size[0] = 1;
    emxEnsureCapacity_real_T(f, i);
    f_data = f->data;
    f_data[0] = rtNaN;
  } else if (ar < 0.0) {
    f->size[1] = 0;
  } else if (rtIsInf(ar) && (0.0 == ar)) {
    i = f->size[0] * f->size[1];
    f->size[1] = 1;
    f->size[0] = 1;
    emxEnsureCapacity_real_T(f, i);
    f_data = f->data;
    f_data[0] = rtNaN;
  } else {
    loop_ub = (int)floor(ar) + 1;
    i = f->size[0] * f->size[1];
    f->size[1] = loop_ub;
    f->size[0] = 1;
    emxEnsureCapacity_real_T(f, i);
    f_data = f->data;
    for (i = 0; i < loop_ub; i++) {
      f_data[i] = i;
    }
  }
  i = f->size[0] * f->size[1];
  f->size[0] = 1;
  emxEnsureCapacity_real_T(f, i);
  f_data = f->data;
  loop_ub = f->size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    f_data[i] = Fs_ppg * f_data[i] / ir_length;
  }
  /*  stem(f,P1) */
  /*  figure */
  fft(sound_vals, Y2);
  Y2_data = Y2->data;
  loop_ub = Y2->size[0];
  for (i = 0; i < loop_ub; i++) {
    ar = Y2_data[i].re;
    a = Y2_data[i].im;
    if (a == 0.0) {
      ar /= sound_length;
      a = 0.0;
    } else if (ar == 0.0) {
      ar = 0.0;
      a /= sound_length;
    } else {
      ar /= sound_length;
      a /= sound_length;
    }
    Y2_data[i].re = ar;
    Y2_data[i].im = a;
  }
  i = P22->size[0];
  P22->size[0] = (short)Y2->size[0];
  emxEnsureCapacity_real_T(P22, i);
  P22_data = P22->data;
  if (Y2->size[0] != 0) {
    i = (short)Y2->size[0];
    for (Y_size = 0; Y_size < i; Y_size++) {
      P22_data[Y_size] = rt_hypotd_snf(Y2_data[Y_size].re, Y2_data[Y_size].im);
    }
  }
  emxFree_creal_T(&Y2);
  emxInit_real_T(&P12, 1);
  ar = sound_length / 2.0 + 1.0;
  if (1.0 > ar) {
    loop_ub = 0;
  } else {
    loop_ub = (int)ar;
  }
  i = P12->size[0];
  P12->size[0] = loop_ub;
  emxEnsureCapacity_real_T(P12, i);
  P12_data = P12->data;
  for (i = 0; i < loop_ub; i++) {
    P12_data[i] = P22_data[i];
  }
  if (2.0 > (double)loop_ub - 1.0) {
    i = 0;
    i1 = 0;
    i2 = 0;
  } else {
    i = 1;
    i1 = 1;
    i2 = loop_ub - 1;
  }
  Y_size = i2 - i1;
  for (i2 = 0; i2 < Y_size; i2++) {
    P12_data[i1 + i2] = 2.0 * P22_data[i + i2];
  }
  emxFree_real_T(&P22);
  emxInit_real_T(&f2, 2);
  ar = sound_length / 2.0;
  if (rtIsNaN(ar)) {
    i = f2->size[0] * f2->size[1];
    f2->size[1] = 1;
    f2->size[0] = 1;
    emxEnsureCapacity_real_T(f2, i);
    P22_data = f2->data;
    P22_data[0] = rtNaN;
  } else if (ar < 0.0) {
    f2->size[1] = 0;
  } else if (rtIsInf(ar) && (0.0 == ar)) {
    i = f2->size[0] * f2->size[1];
    f2->size[1] = 1;
    f2->size[0] = 1;
    emxEnsureCapacity_real_T(f2, i);
    P22_data = f2->data;
    P22_data[0] = rtNaN;
  } else {
    loop_ub = (int)floor(ar) + 1;
    i = f2->size[0] * f2->size[1];
    f2->size[1] = loop_ub;
    f2->size[0] = 1;
    emxEnsureCapacity_real_T(f2, i);
    P22_data = f2->data;
    for (i = 0; i < loop_ub; i++) {
      P22_data[i] = i;
    }
  }
  i = f2->size[0] * f2->size[1];
  f2->size[0] = 1;
  emxEnsureCapacity_real_T(f2, i);
  P22_data = f2->data;
  loop_ub = f2->size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    P22_data[i] = Fs_sound * P22_data[i] / sound_length;
  }
  /*  stem(f2,P12) */
  a = 0.0;
  ar = 0.0;
  /* finding min and max frequency index (b/w 1 and 4) for ppg */
  Y_size = 0;
  exitg1 = false;
  while ((!exitg1) && (Y_size <= f->size[1] - 1)) {
    a = f_data[Y_size];
    if (f_data[Y_size] >= 1.0) {
      a = (double)Y_size + 1.0;
      /*  min frequency index */
      exitg1 = true;
    } else {
      Y_size++;
    }
  }
  Y_size = 0;
  exitg1 = false;
  while ((!exitg1) && (Y_size <= f->size[1] - 1)) {
    ar = f_data[Y_size];
    if (f_data[Y_size] >= 4.0) {
      ar = (double)Y_size + 1.0;
      /*  max frequency index */
      exitg1 = true;
    } else {
      Y_size++;
    }
  }
  if (a > ar) {
    i = -1;
    i1 = 0;
    i2 = 0;
  } else {
    i = (int)a - 2;
    i1 = (int)a - 1;
    i2 = (int)ar;
  }
  /* plot(chunk_f, chunk_p1) */
  Y_size = i2 - i1;
  for (i2 = 0; i2 < Y_size; i2++) {
    b_P1_data[i2] = b_P1_data[i1 + i2];
  }
  P1_size = Y_size;
  iv[0] = 1;
  iv[1] = Y_size;
  c_P1_data.data = &b_P1_data[0];
  c_P1_data.size = &iv[0];
  c_P1_data.allocatedSize = 500;
  c_P1_data.numDimensions = 2;
  c_P1_data.canFreeData = false;
  maximum(&c_P1_data, &ex, &loop_ub);
  /* finding min and max frequency index (b/w 1 and 4) for sound */
  Y_size = 0;
  exitg1 = false;
  while ((!exitg1) && (Y_size <= f2->size[1] - 1)) {
    a = P22_data[Y_size];
    if (P22_data[Y_size] >= 1.0) {
      a = (double)Y_size + 1.0;
      exitg1 = true;
    } else {
      Y_size++;
    }
  }
  Y_size = 0;
  exitg1 = false;
  while ((!exitg1) && (Y_size <= f2->size[1] - 1)) {
    ar = P22_data[Y_size];
    if (P22_data[Y_size] >= 4.0) {
      ar = (double)Y_size + 1.0;
      exitg1 = true;
    } else {
      Y_size++;
    }
  }
  if (a > ar) {
    i1 = -1;
    i2 = 0;
    i3 = 0;
  } else {
    i1 = (int)a - 2;
    i2 = (int)a - 1;
    i3 = (int)ar;
  }
  emxInit_real_T(&b_P12, 2);
  /* plot(chunk_f2, chunk_p12) */
  Y_size = i3 - i2;
  i3 = b_P12->size[0] * b_P12->size[1];
  b_P12->size[1] = Y_size;
  b_P12->size[0] = 1;
  emxEnsureCapacity_real_T(b_P12, i3);
  b_P12_data = b_P12->data;
  for (i3 = 0; i3 < Y_size; i3++) {
    b_P12_data[i3] = P12_data[i2 + i3];
  }
  emxFree_real_T(&P12);
  maximum(b_P12, &ex, &Y_size);
  /* the previous value would be multiplied by 0.3 when running in loop */
  heartbeat = (0.4 * (f_data[i + loop_ub] * 60.0) +
               0.3 * (P22_data[i1 + Y_size] * 60.0)) +
              0.4 * previous_bpm;
  /*  complementary filter */
  emxFree_real_T(&b_P12);
  emxFree_real_T(&f2);
  emxFree_real_T(&f);
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
