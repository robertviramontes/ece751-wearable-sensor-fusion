/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: find.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 07-Dec-2021 17:22:13
 */

/* Include Files */
#include "find.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const bool x_data[]
 *                int x_size
 *                int i_data[]
 *                int *i_size
 * Return Type  : void
 */
void eml_find(const bool x_data[], int x_size, int i_data[], int *i_size)
{
  int idx;
  int ii;
  bool exitg1;
  idx = 0;
  *i_size = x_size;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= x_size - 1)) {
    if (x_data[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= x_size) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (x_size == 1) {
    if (idx == 0) {
      *i_size = 0;
    }
  } else if (1 > idx) {
    *i_size = 0;
  } else {
    *i_size = idx;
  }
}

/*
 * File trailer for find.c
 *
 * [EOF]
 */
