/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: hr_processing.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 29-Nov-2021 22:32:31
 */

/* Include Files */
#include "hr_processing.h"

/* Function Definitions */
/*
 * HR_PROCESSING Implements the DSP block to determine the heartrate
 *    Takes in PPG data and MEMS microphone data and
 *    merges the processed signals to predict the heart rate.
 * hold off;
 * plot(ppg_data)
 * hold on;
 *  plot(mic_data)
 *  hold off;
 *  return a heart rate
 *
 * Arguments    : const double ppg_data[200]
 *                const double mic_data[200]
 * Return Type  : double
 */
double hr_processing(const double ppg_data[200], const double mic_data[200])
{
  (void)ppg_data;
  (void)mic_data;
  /* plot(locs, pks,'o','MarkerSize',12); */
  return 60.0;
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
