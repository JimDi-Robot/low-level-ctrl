/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: highgain_acro.h
 *
 * Code generated for Simulink model 'highgain_acro'.
 *
 * Model version                  : 1.0
 * Simulink Coder version         : 9.6 (R2021b) 14-May-2021
 * C/C++ source code generated on : Sat Feb 12 17:00:50 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_highgain_acro_h_
#define RTW_HEADER_highgain_acro_h_
#include <stddef.h>
#include <math.h>
#include <string.h>
#ifndef highgain_acro_COMMON_INCLUDES_
#define highgain_acro_COMMON_INCLUDES_
#include <drivers/drv_rc_input.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <drivers/drv_pwm_output.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>
#include <drivers/drv_board_led.h>
#include <uORB/topics/led_control.h>
#include <stdlib.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <systemlib/err.h>
#include <uORB/topics/jim_attctrl.h>
#include <uORB/topics/jim_actuator.h>
#include <uORB/topics/jim_gain.h>
#include <uORB/topics/jim_pid_out.h>
#include <uORB/topics/vehicle_attitude.h>
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#endif                                 /* highgain_acro_COMMON_INCLUDES_ */

#include "highgain_acro_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

typedef struct pollfd pollfd_t;
typedef struct actuator_outputs_s actuator_outputs_s_t;
typedef struct actuator_armed_s actuator_armed_s_t;
typedef struct led_control_s led_control_s_t;
typedef struct jim_attctrl_s jim_attctrl_s_t;
typedef struct jim_actuator_s jim_actuator_s_t;
typedef struct jim_gain_s jim_gain_s_t;
typedef struct jim_pid_out_s jim_pid_out_s_t;

/* Block signals for system '<S5>/deadzone0' */
typedef struct {
  real_T y;                            /* '<S5>/deadzone0' */
} B_deadzone0_highgain_acro_T;

/* Block signals (default storage) */
typedef struct {
  real_T dv[16];
  real_T M_inv[12];
  real_T dv1[12];
  real_T temp[4];
  real_T TmpSignalConversionAtSFun_k[4];/* '<S7>/MATLAB Function3' */
  real_T dv2[4];
  real_T Sum[3];                       /* '<S7>/Sum' */
  real_T c3[4];                        /* '<S7>/MATLAB Function6' */
  real_T c2[4];                        /* '<S7>/MATLAB Function5' */
  real_T b_tmp[3];
  real_T TmpSignalConversionAtSFun_f[3];/* '<S7>/MATLAB Function3' */
  real_T dv3[3];
  int8_T temp_tmp[16];
  real_T Saturation10;                 /* '<S5>/Saturation10' */
  real_T Gain;                         /* '<S5>/Gain' */
  real_T deg2rad2;                     /* '<S5>/deg2rad2' */
  real_T deg2rad3;                     /* '<S5>/deg2rad3' */
  real_T Conversion8;                  /* '<S1>/Conversion8' */
  real_T Conversion9;                  /* '<S1>/Conversion9' */
  real_T Conversion10;                 /* '<S1>/Conversion10' */
  real_T u0filter;                     /* '<S7>/30filter' */
  real_T u0filter1;                    /* '<S7>/30filter1' */
  real_T u0filter2;                    /* '<S7>/30filter2' */
  real_T u0filter3;                    /* '<S7>/30filter3' */
  real_T Conversion7;                  /* '<S1>/Conversion7' */
  real_T Conversion11;                 /* '<S1>/Conversion11' */
  real_T u0filter_tmp;
  real_T rtb_TSamp_idx_1;
  real_T rtb_TSamp_idx_2;
  real_T rtb_TSamp_idx_1_m;
  real_T u0;
  real_T a;
  real_T rtb_u0filter_c;
  real_T rtb_u0filter1_k;
  real_T rtb_u0filter2_c;
  real_T rtb_u0filter3_b;
  real_T rtb_TSamp_idx_0_tmp;
  real_T rtb_TSamp_idx_2_tmp;
  real32_T sensor_combined_o2;         /* '<Root>/sensor_combined' */
  real32_T sensor_combined_o3;         /* '<Root>/sensor_combined' */
  real32_T vehicle_attitude[4];        /* '<Root>/vehicle_attitude' */
  real32_T Conversion12;               /* '<S1>/Conversion12' */
  real32_T Conversion13;               /* '<S1>/Conversion13' */
  real32_T Conversion14;               /* '<S1>/Conversion14' */
  real32_T Conversion15;               /* '<S1>/Conversion15' */
  real32_T Conversion16;               /* '<S1>/Conversion16' */
  real32_T Conversion18;               /* '<S1>/Conversion18' */
  real32_T Conversion20;               /* '<S1>/Conversion20' */
  real32_T Conversion21;               /* '<S1>/Conversion21' */
  real32_T Conversion22;               /* '<S1>/Conversion22' */
  real32_T Conversion23;               /* '<S1>/Conversion23' */
  real32_T Conversion24;               /* '<S1>/Conversion24' */
  real32_T Conversion25;               /* '<S1>/Conversion25' */
  real32_T Conversion26;               /* '<S1>/Conversion26' */
  real32_T Conversion27;               /* '<S1>/Conversion27' */
  real32_T Conversion28;               /* '<S1>/Conversion28' */
  real32_T Conversion29;               /* '<S1>/Conversion29' */
  real32_T aSq;
  real32_T bSq;
  uint32_T Conversion19;               /* '<S1>/Conversion19' */
  real32_T sensor_combined_o1;         /* '<Root>/sensor_combined' */
  RGBLED_MODE_ENUM Switch;             /* '<S2>/Switch' */
  RGBLED_COLOR_ENUM Switch1;           /* '<S2>/Switch1' */
  uint16_T input_rc_o1;                /* '<Root>/input_rc' */
  uint16_T input_rc_o2;                /* '<Root>/input_rc' */
  uint16_T input_rc_o3;                /* '<Root>/input_rc' */
  uint16_T input_rc_o4;                /* '<Root>/input_rc' */
  uint16_T input_rc_o5;                /* '<Root>/input_rc' */
  uint16_T DataTypeConversion2[4];     /* '<Root>/Data Type Conversion2' */
  boolean_T Compare;                   /* '<S27>/Compare' */
  boolean_T Compare_e;                 /* '<S21>/Compare' */
  boolean_T Compare_f;                 /* '<S20>/Compare' */
  B_deadzone0_highgain_acro_T sf_deadzone3;/* '<S5>/deadzone3' */
  B_deadzone0_highgain_acro_T sf_deadzone2;/* '<S5>/deadzone2' */
  B_deadzone0_highgain_acro_T sf_deadzone1;/* '<S5>/deadzone1' */
  B_deadzone0_highgain_acro_T sf_deadzone0;/* '<S5>/deadzone0' */
} B_highgain_acro_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  jim_attctrl_s_t uORBWriteAdvanced_uorb_msg;/* '<S1>/uORB Write Advanced' */
  jim_pid_out_s_t uORBWriteAdvanced3_uorb_msg;/* '<S1>/uORB Write Advanced3' */
  jim_actuator_s_t uORBWriteAdvanced1_uorb_msg;/* '<S1>/uORB Write Advanced1' */
  jim_gain_s_t uORBWriteAdvanced2_uorb_msg;/* '<S1>/uORB Write Advanced2' */
  led_control_s_t RGB_LED_sl_led_control_s;/* '<Root>/RGB_LED' */
  pollfd_t input_rc_input_rc_fd;       /* '<Root>/input_rc' */
  pollfd_t sensor_combined_sensor_fd;  /* '<Root>/sensor_combined' */
  pollfd_t sensor_combined_mag_fd;     /* '<Root>/sensor_combined' */
  pollfd_t sensor_combined_baro_fd;    /* '<Root>/sensor_combined' */
  pollfd_t vehicle_attitude_vehicle_attitu;/* '<Root>/vehicle_attitude' */
  real_T Delay2_DSTATE[4];             /* '<S7>/Delay2' */
  real_T UD_DSTATE[3];                 /* '<S8>/UD' */
  real_T u0filter_states[3];           /* '<S7>/30filter' */
  real_T u0filter1_states[3];          /* '<S7>/30filter1' */
  real_T u0filter2_states[3];          /* '<S7>/30filter2' */
  real_T u0filter3_states[3];          /* '<S7>/30filter3' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S16>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator1_DSTATE;/* '<S16>/Discrete-Time Integrator1' */
  real_T DiscreteTimeIntegrator2_DSTATE;/* '<S16>/Discrete-Time Integrator2' */
  real_T DiscreteTimeIntegrator_DSTATE_m;/* '<S17>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator1_DSTAT_f;/* '<S17>/Discrete-Time Integrator1' */
  real_T DiscreteTimeIntegrator2_DSTAT_j;/* '<S17>/Discrete-Time Integrator2' */
  real_T DiscreteTimeIntegrator3_DSTATE;/* '<S17>/Discrete-Time Integrator3' */
  orb_advert_t RGB_LED_orb_advert_t;   /* '<Root>/RGB_LED' */
  orb_advert_t uORBWriteAdvanced_uorb_advert;/* '<S1>/uORB Write Advanced' */
  orb_advert_t uORBWriteAdvanced1_uorb_advert;/* '<S1>/uORB Write Advanced1' */
  orb_advert_t uORBWriteAdvanced2_uorb_advert;/* '<S1>/uORB Write Advanced2' */
  orb_advert_t uORBWriteAdvanced3_uorb_advert;/* '<S1>/uORB Write Advanced3' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S16>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevRes;/* '<S16>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator2_PrevRes;/* '<S16>/Discrete-Time Integrator2' */
  int8_T DiscreteTimeIntegrator_PrevRe_f;/* '<S17>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevR_c;/* '<S17>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator2_PrevR_o;/* '<S17>/Discrete-Time Integrator2' */
  int8_T DiscreteTimeIntegrator3_PrevRes;/* '<S17>/Discrete-Time Integrator3' */
  boolean_T RateTransition_Buffer;     /* '<Root>/Rate Transition' */
} DW_highgain_acro_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState u0filter_Reset_ZCE;       /* '<S7>/30filter' */
  ZCSigState u0filter1_Reset_ZCE;      /* '<S7>/30filter1' */
  ZCSigState u0filter2_Reset_ZCE;      /* '<S7>/30filter2' */
  ZCSigState u0filter3_Reset_ZCE;      /* '<S7>/30filter3' */
} PrevZCX_highgain_acro_T;

/* Real-time Model Data Structure */
struct tag_RTM_highgain_acro_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    struct {
      uint16_T TID[3];
    } TaskCounters;
  } Timing;
};

/* Block signals (default storage) */
extern B_highgain_acro_T highgain_acro_B;

/* Block states (default storage) */
extern DW_highgain_acro_T highgain_acro_DW;

/* Zero-crossing (trigger) state */
extern PrevZCX_highgain_acro_T highgain_acro_PrevZCX;

/* Model entry point functions */
extern void highgain_acro_initialize(void);
extern void highgain_acro_step(void);
extern void highgain_acro_terminate(void);
extern const char* g_pwm_device;
extern bool g_pwm_enabled;
extern int g_pwm_fd;

/* Exported data declaration */

/* Declaration of data with custom storage class MTD */
extern real32_T MC_ROLLRATE_D;         /* Referenced by: '<S1>/Constant3' */
extern real32_T MC_ROLLRATE_I;         /* Referenced by: '<S1>/Constant2' */
extern real32_T MC_ROLLRATE_P;         /* Referenced by: '<S1>/Constant1' */
extern real32_T MC_ROLL_P;             /* Referenced by:
                                        * '<S1>/Constant'
                                        * '<S1>/k1'
                                        */

/* Real-time Model object */
extern RT_MODEL_highgain_acro_T *const highgain_acro_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S8>/Data Type Duplicate' : Unused code path elimination
 * Block '<S7>/Gain' : Eliminated nontunable gain of 1
 * Block '<S1>/Conversion17' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion30' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion31' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion32' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion33' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion34' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion35' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion36' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion37' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion38' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion39' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion40' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion41' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion42' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion43' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion45' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion46' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion47' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion48' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion49' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion50' : Eliminate redundant data type conversion
 * Block '<S1>/Conversion51' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'highgain_acro'
 * '<S1>'   : 'highgain_acro/Control System'
 * '<S2>'   : 'highgain_acro/RGB_Mode_Subsystem'
 * '<S3>'   : 'highgain_acro/quat2eul'
 * '<S4>'   : 'highgain_acro/Control System/Control System1'
 * '<S5>'   : 'highgain_acro/Control System/InputConditioning'
 * '<S6>'   : 'highgain_acro/Control System/LED '
 * '<S7>'   : 'highgain_acro/Control System/Control System1/Subsystem2'
 * '<S8>'   : 'highgain_acro/Control System/Control System1/Subsystem2/Discrete Derivative'
 * '<S9>'   : 'highgain_acro/Control System/Control System1/Subsystem2/MATLAB Function'
 * '<S10>'  : 'highgain_acro/Control System/Control System1/Subsystem2/MATLAB Function2'
 * '<S11>'  : 'highgain_acro/Control System/Control System1/Subsystem2/MATLAB Function3'
 * '<S12>'  : 'highgain_acro/Control System/Control System1/Subsystem2/MATLAB Function4'
 * '<S13>'  : 'highgain_acro/Control System/Control System1/Subsystem2/MATLAB Function5'
 * '<S14>'  : 'highgain_acro/Control System/Control System1/Subsystem2/MATLAB Function6'
 * '<S15>'  : 'highgain_acro/Control System/Control System1/Subsystem2/MATLAB Function7'
 * '<S16>'  : 'highgain_acro/Control System/Control System1/Subsystem2/Subsystem'
 * '<S17>'  : 'highgain_acro/Control System/Control System1/Subsystem2/Subsystem1'
 * '<S18>'  : 'highgain_acro/Control System/Control System1/Subsystem2/reset_i'
 * '<S19>'  : 'highgain_acro/Control System/Control System1/Subsystem2/reset_i1'
 * '<S20>'  : 'highgain_acro/Control System/Control System1/Subsystem2/reset_i/Compare To Constant3'
 * '<S21>'  : 'highgain_acro/Control System/Control System1/Subsystem2/reset_i1/Compare To Constant3'
 * '<S22>'  : 'highgain_acro/Control System/InputConditioning/deadzone0'
 * '<S23>'  : 'highgain_acro/Control System/InputConditioning/deadzone1'
 * '<S24>'  : 'highgain_acro/Control System/InputConditioning/deadzone2'
 * '<S25>'  : 'highgain_acro/Control System/InputConditioning/deadzone3'
 * '<S26>'  : 'highgain_acro/Control System/InputConditioning/f1'
 * '<S27>'  : 'highgain_acro/Control System/LED /Compare To Constant3'
 */
#endif                                 /* RTW_HEADER_highgain_acro_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
