/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: highgain_acro.c
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

#include "highgain_acro.h"
#include "highgain_acro_private.h"

const char *g_pwm_device = PWM_OUTPUT0_DEVICE_PATH;
int g_pwm_fd = -1;
bool g_pwm_enabled = false;

/* advertise the mixed control outputs */
actuator_outputs_s_t g_outputs = { };

orb_advert_t g_outputs_pub;
actuator_armed_s_t g_armed = { };

orb_advert_t g_armed_pub;

/* Exported data definition */

/* Definition of data with custom storage class MTD */
real32_T MC_ROLLRATE_D = 1.0F;

/* Referenced by: '<S1>/Constant3' */
real32_T MC_ROLLRATE_I = 1.0F;

/* Referenced by: '<S1>/Constant2' */
real32_T MC_ROLLRATE_P = 1.0F;

/* Referenced by: '<S1>/Constant1' */
real32_T MC_ROLL_P = 1.0F;

/* Referenced by:
 * '<S1>/Constant'
 * '<S1>/k1'
 */

/* Block signals (default storage) */
B_highgain_acro_T highgain_acro_B;

/* Block states (default storage) */
DW_highgain_acro_T highgain_acro_DW;

/* Previous zero-crossings (trigger) states */
PrevZCX_highgain_acro_T highgain_acro_PrevZCX;

/* Real-time model */
static RT_MODEL_highgain_acro_T highgain_acro_M_;
RT_MODEL_highgain_acro_T *const highgain_acro_M = &highgain_acro_M_;
static void rate_scheduler(void);

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (highgain_acro_M->Timing.TaskCounters.TID[1])++;
  if ((highgain_acro_M->Timing.TaskCounters.TID[1]) > 3) {/* Sample time: [0.004s, 0.0s] */
    highgain_acro_M->Timing.TaskCounters.TID[1] = 0;
  }

  (highgain_acro_M->Timing.TaskCounters.TID[2])++;
  if ((highgain_acro_M->Timing.TaskCounters.TID[2]) > 999) {/* Sample time: [1.0s, 0.0s] */
    highgain_acro_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/*
 * Output and update for atomic system:
 *    '<S5>/deadzone0'
 *    '<S5>/deadzone1'
 *    '<S5>/deadzone2'
 *    '<S5>/deadzone3'
 */
void highgain_acro_deadzone0(real_T rtu_u, B_deadzone0_highgain_acro_T *localB)
{
  real_T u;
  u = rtu_u;

  /* Function description: */
  /*   Control the range of the remote control input from RCMin to RCMax, and set */
  /*   a dead zone with a dead zone size of ㊣deadZone,the output y is */
  /*   normalized to 0~1.You can get the RC calibration parameters in QGC. */
  /* MATLAB Function 'Control System/InputConditioning/deadzone0': '<S22>:1' */
  /* '<S22>:1:6' RCMin = 1065; */
  /* '<S22>:1:7' RCMax = 1932; */
  /* '<S22>:1:8' RCMid = (RCMin + RCMax)/2; */
  /* '<S22>:1:9' deadZoneRate = 0.05; */
  /* '<S22>:1:10' deadZone = deadZoneRate*(RCMax - RCMin); */
  /* '<S22>:1:11' k = 1/(RCMax - RCMid - deadZone); */
  /* Limiting */
  /* '<S22>:1:13' if(u < RCMin) */
  if (rtu_u < 1065.0) {
    /* '<S22>:1:14' u = RCMin; */
    u = 1065.0;
  } else if (rtu_u > 1932.0) {
    /* '<S22>:1:15' elseif(u > RCMax) */
    /* '<S22>:1:16' u = RCMax; */
    u = 1932.0;
  }

  /* dead zone and normalize */
  /* '<S22>:1:19' if(u > RCMid + deadZone) */
  if (u > 1541.85) {
    /* '<S22>:1:20' y = (u-RCMid - deadZone)*k; */
    localB->y = ((u - 1498.5) - 43.35) * 0.0025631167499679613;
  } else if (u < 1455.15) {
    /* '<S22>:1:21' elseif(u < RCMid - deadZone) */
    /* '<S22>:1:22' y = (u - RCMid + deadZone)*k; */
    localB->y = ((u - 1498.5) + 43.35) * 0.0025631167499679613;
  } else {
    /* '<S22>:1:23' else */
    /* '<S22>:1:24' y = 0; */
    localB->y = 0.0;
  }
}

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  int32_T u0_0;
  int32_T u1_0;
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0F) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = (real32_T)atan2((real32_T)u0_0, (real32_T)u1_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void highgain_acro_step(void)
{
  int32_T c2_tmp;
  int32_T i;
  int32_T temp_tmp;
  int32_T tmp;
  real32_T cSq;
  real32_T dSq;
  real32_T dcm02;
  real32_T dcm02_tmp;
  real32_T dcm12;
  real32_T dcm12_tmp;
  real32_T rtb_phi;
  real32_T rtb_theta;
  static const int8_T b[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  static const int8_T b_0[12] = { -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1 };

  static const int8_T c_b[12] = { -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, -1, -1 };

  if (highgain_acro_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (sfun_px4_input_rc): '<Root>/input_rc'
     *
     * Block description for '<Root>/input_rc':
     *  RC Input Block
     *
     *  This block provides user input control to the model.
     *  It uses the input_rc uORB topic.
     *
     *  The user has the ability to choose which channels are available as outputs from this block and also some optional outputs. These include
     *  Channels 1 through 18
     *  double data type indicating the PWM value from the controller
     *  measured pulse widths for each of the supported channels
     *  Channel Count
     *  uint32 data type of the number of channels which are detector by the PX4
     *  RC Failsafe
     *  boolean data types indicating that the RC Tx is sending the FailSafe signal (if equipped and properly setup)
     *  explicit failsafe flag: true on TX failure or TX out of range , false otherwise.
     *  Only the true state is reliable, as there are some (PPM) receivers on the market going into failsafe without telling us explicitly.
     *  RC Input Source
     *  Enumeration data type indicating which source the RC input is from.
     *  Valid values are found in the ENUM file: RC_INPUT_SOURCE_ENUM.m
     *            Enumeration members for class 'RC_INPUT_SOURCE_ENUM':
     *            RCINPUT_SOURCE_UNKNOWN         (0)
     *            RCINPUT_SOURCE_PX4FMU_PPM      (1)
     *            RCINPUT_SOURCE_PX4IO_PPM       (2)
     *            RCINPUT_SOURCE_PX4IO_SPEKTRUM  (3)
     *            RCINPUT_SOURCE_PX4IO_SBUS      (4)
     *            RCINPUT_SOURCE_PX4IO_ST24      (5)
     *            RCINPUT_SOURCE_MAVLINK         (6)
     *            RCINPUT_SOURCE_QURT            (7)
     *            RCINPUT_SOURCE_PX4FMU_SPEKTRUM (8)
     *            RCINPUT_SOURCE_PX4FMU_SBUS     (9)
     *            RCINPUT_SOURCE_PX4FMU_ST24     (10)
     *            RCINPUT_SOURCE_PX4FMU_SUMD     (11)
     *            RCINPUT_SOURCE_PX4FMU_DSM      (12)
     *            RCINPUT_SOURCE_PX4IO_SUMD      (13)
     *
     *  RSSI - Receive signal strength index
     *  receive signal strength indicator (RSSI): < 0: Undefined, 0: no signal, 255: full reception
     *  RC Lost Connection
     *  boolean data type indicating RC receiver connection status
     *  True, if no frame has arrived in the expected time, false otherwise.
     *  True usally means that the receiver has been disconnected, but can also indicate a radio link loss on "stupid" systems.
     *  Will remain false, if a RX with failsafe option continues to transmit frames after a link loss.
     *
     *  Sample Model: px4demo_input_rc.slx
     */
    {
      bool updated;
      orb_check(highgain_acro_DW.input_rc_input_rc_fd.fd, &updated);
      if (updated) {
        struct rc_input_values pwm_inputs;

        /* copy input_rc raw data into local buffer (uint16)*/
        orb_copy(ORB_ID(input_rc), highgain_acro_DW.input_rc_input_rc_fd.fd,
                 &pwm_inputs);
        highgain_acro_B.input_rc_o1 = pwm_inputs.values[0];
        highgain_acro_B.input_rc_o2 = pwm_inputs.values[1];
        highgain_acro_B.input_rc_o3 = pwm_inputs.values[2];
        highgain_acro_B.input_rc_o4 = pwm_inputs.values[3];
        highgain_acro_B.input_rc_o5 = pwm_inputs.values[4];
      }
    }

    /* RelationalOperator: '<S27>/Compare' incorporates:
     *  Constant: '<S27>/Constant'
     *  DataTypeConversion: '<S1>/Conversion6'
     */
    highgain_acro_B.Compare = (highgain_acro_B.input_rc_o5 >= 1500);

    /* MATLAB Function: '<S5>/deadzone2' incorporates:
     *  DataTypeConversion: '<S1>/Conversion4'
     */
    highgain_acro_deadzone0((real_T)highgain_acro_B.input_rc_o3,
      &highgain_acro_B.sf_deadzone2);

    /* Gain: '<S5>/Gain' incorporates:
     *  Constant: '<S5>/Constant'
     *  Sum: '<S5>/Sum'
     */
    highgain_acro_B.Gain = (highgain_acro_B.sf_deadzone2.y + 1.0) * 0.5;

    /* MATLAB Function: '<S5>/f1' incorporates:
     *  Constant: '<S5>/thr_hover '
     */
    /* Function description: */
    /*   After this function, when the throttle is in the middle position, the output is hovering throttle hover */
    /* Input: */
    /*   in: Throttle normalized value, range (0~1) */
    /*   hover: Hovering throttle value */
    /* Output: */
    /*   Processed throttle value */
    /* MATLAB Function 'Control System/InputConditioning/f1': '<S26>:1' */
    /* '<S26>:1:9' if in < 0.5 */
    if (highgain_acro_B.Gain < 0.5) {
      /* '<S26>:1:10' y=2 * in * hover; */
      highgain_acro_B.Gain = 2.0 * highgain_acro_B.Gain * 0.609;
    } else {
      /* '<S26>:1:11' else */
      /* '<S26>:1:12' y=hover + 2 * (in - 0.5) * (1.0 - hover); */
      highgain_acro_B.Gain = (highgain_acro_B.Gain - 0.5) * 2.0 * 0.391 + 0.609;
    }

    /* End of MATLAB Function: '<S5>/f1' */

    /* Saturate: '<S5>/Saturation10' */
    if (highgain_acro_B.Gain > 1.0) {
      /* Saturate: '<S5>/Saturation10' */
      highgain_acro_B.Saturation10 = 1.0;
    } else if (highgain_acro_B.Gain < 0.0) {
      /* Saturate: '<S5>/Saturation10' */
      highgain_acro_B.Saturation10 = 0.0;
    } else {
      /* Saturate: '<S5>/Saturation10' */
      highgain_acro_B.Saturation10 = highgain_acro_B.Gain;
    }

    /* End of Saturate: '<S5>/Saturation10' */

    /* MATLAB Function: '<S5>/deadzone0' incorporates:
     *  DataTypeConversion: '<S1>/Conversion2'
     */
    highgain_acro_deadzone0((real_T)highgain_acro_B.input_rc_o1,
      &highgain_acro_B.sf_deadzone0);

    /* Saturate: '<S5>/Saturation9' */
    if (highgain_acro_B.sf_deadzone0.y > 1.0) {
      highgain_acro_B.rtb_u0filter3_b = 1.0;
    } else if (highgain_acro_B.sf_deadzone0.y < -1.0) {
      highgain_acro_B.rtb_u0filter3_b = -1.0;
    } else {
      highgain_acro_B.rtb_u0filter3_b = highgain_acro_B.sf_deadzone0.y;
    }

    /* End of Saturate: '<S5>/Saturation9' */

    /* Gain: '<S5>/deg2rad1' incorporates:
     *  Gain: '<S5>/Gain1'
     */
    highgain_acro_B.Gain = 220.0 * highgain_acro_B.rtb_u0filter3_b * 0.0174533;

    /* MATLAB Function: '<S5>/deadzone1' incorporates:
     *  DataTypeConversion: '<S1>/Conversion3'
     */
    highgain_acro_deadzone0((real_T)highgain_acro_B.input_rc_o2,
      &highgain_acro_B.sf_deadzone1);

    /* Saturate: '<S5>/Saturation8' */
    if (highgain_acro_B.sf_deadzone1.y > 1.0) {
      highgain_acro_B.rtb_u0filter3_b = 1.0;
    } else if (highgain_acro_B.sf_deadzone1.y < -1.0) {
      highgain_acro_B.rtb_u0filter3_b = -1.0;
    } else {
      highgain_acro_B.rtb_u0filter3_b = highgain_acro_B.sf_deadzone1.y;
    }

    /* End of Saturate: '<S5>/Saturation8' */

    /* Gain: '<S5>/deg2rad2' incorporates:
     *  Gain: '<S5>/Gain3'
     */
    highgain_acro_B.deg2rad2 = 220.0 * highgain_acro_B.rtb_u0filter3_b *
      0.0174533;

    /* MATLAB Function: '<S5>/deadzone3' incorporates:
     *  DataTypeConversion: '<S1>/Conversion5'
     */
    highgain_acro_deadzone0((real_T)highgain_acro_B.input_rc_o4,
      &highgain_acro_B.sf_deadzone3);

    /* Saturate: '<S5>/Saturation7' */
    if (highgain_acro_B.sf_deadzone3.y > 1.0) {
      highgain_acro_B.rtb_u0filter3_b = 1.0;
    } else if (highgain_acro_B.sf_deadzone3.y < -1.0) {
      highgain_acro_B.rtb_u0filter3_b = -1.0;
    } else {
      highgain_acro_B.rtb_u0filter3_b = highgain_acro_B.sf_deadzone3.y;
    }

    /* End of Saturate: '<S5>/Saturation7' */

    /* Gain: '<S5>/deg2rad3' incorporates:
     *  Gain: '<S5>/Gain2'
     */
    highgain_acro_B.deg2rad3 = 200.0 * highgain_acro_B.rtb_u0filter3_b *
      0.0174533;

    /* S-Function (sfun_px4_sensor_combined): '<Root>/sensor_combined'
     *
     * Block description for '<Root>/sensor_combined':
     *  Sensor Combined Block
     *
     *  This block enables access to the various sensors available on the px4fmu-v2 hardware.
     *  The user can use these signals in the Simulink control model.
     *  The sample time needs to be provided in the mask dialog.
     *  Optional output ports can also be selected.
     *  Refer to the sample model: px4demo_attitude_control.slx
     *
     *  Signal definitions:
     *  Magnetometer (x,y,z) - single values 每 Magnetic field in NED body frame, in Gauss
     *  Accelerometer (x,y,z) - single values 每 Acceleration in NED body frame, in m/s^2
     *  Gyroscope (p,q,r) - single values 每 Angular velocity in radians per second
     *  Barometer (Altitude) - single value 每 Barometric pressure, already temperature compensated (millibars)
     *  RunTime (timestamp) - double value 每 Timestamp in microseconds since boot, from gyro
     *
     *  The sensor_combined block needs to have the px4io service running on the PX4 hardware in order to get valid signal values.

     */
    {
      bool updated_sensor = false;
      double acc_tm= 0;
      double gyro_tm= 0;

      /* obtained data sensor combined */
      struct sensor_combined_s raw;
      orb_check(highgain_acro_DW.sensor_combined_sensor_fd.fd, &updated_sensor);
      if (updated_sensor) {
        /* copy sensors raw data into local buffer */
        orb_copy(ORB_ID(sensor_combined),
                 highgain_acro_DW.sensor_combined_sensor_fd.fd, &raw);
        gyro_tm = (double)raw.timestamp;
        acc_tm = (double)raw.accelerometer_timestamp_relative + gyro_tm;
      }

      /* read out the gyro X,Y,Z */
      if (updated_sensor) {
        highgain_acro_B.sensor_combined_o1 = (float)raw.gyro_rad[0];
        highgain_acro_B.sensor_combined_o2 = (float)raw.gyro_rad[1];
        highgain_acro_B.sensor_combined_o3 = (float)raw.gyro_rad[2];
      }
    }

    /* DataTypeConversion: '<S1>/Conversion8' */
    highgain_acro_B.Conversion8 = highgain_acro_B.sensor_combined_o1;

    /* DataTypeConversion: '<S1>/Conversion9' */
    highgain_acro_B.Conversion9 = highgain_acro_B.sensor_combined_o2;

    /* DataTypeConversion: '<S1>/Conversion10' */
    highgain_acro_B.Conversion10 = highgain_acro_B.sensor_combined_o3;

    /* Sum: '<S7>/Sum' incorporates:
     *  DataTypeConversion: '<S1>/Conversion10'
     *  DataTypeConversion: '<S1>/Conversion8'
     *  DataTypeConversion: '<S1>/Conversion9'
     */
    highgain_acro_B.Sum[0] = highgain_acro_B.sensor_combined_o1 -
      highgain_acro_B.Gain;
    highgain_acro_B.Sum[1] = highgain_acro_B.sensor_combined_o2 -
      highgain_acro_B.deg2rad2;
    highgain_acro_B.Sum[2] = highgain_acro_B.sensor_combined_o3 -
      highgain_acro_B.deg2rad3;

    /* MATLAB Function: '<S7>/MATLAB Function5' incorporates:
     *  Constant: '<S1>/k1'
     *  Constant: '<S1>/kM'
     *  Constant: '<S7>/motor'
     *  DataTypeConversion: '<S1>/Conversion44'
     */
    /* MATLAB Function 'Control System/Control System1/Subsystem2/MATLAB Function5': '<S13>:1' */
    /* '<S13>:1:2' A = a_motor * eye(4); */
    /* '<S13>:1:3' M_inv = kM*[-1,  1,  1; */
    /* '<S13>:1:4'           1, -1,  1; */
    /* '<S13>:1:5'           1,  1, -1; */
    /* '<S13>:1:6'          -1, -1, -1]; */
    for (i = 0; i < 12; i++) {
      highgain_acro_B.M_inv[i] = 0.01 * (real_T)b_0[i];
    }

    /* '<S13>:1:7' M = kM*0.25 * [-1,  1,  1, -1; */
    /* '<S13>:1:8'              1, -1,  1, -1; */
    /* '<S13>:1:9'              1,  1, -1, -1]; */
    /* '<S13>:1:10' r = 10 * r; */
    /*   test temp */
    /* '<S13>:1:11' epislo = [p;q;r]; */
    /* '<S13>:1:12' c2 = (A*k1*M_inv + k1*k1*M_inv - M')*epislo; */
    highgain_acro_B.u0filter_tmp = (real_T)MC_ROLL_P * MC_ROLL_P;
    highgain_acro_B.dv3[0] = highgain_acro_B.Sum[0];
    highgain_acro_B.dv3[1] = highgain_acro_B.Sum[1];
    highgain_acro_B.dv3[2] = 10.0 * highgain_acro_B.Sum[2];
    for (i = 0; i < 4; i++) {
      highgain_acro_B.c2[i] = 0.0;
      for (temp_tmp = 0; temp_tmp < 3; temp_tmp++) {
        c2_tmp = temp_tmp << 2;
        highgain_acro_B.c2[i] += ((((((real_T)b[i + 4] * -50.0 * MC_ROLL_P *
          highgain_acro_B.M_inv[c2_tmp + 1] + -50.0 * (real_T)b[i] * MC_ROLL_P *
          highgain_acro_B.M_inv[c2_tmp]) + (real_T)b[i + 8] * -50.0 * MC_ROLL_P *
          highgain_acro_B.M_inv[c2_tmp + 2]) + (real_T)b[i + 12] * -50.0 *
          MC_ROLL_P * highgain_acro_B.M_inv[c2_tmp + 3]) +
          highgain_acro_B.M_inv[c2_tmp + i] * highgain_acro_B.u0filter_tmp) -
          (real_T)c_b[3 * i + temp_tmp] * 0.0025) * highgain_acro_B.dv3[temp_tmp];
      }
    }

    /* End of MATLAB Function: '<S7>/MATLAB Function5' */

    /* MATLAB Function: '<S7>/MATLAB Function2' incorporates:
     *  MATLAB Function: '<S7>/MATLAB Function6'
     */
    /* MATLAB Function 'Control System/Control System1/Subsystem2/MATLAB Function2': '<S10>:1' */
    /* '<S10>:1:2' I_xx = 0.006; */
    /* '<S10>:1:2' I_yy = 0.006; */
    /* '<S10>:1:2' I_zz = 0.002; */
    /* '<S10>:1:3' L = [q * r * (I_yy - I_zz); */
    /* '<S10>:1:4'      p * r * (I_zz - I_xx); */
    /* '<S10>:1:5'      p * q * (I_xx - I_yy)]; */
    highgain_acro_B.rtb_TSamp_idx_0_tmp = highgain_acro_B.Sum[1] *
      highgain_acro_B.Sum[2] * 0.004;

    /* SampleTimeMath: '<S8>/TSamp' incorporates:
     *  MATLAB Function: '<S7>/MATLAB Function2'
     *
     * About '<S8>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    highgain_acro_B.u0filter_tmp = highgain_acro_B.rtb_TSamp_idx_0_tmp * 250.0;

    /* MATLAB Function: '<S7>/MATLAB Function2' incorporates:
     *  MATLAB Function: '<S7>/MATLAB Function6'
     */
    highgain_acro_B.u0filter = highgain_acro_B.Sum[0] * highgain_acro_B.Sum[2] *
      -0.004;

    /* SampleTimeMath: '<S8>/TSamp' incorporates:
     *  MATLAB Function: '<S7>/MATLAB Function2'
     *
     * About '<S8>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    highgain_acro_B.rtb_TSamp_idx_1 = highgain_acro_B.u0filter * 250.0;

    /* MATLAB Function: '<S7>/MATLAB Function2' incorporates:
     *  MATLAB Function: '<S7>/MATLAB Function6'
     */
    highgain_acro_B.rtb_TSamp_idx_2_tmp = highgain_acro_B.Sum[0] *
      highgain_acro_B.Sum[1] * 0.0;

    /* SampleTimeMath: '<S8>/TSamp' incorporates:
     *  MATLAB Function: '<S7>/MATLAB Function2'
     *
     * About '<S8>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    highgain_acro_B.rtb_TSamp_idx_2 = highgain_acro_B.rtb_TSamp_idx_2_tmp *
      250.0;

    /* MATLAB Function: '<S7>/MATLAB Function6' incorporates:
     *  Constant: '<S1>/kM'
     *  Constant: '<S7>/motor'
     */
    /* MATLAB Function 'Control System/Control System1/Subsystem2/MATLAB Function6': '<S14>:1' */
    /* '<S14>:1:2' A = a_motor * eye(4); */
    /* '<S14>:1:3' M_inv = kM*[-1,  1,  1; */
    /* '<S14>:1:4'           1, -1,  1; */
    /* '<S14>:1:5'           1,  1, -1; */
    /* '<S14>:1:6'          -1, -1, -1]; */
    for (i = 0; i < 12; i++) {
      highgain_acro_B.M_inv[i] = 0.01 * (real_T)b_0[i];
    }

    /* '<S14>:1:7' I_xx = 0.006; */
    /* '<S14>:1:7' I_yy = 0.006; */
    /* '<S14>:1:7' I_zz = 0.002; */
    /* '<S14>:1:8' L = [q * r * (I_yy - I_zz); */
    /* '<S14>:1:9'      p * r * (I_zz - I_xx); */
    /* '<S14>:1:10'      p * q * (I_xx - I_yy)]; */
    /*  need m dot */
    /* '<S14>:1:12' c3=A*M_inv*L - M_inv*L_dot; */
    for (i = 0; i < 3; i++) {
      for (temp_tmp = 0; temp_tmp < 4; temp_tmp++) {
        c2_tmp = i << 2;
        tmp = temp_tmp + c2_tmp;
        highgain_acro_B.dv1[tmp] = 0.0;
        highgain_acro_B.dv1[tmp] += -50.0 * (real_T)b[temp_tmp] *
          highgain_acro_B.M_inv[c2_tmp];
        highgain_acro_B.dv1[tmp] += (real_T)b[temp_tmp + 4] * -50.0 *
          highgain_acro_B.M_inv[c2_tmp + 1];
        highgain_acro_B.dv1[tmp] += (real_T)b[temp_tmp + 8] * -50.0 *
          highgain_acro_B.M_inv[c2_tmp + 2];
        highgain_acro_B.dv1[tmp] += (real_T)b[temp_tmp + 12] * -50.0 *
          highgain_acro_B.M_inv[c2_tmp + 3];
      }
    }

    /* Sum: '<S8>/Diff' incorporates:
     *  UnitDelay: '<S8>/UD'
     *
     * Block description for '<S8>/Diff':
     *
     *  Add in CPU
     *
     * Block description for '<S8>/UD':
     *
     *  Store in Global RAM
     */
    highgain_acro_B.u0filter1 = highgain_acro_B.u0filter_tmp -
      highgain_acro_DW.UD_DSTATE[0];
    highgain_acro_B.rtb_TSamp_idx_1_m = highgain_acro_B.rtb_TSamp_idx_1 -
      highgain_acro_DW.UD_DSTATE[1];
    highgain_acro_B.u0filter2 = highgain_acro_B.rtb_TSamp_idx_2 -
      highgain_acro_DW.UD_DSTATE[2];

    /* MATLAB Function: '<S7>/MATLAB Function6' */
    for (i = 0; i < 4; i++) {
      highgain_acro_B.c3[i] = ((highgain_acro_B.dv1[i + 4] *
        highgain_acro_B.u0filter + highgain_acro_B.dv1[i] *
        highgain_acro_B.rtb_TSamp_idx_0_tmp) + highgain_acro_B.dv1[i + 8] *
        highgain_acro_B.rtb_TSamp_idx_2_tmp) - ((highgain_acro_B.M_inv[i + 4] *
        highgain_acro_B.rtb_TSamp_idx_1_m + highgain_acro_B.M_inv[i] *
        highgain_acro_B.u0filter1) + highgain_acro_B.M_inv[i + 8] *
        highgain_acro_B.u0filter2);
    }
  }

  /* MATLAB Function: '<S7>/MATLAB Function4' incorporates:
   *  Constant: '<S1>/k1'
   *  Constant: '<S1>/k2'
   *  Constant: '<S1>/kM'
   *  Constant: '<S7>/motor'
   *  DataTypeConversion: '<S1>/Conversion44'
   *  Delay: '<S7>/Delay2'
   */
  /* MATLAB Function 'Control System/Control System1/Subsystem2/MATLAB Function4': '<S12>:1' */
  /* '<S12>:1:2' A = a_motor * eye(4); */
  /* '<S12>:1:3' I = eye(4); */
  /* '<S12>:1:4' M_inv = kM*[-1,  1,  1; */
  /* '<S12>:1:5'           1, -1,  1; */
  /* '<S12>:1:6'           1,  1, -1; */
  /* '<S12>:1:7'          -1, -1, -1]; */
  /* '<S12>:1:8' I_xx = 0.006; */
  /* '<S12>:1:8' I_yy = 0.006; */
  /* '<S12>:1:8' I_zz = 0.002; */
  /* '<S12>:1:9' L = [q * r * (I_yy - I_zz); */
  /* '<S12>:1:10'      p * r * (I_zz - I_xx); */
  /* '<S12>:1:11'      p * q * (I_xx - I_yy)]; */
  /* '<S12>:1:12' x = [p;q;r]; */
  /* '<S12>:1:13' fan = -M_inv*(L+k1*x); */
  /* '<S12>:1:14' enta = f - fan; */
  /* '<S12>:1:15' c1 = -(A + k1*I + k2*I)*enta; */
  /* MATLAB Function 'Control System/Control System1/Subsystem2/MATLAB Function7': '<S15>:1' */
  /* '<S15>:1:2' b = -a_motor; */
  /* '<S15>:1:3' temp = kc1*c1 + kc2*c2 + kc3*c3; */
  /* '<S15>:1:4' temp = (1/b) * temp; */
  for (i = 0; i < 16; i++) {
    highgain_acro_B.temp_tmp[i] = b[i];
  }

  for (i = 0; i < 12; i++) {
    highgain_acro_B.dv1[i] = -(0.01 * (real_T)b_0[i]);
  }

  highgain_acro_B.rtb_TSamp_idx_0_tmp = highgain_acro_B.Sum[1] *
    highgain_acro_B.Sum[2] * 0.004 + MC_ROLL_P * highgain_acro_B.Sum[0];
  highgain_acro_B.u0filter = highgain_acro_B.Sum[0] * highgain_acro_B.Sum[2] *
    -0.004 + MC_ROLL_P * highgain_acro_B.Sum[1];
  highgain_acro_B.rtb_TSamp_idx_2_tmp = highgain_acro_B.Sum[0] *
    highgain_acro_B.Sum[1] * 0.0 + MC_ROLL_P * highgain_acro_B.Sum[2];
  for (i = 0; i < 16; i++) {
    temp_tmp = highgain_acro_B.temp_tmp[i];
    highgain_acro_B.dv[i] = -((-50.0 * (real_T)temp_tmp + MC_ROLL_P * (real_T)
      temp_tmp) + 0.01 * (real_T)temp_tmp);
  }

  for (i = 0; i < 4; i++) {
    highgain_acro_B.dv2[i] = highgain_acro_DW.Delay2_DSTATE[i] -
      ((highgain_acro_B.dv1[i + 4] * highgain_acro_B.u0filter +
        highgain_acro_B.dv1[i] * highgain_acro_B.rtb_TSamp_idx_0_tmp) +
       highgain_acro_B.dv1[i + 8] * highgain_acro_B.rtb_TSamp_idx_2_tmp);
  }

  for (i = 0; i < 4; i++) {
    /* MATLAB Function: '<S7>/MATLAB Function7' incorporates:
     *  Constant: '<S1>/kc1'
     */
    highgain_acro_B.temp[i] = (((((highgain_acro_B.dv[i + 4] *
      highgain_acro_B.dv2[1] + highgain_acro_B.dv[i] * highgain_acro_B.dv2[0]) +
      highgain_acro_B.dv[i + 8] * highgain_acro_B.dv2[2]) + highgain_acro_B.dv[i
      + 12] * highgain_acro_B.dv2[3]) * 0.1 + highgain_acro_B.c2[i]) +
      highgain_acro_B.c3[i]) * 0.02;
  }

  /* End of MATLAB Function: '<S7>/MATLAB Function4' */
  /* '<S15>:1:5' fd1 = temp(1); */
  /* '<S15>:1:6' fd2 = temp(2); */
  /* '<S15>:1:7' fd3 = temp(3); */
  /* '<S15>:1:8' fd4 = temp(4); */
  if (highgain_acro_M->Timing.TaskCounters.TID[1] == 0) {
    /* RelationalOperator: '<S21>/Compare' incorporates:
     *  Constant: '<S21>/Constant'
     */
    highgain_acro_B.Compare_e = (highgain_acro_B.Saturation10 <= 0.1);
  }

  /* DiscreteFilter: '<S7>/30filter' incorporates:
   *  MATLAB Function: '<S7>/MATLAB Function7'
   */
  if ((((highgain_acro_PrevZCX.u0filter_Reset_ZCE == POS_ZCSIG) != (int32_T)
        highgain_acro_B.Compare_e) && (highgain_acro_PrevZCX.u0filter_Reset_ZCE
        != UNINITIALIZED_ZCSIG)) || highgain_acro_B.Compare_e) {
    highgain_acro_DW.u0filter_states[0] = 0.0;
    highgain_acro_DW.u0filter_states[1] = 0.0;
    highgain_acro_DW.u0filter_states[2] = 0.0;
  }

  highgain_acro_PrevZCX.u0filter_Reset_ZCE = highgain_acro_B.Compare_e;
  highgain_acro_B.rtb_TSamp_idx_0_tmp = ((highgain_acro_B.temp[0] - -2.0651 *
    highgain_acro_DW.u0filter_states[0]) - 1.52 *
    highgain_acro_DW.u0filter_states[1]) - -0.3861 *
    highgain_acro_DW.u0filter_states[2];
  highgain_acro_B.u0filter = ((0.0086 * highgain_acro_B.rtb_TSamp_idx_0_tmp +
    0.0258 * highgain_acro_DW.u0filter_states[0]) + 0.0258 *
    highgain_acro_DW.u0filter_states[1]) + 0.0086 *
    highgain_acro_DW.u0filter_states[2];

  /* DiscreteFilter: '<S7>/30filter1' incorporates:
   *  DiscreteFilter: '<S7>/30filter'
   *  MATLAB Function: '<S7>/MATLAB Function7'
   */
  if ((((highgain_acro_PrevZCX.u0filter1_Reset_ZCE == POS_ZCSIG) != (int32_T)
        highgain_acro_B.Compare_e) && (highgain_acro_PrevZCX.u0filter1_Reset_ZCE
        != UNINITIALIZED_ZCSIG)) || highgain_acro_B.Compare_e) {
    highgain_acro_DW.u0filter1_states[0] = 0.0;
    highgain_acro_DW.u0filter1_states[1] = 0.0;
    highgain_acro_DW.u0filter1_states[2] = 0.0;
  }

  highgain_acro_PrevZCX.u0filter1_Reset_ZCE = highgain_acro_B.Compare_e;
  highgain_acro_B.rtb_TSamp_idx_2_tmp = ((highgain_acro_B.temp[1] - -2.0651 *
    highgain_acro_DW.u0filter1_states[0]) - 1.52 *
    highgain_acro_DW.u0filter1_states[1]) - -0.3861 *
    highgain_acro_DW.u0filter1_states[2];
  highgain_acro_B.u0filter1 = ((0.0086 * highgain_acro_B.rtb_TSamp_idx_2_tmp +
    0.0258 * highgain_acro_DW.u0filter1_states[0]) + 0.0258 *
    highgain_acro_DW.u0filter1_states[1]) + 0.0086 *
    highgain_acro_DW.u0filter1_states[2];

  /* DiscreteFilter: '<S7>/30filter2' incorporates:
   *  DiscreteFilter: '<S7>/30filter'
   *  MATLAB Function: '<S7>/MATLAB Function7'
   */
  if ((((highgain_acro_PrevZCX.u0filter2_Reset_ZCE == POS_ZCSIG) != (int32_T)
        highgain_acro_B.Compare_e) && (highgain_acro_PrevZCX.u0filter2_Reset_ZCE
        != UNINITIALIZED_ZCSIG)) || highgain_acro_B.Compare_e) {
    highgain_acro_DW.u0filter2_states[0] = 0.0;
    highgain_acro_DW.u0filter2_states[1] = 0.0;
    highgain_acro_DW.u0filter2_states[2] = 0.0;
  }

  highgain_acro_PrevZCX.u0filter2_Reset_ZCE = highgain_acro_B.Compare_e;
  highgain_acro_B.rtb_TSamp_idx_1_m = ((highgain_acro_B.temp[2] - -2.0651 *
    highgain_acro_DW.u0filter2_states[0]) - 1.52 *
    highgain_acro_DW.u0filter2_states[1]) - -0.3861 *
    highgain_acro_DW.u0filter2_states[2];
  highgain_acro_B.u0filter2 = ((0.0086 * highgain_acro_B.rtb_TSamp_idx_1_m +
    0.0258 * highgain_acro_DW.u0filter2_states[0]) + 0.0258 *
    highgain_acro_DW.u0filter2_states[1]) + 0.0086 *
    highgain_acro_DW.u0filter2_states[2];

  /* DiscreteFilter: '<S7>/30filter3' incorporates:
   *  DiscreteFilter: '<S7>/30filter'
   *  MATLAB Function: '<S7>/MATLAB Function7'
   */
  if ((((highgain_acro_PrevZCX.u0filter3_Reset_ZCE == POS_ZCSIG) != (int32_T)
        highgain_acro_B.Compare_e) && (highgain_acro_PrevZCX.u0filter3_Reset_ZCE
        != UNINITIALIZED_ZCSIG)) || highgain_acro_B.Compare_e) {
    highgain_acro_DW.u0filter3_states[0] = 0.0;
    highgain_acro_DW.u0filter3_states[1] = 0.0;
    highgain_acro_DW.u0filter3_states[2] = 0.0;
  }

  highgain_acro_PrevZCX.u0filter3_Reset_ZCE = highgain_acro_B.Compare_e;
  highgain_acro_B.a = ((highgain_acro_B.temp[3] - -2.0651 *
                        highgain_acro_DW.u0filter3_states[0]) - 1.52 *
                       highgain_acro_DW.u0filter3_states[1]) - -0.3861 *
    highgain_acro_DW.u0filter3_states[2];
  highgain_acro_B.u0filter3 = ((0.0086 * highgain_acro_B.a + 0.0258 *
    highgain_acro_DW.u0filter3_states[0]) + 0.0258 *
    highgain_acro_DW.u0filter3_states[1]) + 0.0086 *
    highgain_acro_DW.u0filter3_states[2];

  /* Saturate: '<S7>/Output_Limits2' */
  /* MATLAB Function 'Control System/Control System1/Subsystem2/MATLAB Function': '<S9>:1' */
  /* '<S9>:1:2' idle_PWM = 1100 */
  /* '<S9>:1:3' scale = 1800 - idle_PWM; */
  /* '<S9>:1:4' speed1 = (f1 + thrust ) * scale + idle_PWM; */
  /* '<S9>:1:5' speed2 = (f2 + thrust ) * scale + idle_PWM; */
  /* '<S9>:1:6' speed3 = (f3 + thrust ) * scale + idle_PWM; */
  /* '<S9>:1:7' speed4 = (f4 + thrust ) * scale + idle_PWM; */
  if (highgain_acro_B.u0filter > 1.0) {
    highgain_acro_B.rtb_u0filter_c = 1.0;
  } else if (highgain_acro_B.u0filter < -1.0) {
    highgain_acro_B.rtb_u0filter_c = -1.0;
  } else {
    highgain_acro_B.rtb_u0filter_c = highgain_acro_B.u0filter;
  }

  /* End of Saturate: '<S7>/Output_Limits2' */

  /* MATLAB Function: '<S7>/MATLAB Function' */
  highgain_acro_B.rtb_u0filter_c = (highgain_acro_B.rtb_u0filter_c +
    highgain_acro_B.Saturation10) * 700.0 + 1100.0;

  /* Saturate: '<S7>/Output_Limits1' */
  if (highgain_acro_B.rtb_u0filter_c > 1800.0) {
    highgain_acro_B.rtb_u0filter_c = 1800.0;
  } else if (highgain_acro_B.rtb_u0filter_c < 1100.0) {
    highgain_acro_B.rtb_u0filter_c = 1100.0;
  }

  /* Saturate: '<S7>/Output_Limits3' */
  if (highgain_acro_B.u0filter1 > 1.0) {
    highgain_acro_B.rtb_u0filter1_k = 1.0;
  } else if (highgain_acro_B.u0filter1 < -1.0) {
    highgain_acro_B.rtb_u0filter1_k = -1.0;
  } else {
    highgain_acro_B.rtb_u0filter1_k = highgain_acro_B.u0filter1;
  }

  /* End of Saturate: '<S7>/Output_Limits3' */

  /* MATLAB Function: '<S7>/MATLAB Function' */
  highgain_acro_B.rtb_u0filter1_k = (highgain_acro_B.rtb_u0filter1_k +
    highgain_acro_B.Saturation10) * 700.0 + 1100.0;

  /* Saturate: '<S7>/Output_Limits1' */
  if (highgain_acro_B.rtb_u0filter1_k > 1800.0) {
    highgain_acro_B.rtb_u0filter1_k = 1800.0;
  } else if (highgain_acro_B.rtb_u0filter1_k < 1100.0) {
    highgain_acro_B.rtb_u0filter1_k = 1100.0;
  }

  /* Saturate: '<S7>/Output_Limits4' */
  if (highgain_acro_B.u0filter2 > 1.0) {
    highgain_acro_B.rtb_u0filter2_c = 1.0;
  } else if (highgain_acro_B.u0filter2 < -1.0) {
    highgain_acro_B.rtb_u0filter2_c = -1.0;
  } else {
    highgain_acro_B.rtb_u0filter2_c = highgain_acro_B.u0filter2;
  }

  /* End of Saturate: '<S7>/Output_Limits4' */

  /* MATLAB Function: '<S7>/MATLAB Function' */
  highgain_acro_B.rtb_u0filter2_c = (highgain_acro_B.rtb_u0filter2_c +
    highgain_acro_B.Saturation10) * 700.0 + 1100.0;

  /* Saturate: '<S7>/Output_Limits1' */
  if (highgain_acro_B.rtb_u0filter2_c > 1800.0) {
    highgain_acro_B.rtb_u0filter2_c = 1800.0;
  } else if (highgain_acro_B.rtb_u0filter2_c < 1100.0) {
    highgain_acro_B.rtb_u0filter2_c = 1100.0;
  }

  /* Saturate: '<S7>/Output_Limits5' */
  if (highgain_acro_B.u0filter3 > 1.0) {
    highgain_acro_B.rtb_u0filter3_b = 1.0;
  } else if (highgain_acro_B.u0filter3 < -1.0) {
    highgain_acro_B.rtb_u0filter3_b = -1.0;
  } else {
    highgain_acro_B.rtb_u0filter3_b = highgain_acro_B.u0filter3;
  }

  /* End of Saturate: '<S7>/Output_Limits5' */

  /* MATLAB Function: '<S7>/MATLAB Function' */
  highgain_acro_B.u0 = (highgain_acro_B.rtb_u0filter3_b +
                        highgain_acro_B.Saturation10) * 700.0 + 1100.0;

  /* Saturate: '<S7>/Output_Limits1' */
  if (highgain_acro_B.u0 > 1800.0) {
    highgain_acro_B.u0 = 1800.0;
  } else if (highgain_acro_B.u0 < 1100.0) {
    highgain_acro_B.u0 = 1100.0;
  }

  /* DataTypeConversion: '<Root>/Data Type Conversion2' incorporates:
   *  Saturate: '<S7>/Output_Limits1'
   */
  highgain_acro_B.rtb_u0filter3_b = floor(highgain_acro_B.rtb_u0filter_c);
  if (rtIsNaN(highgain_acro_B.rtb_u0filter3_b)) {
    /* DataTypeConversion: '<Root>/Data Type Conversion2' */
    highgain_acro_B.DataTypeConversion2[0] = 0U;
  } else {
    /* DataTypeConversion: '<Root>/Data Type Conversion2' */
    highgain_acro_B.DataTypeConversion2[0] = (uint16_T)fmod
      (highgain_acro_B.rtb_u0filter3_b, 65536.0);
  }

  highgain_acro_B.rtb_u0filter3_b = floor(highgain_acro_B.rtb_u0filter1_k);
  if (rtIsNaN(highgain_acro_B.rtb_u0filter3_b)) {
    /* DataTypeConversion: '<Root>/Data Type Conversion2' */
    highgain_acro_B.DataTypeConversion2[1] = 0U;
  } else {
    /* DataTypeConversion: '<Root>/Data Type Conversion2' */
    highgain_acro_B.DataTypeConversion2[1] = (uint16_T)fmod
      (highgain_acro_B.rtb_u0filter3_b, 65536.0);
  }

  highgain_acro_B.rtb_u0filter3_b = floor(highgain_acro_B.rtb_u0filter2_c);
  if (rtIsNaN(highgain_acro_B.rtb_u0filter3_b)) {
    /* DataTypeConversion: '<Root>/Data Type Conversion2' */
    highgain_acro_B.DataTypeConversion2[2] = 0U;
  } else {
    /* DataTypeConversion: '<Root>/Data Type Conversion2' */
    highgain_acro_B.DataTypeConversion2[2] = (uint16_T)fmod
      (highgain_acro_B.rtb_u0filter3_b, 65536.0);
  }

  highgain_acro_B.rtb_u0filter3_b = floor(highgain_acro_B.u0);
  if (rtIsNaN(highgain_acro_B.rtb_u0filter3_b)) {
    /* DataTypeConversion: '<Root>/Data Type Conversion2' */
    highgain_acro_B.DataTypeConversion2[3] = 0U;
  } else {
    /* DataTypeConversion: '<Root>/Data Type Conversion2' */
    highgain_acro_B.DataTypeConversion2[3] = (uint16_T)fmod
      (highgain_acro_B.rtb_u0filter3_b, 65536.0);
  }

  /* End of DataTypeConversion: '<Root>/Data Type Conversion2' */

  /* S-Function (sfun_px4_pwm): '<Root>/PWM_output'
   *
   * Block description for '<Root>/PWM_output':
   *  This block allows the user to send the appropriate PWM signals out to the PX4 outputs.
   *  These are usually connected to the ESCs which control the motor speeds.
   *  In order for the flight control to arm (enable) the output from the software side, the ARM Output input must be held high (boolean TRUE).
   *  Only then will the PWM values be sent out the PX4 hardware ports.
   *  This is usually a function of the RC Tx in combination with other flight modes programmed in the Simulink model by the user.
   *  The block has 8 available ports (data type uint16) which can be selectively chosen.
   *  These correspond to the 8 PWM output ports on the px4fmu hardware.
   */
  if (highgain_acro_B.Compare == true) {
    if (g_pwm_enabled == false) {
      int rc;

      /* arm system */
      rc = px4_ioctl(g_pwm_fd, PWM_SERVO_ARM, 0);
      g_armed.timestamp = hrt_absolute_time();
      g_armed.armed = true;
      orb_publish(ORB_ID(actuator_armed), g_armed_pub, &g_armed);
      if (rc != OK)
        err(1,"PWM_SERVO_ARM");
      else {
        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(0), PWM_MOTOR_OFF);
        g_outputs.output[0] = PWM_MOTOR_OFF;
        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(1), PWM_MOTOR_OFF);
        g_outputs.output[1] = PWM_MOTOR_OFF;
        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(2), PWM_MOTOR_OFF);
        g_outputs.output[2] = PWM_MOTOR_OFF;
        px4_ioctl(g_pwm_fd, PWM_SERVO_SET(3), PWM_MOTOR_OFF);
        g_outputs.output[3] = PWM_MOTOR_OFF;
        g_pwm_enabled = true;
        printf("***ARMED*** PWM fd = %d\n", g_pwm_fd);
      }
    }
  } else {
    if (g_pwm_enabled == true) {
      int rc;

      /* disarm system if enabled */
      px4_ioctl(g_pwm_fd, PWM_SERVO_SET(0), PWM_MOTOR_OFF);
      g_outputs.output[0] = PWM_MOTOR_OFF;
      px4_ioctl(g_pwm_fd, PWM_SERVO_SET(1), PWM_MOTOR_OFF);
      g_outputs.output[1] = PWM_MOTOR_OFF;
      px4_ioctl(g_pwm_fd, PWM_SERVO_SET(2), PWM_MOTOR_OFF);
      g_outputs.output[2] = PWM_MOTOR_OFF;
      px4_ioctl(g_pwm_fd, PWM_SERVO_SET(3), PWM_MOTOR_OFF);
      g_outputs.output[3] = PWM_MOTOR_OFF;
      rc = px4_ioctl(g_pwm_fd, PWM_SERVO_DISARM, 0);
      g_armed.timestamp = hrt_absolute_time();
      g_armed.armed = false;
      orb_publish(ORB_ID(actuator_armed), g_armed_pub, &g_armed);
      g_pwm_enabled = false;
      if (rc != OK)
        err(1, "PWM_SERVO_DISARM");
      else
        printf("***DISARMED*** PWM fd = %d\n", g_pwm_fd);
    }
  }

  if (g_pwm_enabled) {
    /* output the PWM signals */
    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(0), (unsigned int)
              highgain_acro_B.DataTypeConversion2[0]);
    g_outputs.output[0] = (unsigned int)highgain_acro_B.DataTypeConversion2[0];
    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(1), (unsigned int)
              highgain_acro_B.DataTypeConversion2[1]);
    g_outputs.output[1] = (unsigned int)highgain_acro_B.DataTypeConversion2[1];
    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(2), (unsigned int)
              highgain_acro_B.DataTypeConversion2[2]);
    g_outputs.output[2] = (unsigned int)highgain_acro_B.DataTypeConversion2[2];
    px4_ioctl(g_pwm_fd, PWM_SERVO_SET(3), (unsigned int)
              highgain_acro_B.DataTypeConversion2[3]);
    g_outputs.output[3] = (unsigned int)highgain_acro_B.DataTypeConversion2[3];

    /* now publish to actuator_outputs in case anyone wants to know... */
    g_outputs.timestamp = hrt_absolute_time();
    orb_publish(ORB_ID(actuator_outputs), g_outputs_pub, &g_outputs);
  }

  /* RateTransition: '<Root>/Rate Transition' */
  if ((highgain_acro_M->Timing.TaskCounters.TID[1] == 0) &&
      (highgain_acro_M->Timing.TaskCounters.TID[2] == 0)) {
    highgain_acro_DW.RateTransition_Buffer = highgain_acro_B.Compare;
  }

  if (highgain_acro_M->Timing.TaskCounters.TID[2] == 0) {
    /* Switch: '<S2>/Switch' incorporates:
     *  Switch: '<S2>/Switch1'
     */
    if (highgain_acro_DW.RateTransition_Buffer) {
      /* Switch: '<S2>/Switch' incorporates:
       *  Constant: '<S2>/Constant'
       */
      highgain_acro_B.Switch = SL_MODE_BLINK_FAST;

      /* Switch: '<S2>/Switch1' incorporates:
       *  Constant: '<S2>/Constant3'
       */
      highgain_acro_B.Switch1 = SL_COLOR_RED;
    } else {
      /* Switch: '<S2>/Switch' incorporates:
       *  Constant: '<S2>/Constant1'
       */
      highgain_acro_B.Switch = SL_MODE_BREATHE;

      /* Switch: '<S2>/Switch1' incorporates:
       *  Constant: '<S2>/Constant2'
       */
      highgain_acro_B.Switch1 = SL_COLOR_GREEN;
    }

    /* End of Switch: '<S2>/Switch' */

    /* S-Function (sfun_px4_rgbled): '<Root>/RGB_LED' */
    highgain_acro_DW.RGB_LED_sl_led_control_s.mode = highgain_acro_B.Switch;
    highgain_acro_DW.RGB_LED_sl_led_control_s.color = highgain_acro_B.Switch1;
    highgain_acro_DW.RGB_LED_sl_led_control_s.num_blinks = 0;
    highgain_acro_DW.RGB_LED_sl_led_control_s.priority = 0;
    highgain_acro_DW.RGB_LED_sl_led_control_s.timestamp = hrt_absolute_time();
    orb_publish(ORB_ID(led_control), highgain_acro_DW.RGB_LED_orb_advert_t ,
                &highgain_acro_DW.RGB_LED_sl_led_control_s);
  }

  /* End of RateTransition: '<Root>/Rate Transition' */

  /* S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced' */
  {
    //struct jim_attctrl_s InputStruct;
    //memset( &InputStruct, 0, sizeof(InputStruct));

    /* assign input parameters to struct */
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.timestamp =
      highgain_acro_B.Conversion19;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.roll =
      highgain_acro_B.Conversion23;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.pitch =
      highgain_acro_B.Conversion24;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.yaw =
      highgain_acro_B.Conversion25;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.roll_rate =
      highgain_acro_B.Conversion20;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.pitch_rate =
      highgain_acro_B.Conversion21;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.yaw_rate =
      highgain_acro_B.Conversion22;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.roll_d =
      highgain_acro_B.Conversion12;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.pitch_d =
      highgain_acro_B.Conversion13;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.yaw_d = 0.0F;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.roll_rate_d =
      highgain_acro_B.Conversion15;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.pitch_rate_d =
      highgain_acro_B.Conversion16;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.yawrate_d =
      highgain_acro_B.Conversion14;
    highgain_acro_DW.uORBWriteAdvanced_uorb_msg.thrust_d =
      highgain_acro_B.Conversion18;

    /* Publish data for subscribers */
    orb_publish(ORB_ID(jim_attctrl),
                highgain_acro_DW.uORBWriteAdvanced_uorb_advert,
                &highgain_acro_DW.uORBWriteAdvanced_uorb_msg);
  }

  /* S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced1' */
  {
    //struct jim_actuator_s InputStruct;
    //memset( &InputStruct, 0, sizeof(InputStruct));

    /* assign input parameters to struct */
    highgain_acro_DW.uORBWriteAdvanced1_uorb_msg.timestamp =
      highgain_acro_B.Conversion19;
    highgain_acro_DW.uORBWriteAdvanced1_uorb_msg.pwm_1 =
      highgain_acro_B.Conversion26;
    highgain_acro_DW.uORBWriteAdvanced1_uorb_msg.pwm_2 =
      highgain_acro_B.Conversion27;
    highgain_acro_DW.uORBWriteAdvanced1_uorb_msg.pwm_3 =
      highgain_acro_B.Conversion28;
    highgain_acro_DW.uORBWriteAdvanced1_uorb_msg.pwm_4 =
      highgain_acro_B.Conversion29;

    /* Publish data for subscribers */
    orb_publish(ORB_ID(jim_actuator),
                highgain_acro_DW.uORBWriteAdvanced1_uorb_advert,
                &highgain_acro_DW.uORBWriteAdvanced1_uorb_msg);
  }

  /* S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced2' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   *  Constant: '<S1>/Constant2'
   *  Constant: '<S1>/Constant3'
   */
  {
    //struct jim_gain_s InputStruct;
    //memset( &InputStruct, 0, sizeof(InputStruct));

    /* assign input parameters to struct */
    highgain_acro_DW.uORBWriteAdvanced2_uorb_msg.timestamp =
      highgain_acro_B.Conversion19;
    highgain_acro_DW.uORBWriteAdvanced2_uorb_msg.mc_roll_p_k1 = MC_ROLL_P;
    highgain_acro_DW.uORBWriteAdvanced2_uorb_msg.attrate_p = MC_ROLLRATE_P;
    highgain_acro_DW.uORBWriteAdvanced2_uorb_msg.attrate_i = MC_ROLLRATE_I;
    highgain_acro_DW.uORBWriteAdvanced2_uorb_msg.attrate_d = MC_ROLLRATE_D;

    /* Publish data for subscribers */
    orb_publish(ORB_ID(jim_gain),
                highgain_acro_DW.uORBWriteAdvanced2_uorb_advert,
                &highgain_acro_DW.uORBWriteAdvanced2_uorb_msg);
  }

  /* S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced3' */
  {
    //struct jim_pid_out_s InputStruct;
    //memset( &InputStruct, 0, sizeof(InputStruct));

    /* assign input parameters to struct */
    highgain_acro_DW.uORBWriteAdvanced3_uorb_msg.timestamp =
      highgain_acro_B.Conversion19;
    highgain_acro_DW.uORBWriteAdvanced3_uorb_msg.roll_p_out = 0.0F;
    highgain_acro_DW.uORBWriteAdvanced3_uorb_msg.roll_i_out = 0.0F;
    highgain_acro_DW.uORBWriteAdvanced3_uorb_msg.roll_d_out = 0.0F;
    highgain_acro_DW.uORBWriteAdvanced3_uorb_msg.pitch_p_out = 0.0F;
    highgain_acro_DW.uORBWriteAdvanced3_uorb_msg.pitch_i_out = 0.0F;
    highgain_acro_DW.uORBWriteAdvanced3_uorb_msg.pitch_d_out = 0.0F;

    /* Publish data for subscribers */
    orb_publish(ORB_ID(jim_pid_out),
                highgain_acro_DW.uORBWriteAdvanced3_uorb_advert,
                &highgain_acro_DW.uORBWriteAdvanced3_uorb_msg);
  }

  if (highgain_acro_M->Timing.TaskCounters.TID[1] == 0) {
    /* RelationalOperator: '<S20>/Compare' incorporates:
     *  Constant: '<S20>/Constant'
     */
    highgain_acro_B.Compare_f = (highgain_acro_B.Saturation10 <= 0.2);
  }

  /* DiscreteIntegrator: '<S16>/Discrete-Time Integrator' */
  if (highgain_acro_B.Compare_f ||
      (highgain_acro_DW.DiscreteTimeIntegrator_PrevRese != 0)) {
    highgain_acro_DW.DiscreteTimeIntegrator_DSTATE = 0.0;
  }

  /* DiscreteIntegrator: '<S16>/Discrete-Time Integrator1' */
  if (highgain_acro_B.Compare_f ||
      (highgain_acro_DW.DiscreteTimeIntegrator1_PrevRes != 0)) {
    highgain_acro_DW.DiscreteTimeIntegrator1_DSTATE = 0.0;
  }

  /* DiscreteIntegrator: '<S16>/Discrete-Time Integrator2' */
  if (highgain_acro_B.Compare_f ||
      (highgain_acro_DW.DiscreteTimeIntegrator2_PrevRes != 0)) {
    highgain_acro_DW.DiscreteTimeIntegrator2_DSTATE = 0.0;
  }

  /* DiscreteIntegrator: '<S17>/Discrete-Time Integrator' */
  if (highgain_acro_B.Compare_f ||
      (highgain_acro_DW.DiscreteTimeIntegrator_PrevRe_f != 0)) {
    highgain_acro_DW.DiscreteTimeIntegrator_DSTATE_m = 0.0;
  }

  /* DiscreteIntegrator: '<S17>/Discrete-Time Integrator1' */
  if (highgain_acro_B.Compare_f ||
      (highgain_acro_DW.DiscreteTimeIntegrator1_PrevR_c != 0)) {
    highgain_acro_DW.DiscreteTimeIntegrator1_DSTAT_f = 0.0;
  }

  /* DiscreteIntegrator: '<S17>/Discrete-Time Integrator2' */
  if (highgain_acro_B.Compare_f ||
      (highgain_acro_DW.DiscreteTimeIntegrator2_PrevR_o != 0)) {
    highgain_acro_DW.DiscreteTimeIntegrator2_DSTAT_j = 0.0;
  }

  /* DiscreteIntegrator: '<S17>/Discrete-Time Integrator3' */
  if (highgain_acro_B.Compare_f ||
      (highgain_acro_DW.DiscreteTimeIntegrator3_PrevRes != 0)) {
    highgain_acro_DW.DiscreteTimeIntegrator3_DSTATE = 0.0;
  }

  /* SignalConversion generated from: '<S11>/ SFunction ' incorporates:
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator1'
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator2'
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator3'
   *  MATLAB Function: '<S7>/MATLAB Function3'
   */
  highgain_acro_B.TmpSignalConversionAtSFun_k[0] =
    highgain_acro_DW.DiscreteTimeIntegrator_DSTATE_m;
  highgain_acro_B.TmpSignalConversionAtSFun_k[1] =
    highgain_acro_DW.DiscreteTimeIntegrator1_DSTAT_f;
  highgain_acro_B.TmpSignalConversionAtSFun_k[2] =
    highgain_acro_DW.DiscreteTimeIntegrator2_DSTAT_j;
  highgain_acro_B.TmpSignalConversionAtSFun_k[3] =
    highgain_acro_DW.DiscreteTimeIntegrator3_DSTATE;

  /* MATLAB Function: '<S7>/MATLAB Function3' incorporates:
   *  Constant: '<S1>/h1'
   *  Constant: '<S1>/h2'
   *  Constant: '<S1>/kM'
   *  Constant: '<S7>/motor'
   *  DiscreteIntegrator: '<S16>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S16>/Discrete-Time Integrator1'
   *  DiscreteIntegrator: '<S16>/Discrete-Time Integrator2'
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator1'
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator2'
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator3'
   *  SignalConversion generated from: '<S11>/ SFunction '
   */
  /* MATLAB Function 'Control System/Control System1/Subsystem2/MATLAB Function3': '<S11>:1' */
  /* '<S11>:1:2' I_xx = 0.006; */
  /* '<S11>:1:2' I_yy = 0.006; */
  /* '<S11>:1:2' I_zz = 0.002; */
  /* '<S11>:1:3' L = [(1/I_xx) * (q * r * (I_yy - I_zz) ); */
  /* '<S11>:1:4'      (1/I_yy) * (p * r * (I_zz - I_xx) ); */
  /* '<S11>:1:5'      (1/I_zz) * (p * q * (I_xx - I_yy) )]; */
  /* '<S11>:1:6' M_inv = kM*[-1,  1,  1; */
  /* '<S11>:1:7'           1, -1,  1; */
  /* '<S11>:1:8'           1,  1, -1; */
  /* '<S11>:1:9'          -1, -1, -1]; */
  /* '<S11>:1:10' M = kM*0.25 * [-1,  1,  1, -1; */
  /* '<S11>:1:11'              1, -1,  1, -1; */
  /* '<S11>:1:12'              1,  1, -1, -1]; */
  /* '<S11>:1:13' a = a_motor; */
  /* '<S11>:1:14' b = -a; */
  /* '<S11>:1:15' y = [p;q;r]; */
  /* '<S11>:1:16' x_hat_dot = L       + M*z_hat   + h1*(y-x_hat); */
  highgain_acro_B.b_tmp[0] = highgain_acro_B.Sum[0] -
    highgain_acro_DW.DiscreteTimeIntegrator_DSTATE;
  highgain_acro_B.b_tmp[1] = highgain_acro_B.Sum[1] -
    highgain_acro_DW.DiscreteTimeIntegrator1_DSTATE;
  highgain_acro_B.b_tmp[2] = highgain_acro_B.Sum[2] -
    highgain_acro_DW.DiscreteTimeIntegrator2_DSTATE;
  highgain_acro_B.dv3[0] = highgain_acro_B.Sum[1] * highgain_acro_B.Sum[2] *
    0.004 * 166.66666666666666;
  highgain_acro_B.dv3[1] = highgain_acro_B.Sum[0] * highgain_acro_B.Sum[2] *
    -0.004 * 166.66666666666666;
  highgain_acro_B.dv3[2] = highgain_acro_B.Sum[0] * highgain_acro_B.Sum[1] * 0.0
    * 500.0;
  for (i = 0; i < 3; i++) {
    highgain_acro_B.TmpSignalConversionAtSFun_f[i] = (((((real_T)c_b[i + 3] *
      0.0025 * highgain_acro_DW.DiscreteTimeIntegrator1_DSTAT_f + 0.0025 *
      (real_T)c_b[i] * highgain_acro_DW.DiscreteTimeIntegrator_DSTATE_m) +
      (real_T)c_b[i + 6] * 0.0025 *
      highgain_acro_DW.DiscreteTimeIntegrator2_DSTAT_j) + (real_T)c_b[i + 9] *
      0.0025 * highgain_acro_DW.DiscreteTimeIntegrator3_DSTATE) +
      highgain_acro_B.dv3[i]) + 100.0 * highgain_acro_B.b_tmp[i];
  }

  /* '<S11>:1:17' z_hat_dot = a*z_hat + b*U       + h2*M_inv*(y-x_hat); */
  highgain_acro_B.dv2[0] = 50.0 * highgain_acro_B.u0filter;
  highgain_acro_B.dv2[1] = 50.0 * highgain_acro_B.u0filter1;
  highgain_acro_B.dv2[2] = 50.0 * highgain_acro_B.u0filter2;
  highgain_acro_B.dv2[3] = 50.0 * highgain_acro_B.u0filter3;
  for (i = 0; i < 4; i++) {
    highgain_acro_B.temp[i] = (((real_T)b_0[i + 4] * 0.01 * 100.0 *
      highgain_acro_B.b_tmp[1] + 0.01 * (real_T)b_0[i] * 100.0 *
      highgain_acro_B.b_tmp[0]) + (real_T)b_0[i + 8] * 0.01 * 100.0 *
      highgain_acro_B.b_tmp[2]) + (-50.0 *
      highgain_acro_B.TmpSignalConversionAtSFun_k[i] + highgain_acro_B.dv2[i]);
  }

  /* '<S11>:1:18' f1_hat = z_hat(1); */
  /* '<S11>:1:19' f2_hat = z_hat(2); */
  /* '<S11>:1:20' f3_hat = z_hat(3); */
  /* '<S11>:1:21' f4_hat = z_hat(4); */
  if (highgain_acro_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (sfun_px4_vehicle_attitude): '<Root>/vehicle_attitude'
     *
     * Block description for '<Root>/vehicle_attitude':
     *  This block gives access to the running service that calculates the vehicle＊s attitude.  A uORB topic (vehicle_attitude (attitude measurements)) publisher MUST be running in order for this block to provide valid signal values.  The available ones as of v1.3 are:
     *  attitude_estimator_ekf 每 EKF-Extended Kalman Filter for attitude estimation
     *  attitude_estimator_so3 每 SO(3)-attitude estimation by using accelerometer, gyroscopes and magnetometer
     *  One of these MUST be running on the px4fmu in order for this block to return valid values.  Refer to the sample model: px4demo_attitude_control.slx. Attitude in NED (North-East-Down) body frame in SI units.
     *  Signal definitions:
     *  Roll 每 single value, Roll angle (rad, Tait-Bryan, NED)
     *  Pitch 每 single value, Pitch angle (rad, Tait-Bryan, NED)
     *  Yaw 每 single value, Yaw angle (rad, Tait-Bryan, NED)
     *  Quaternion (NED) 每 single(4) values (optional based on the uORB publisher)
     *
     */
    {
      bool updated;
      orb_check(highgain_acro_DW.vehicle_attitude_vehicle_attitu.fd, &updated);
      if (updated) {
        struct vehicle_attitude_s raw;
        orb_copy(ORB_ID(vehicle_attitude),
                 highgain_acro_DW.vehicle_attitude_vehicle_attitu.fd, &raw);

        /* read out the Quaternion values */
        highgain_acro_B.vehicle_attitude[0] = raw.q[0];
        highgain_acro_B.vehicle_attitude[1] = raw.q[1];
        highgain_acro_B.vehicle_attitude[2] = raw.q[2];
        highgain_acro_B.vehicle_attitude[3] = raw.q[3];
      }
    }

    /* MATLAB Function: '<Root>/quat2eul' */
    /*  Conversion from Quaternion to Euler angles based on the PX4 Firmware */
    /*  v1.6.5 release. */
    /*   */
    /*  Quaternion -> DCM -> Euler Angles */
    /*  INPUT: */
    /*    q: Quaternion vector of the format: (a + bi + cj + dk) */
    /*  OUTPUT: */
    /*    [phi,theta,psi]: Euler angles in radians. */
    /*   */
    /* MATLAB Function 'quat2eul': '<S3>:1' */
    /* '<S3>:1:12' a = q(1); */
    /* '<S3>:1:13' b = q(2); */
    /* '<S3>:1:14' c = q(3); */
    /* '<S3>:1:15' d = q(4); */
    /* '<S3>:1:17' aSq = a*a; */
    highgain_acro_B.aSq = highgain_acro_B.vehicle_attitude[0] *
      highgain_acro_B.vehicle_attitude[0];

    /* '<S3>:1:18' bSq = b*b; */
    highgain_acro_B.bSq = highgain_acro_B.vehicle_attitude[1] *
      highgain_acro_B.vehicle_attitude[1];

    /* '<S3>:1:19' cSq = c*c; */
    cSq = highgain_acro_B.vehicle_attitude[2] *
      highgain_acro_B.vehicle_attitude[2];

    /* '<S3>:1:20' dSq = d*d; */
    dSq = highgain_acro_B.vehicle_attitude[3] *
      highgain_acro_B.vehicle_attitude[3];

    /* '<S3>:1:22' d_pi2 = pi/2; */
    /*  quaternion_to_dcm */
    /* '<S3>:1:25' dcm00 = aSq + bSq - cSq - dSq; */
    /*  dcm01 = 2*(b*c - a*d); */
    /* '<S3>:1:27' dcm02 = 2*(a*c + b*d); */
    rtb_theta = highgain_acro_B.vehicle_attitude[1] *
      highgain_acro_B.vehicle_attitude[3];
    dcm02_tmp = highgain_acro_B.vehicle_attitude[0] *
      highgain_acro_B.vehicle_attitude[2];
    dcm02 = (dcm02_tmp + rtb_theta) * 2.0F;

    /* '<S3>:1:28' dcm10 = 2*(b*c + a*d); */
    /*  dcm11 = aSq - bSq + cSq - dSq; */
    /* '<S3>:1:30' dcm12 = 2*(c*d - a*b); */
    rtb_phi = highgain_acro_B.vehicle_attitude[0] *
      highgain_acro_B.vehicle_attitude[1];
    dcm12_tmp = highgain_acro_B.vehicle_attitude[2] *
      highgain_acro_B.vehicle_attitude[3];
    dcm12 = (dcm12_tmp - rtb_phi) * 2.0F;

    /* '<S3>:1:31' dcm20 = 2*(b*d - a*c); */
    /* '<S3>:1:32' dcm21 = 2*(a*b + c*d); */
    /* '<S3>:1:33' dcm22 = aSq - bSq - cSq + dSq; */
    /*  dcm_to_euler */
    /* '<S3>:1:36' theta = asin(-dcm20); */
    rtb_theta = (real32_T)asin(-((rtb_theta - dcm02_tmp) * 2.0F));

    /* '<S3>:1:37' if abs(theta - d_pi2) < 1.0e-3 */
    if ((real32_T)fabs(rtb_theta - 1.57079637F) < 0.001) {
      /* '<S3>:1:38' phi = single(0.0); */
      rtb_phi = 0.0F;

      /* '<S3>:1:39' psi = atan2(dcm12, dcm02); */
      highgain_acro_B.aSq = rt_atan2f_snf(dcm12, dcm02);
    } else if ((real32_T)fabs(rtb_theta + 1.57079637F) < 0.001) {
      /* '<S3>:1:40' elseif abs(theta + d_pi2) < 1.0e-3 */
      /* '<S3>:1:41' phi = single(0.0); */
      rtb_phi = 0.0F;

      /* '<S3>:1:42' psi = atan2(-dcm12, -dcm02); */
      highgain_acro_B.aSq = rt_atan2f_snf(-dcm12, -dcm02);
    } else {
      /* '<S3>:1:43' else */
      /* '<S3>:1:44' phi = atan2(dcm21, dcm22); */
      rtb_phi = rt_atan2f_snf((rtb_phi + dcm12_tmp) * 2.0F,
        ((highgain_acro_B.aSq - highgain_acro_B.bSq) - cSq) + dSq);

      /* '<S3>:1:45' psi = atan2(dcm10, dcm00); */
      highgain_acro_B.aSq = rt_atan2f_snf((highgain_acro_B.vehicle_attitude[1] *
        highgain_acro_B.vehicle_attitude[2] + highgain_acro_B.vehicle_attitude[0]
        * highgain_acro_B.vehicle_attitude[3]) * 2.0F, ((highgain_acro_B.aSq +
        highgain_acro_B.bSq) - cSq) - dSq);
    }

    /* End of MATLAB Function: '<Root>/quat2eul' */

    /* DataTypeConversion: '<S1>/Conversion1' */
    highgain_acro_B.Conversion7 = rtb_phi;

    /* DataTypeConversion: '<S1>/Conversion11' */
    highgain_acro_B.Conversion11 = highgain_acro_B.aSq;

    /* DataTypeConversion: '<S1>/Conversion12' */
    highgain_acro_B.Conversion12 = (real32_T)highgain_acro_B.Gain;

    /* DataTypeConversion: '<S1>/Conversion13' */
    highgain_acro_B.Conversion13 = (real32_T)highgain_acro_B.deg2rad2;

    /* DataTypeConversion: '<S1>/Conversion14' */
    highgain_acro_B.Conversion14 = (real32_T)highgain_acro_B.deg2rad3;

    /* DataTypeConversion: '<S1>/Conversion15' incorporates:
     *  DataTypeConversion: '<S1>/Conversion12'
     */
    highgain_acro_B.Conversion15 = (real32_T)highgain_acro_B.Gain;

    /* DataTypeConversion: '<S1>/Conversion16' incorporates:
     *  DataTypeConversion: '<S1>/Conversion13'
     */
    highgain_acro_B.Conversion16 = (real32_T)highgain_acro_B.deg2rad2;

    /* DataTypeConversion: '<S1>/Conversion18' */
    highgain_acro_B.Conversion18 = (real32_T)highgain_acro_B.Saturation10;
  }

  /* DataTypeConversion: '<S1>/Conversion19' incorporates:
   *  DigitalClock: '<S1>/Digital Clock'
   *  Gain: '<S1>/Gain'
   */
  highgain_acro_B.rtb_u0filter3_b = floor(1.0E+6 *
    ((highgain_acro_M->Timing.clockTick0) * 0.001));
  if (rtIsNaN(highgain_acro_B.rtb_u0filter3_b) || rtIsInf
      (highgain_acro_B.rtb_u0filter3_b)) {
    highgain_acro_B.rtb_u0filter3_b = 0.0;
  } else {
    highgain_acro_B.rtb_u0filter3_b = fmod(highgain_acro_B.rtb_u0filter3_b,
      4.294967296E+9);
  }

  /* DataTypeConversion: '<S1>/Conversion19' */
  highgain_acro_B.Conversion19 = highgain_acro_B.rtb_u0filter3_b < 0.0 ?
    (uint32_T)-(int32_T)(uint32_T)-highgain_acro_B.rtb_u0filter3_b : (uint32_T)
    highgain_acro_B.rtb_u0filter3_b;
  if (highgain_acro_M->Timing.TaskCounters.TID[1] == 0) {
    /* DataTypeConversion: '<S1>/Conversion20' */
    highgain_acro_B.Conversion20 = (real32_T)highgain_acro_B.Conversion8;

    /* DataTypeConversion: '<S1>/Conversion21' */
    highgain_acro_B.Conversion21 = (real32_T)highgain_acro_B.Conversion9;

    /* DataTypeConversion: '<S1>/Conversion22' */
    highgain_acro_B.Conversion22 = (real32_T)highgain_acro_B.Conversion10;

    /* DataTypeConversion: '<S1>/Conversion23' */
    highgain_acro_B.Conversion23 = (real32_T)highgain_acro_B.Conversion7;

    /* DataTypeConversion: '<S1>/Conversion24' incorporates:
     *  DataTypeConversion: '<S1>/Conversion7'
     */
    highgain_acro_B.Conversion24 = rtb_theta;

    /* DataTypeConversion: '<S1>/Conversion25' */
    highgain_acro_B.Conversion25 = (real32_T)highgain_acro_B.Conversion11;

    /* Update for UnitDelay: '<S8>/UD'
     *
     * Block description for '<S8>/UD':
     *
     *  Store in Global RAM
     */
    highgain_acro_DW.UD_DSTATE[0] = highgain_acro_B.u0filter_tmp;
    highgain_acro_DW.UD_DSTATE[1] = highgain_acro_B.rtb_TSamp_idx_1;
    highgain_acro_DW.UD_DSTATE[2] = highgain_acro_B.rtb_TSamp_idx_2;
  }

  /* DataTypeConversion: '<S1>/Conversion26' incorporates:
   *  Saturate: '<S7>/Output_Limits1'
   */
  highgain_acro_B.Conversion26 = (real32_T)highgain_acro_B.rtb_u0filter_c;

  /* DataTypeConversion: '<S1>/Conversion27' incorporates:
   *  Saturate: '<S7>/Output_Limits1'
   */
  highgain_acro_B.Conversion27 = (real32_T)highgain_acro_B.rtb_u0filter1_k;

  /* DataTypeConversion: '<S1>/Conversion28' incorporates:
   *  Saturate: '<S7>/Output_Limits1'
   */
  highgain_acro_B.Conversion28 = (real32_T)highgain_acro_B.rtb_u0filter2_c;

  /* DataTypeConversion: '<S1>/Conversion29' incorporates:
   *  Saturate: '<S7>/Output_Limits1'
   */
  highgain_acro_B.Conversion29 = (real32_T)highgain_acro_B.u0;

  /* Update for Delay: '<S7>/Delay2' incorporates:
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator1'
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator2'
   *  DiscreteIntegrator: '<S17>/Discrete-Time Integrator3'
   */
  highgain_acro_DW.Delay2_DSTATE[0] =
    highgain_acro_DW.DiscreteTimeIntegrator_DSTATE_m;
  highgain_acro_DW.Delay2_DSTATE[1] =
    highgain_acro_DW.DiscreteTimeIntegrator1_DSTAT_f;
  highgain_acro_DW.Delay2_DSTATE[2] =
    highgain_acro_DW.DiscreteTimeIntegrator2_DSTAT_j;
  highgain_acro_DW.Delay2_DSTATE[3] =
    highgain_acro_DW.DiscreteTimeIntegrator3_DSTATE;

  /* Update for DiscreteFilter: '<S7>/30filter' */
  highgain_acro_DW.u0filter_states[2] = highgain_acro_DW.u0filter_states[1];
  highgain_acro_DW.u0filter_states[1] = highgain_acro_DW.u0filter_states[0];
  highgain_acro_DW.u0filter_states[0] = highgain_acro_B.rtb_TSamp_idx_0_tmp;

  /* Update for DiscreteFilter: '<S7>/30filter1' */
  highgain_acro_DW.u0filter1_states[2] = highgain_acro_DW.u0filter1_states[1];
  highgain_acro_DW.u0filter1_states[1] = highgain_acro_DW.u0filter1_states[0];
  highgain_acro_DW.u0filter1_states[0] = highgain_acro_B.rtb_TSamp_idx_2_tmp;

  /* Update for DiscreteFilter: '<S7>/30filter2' */
  highgain_acro_DW.u0filter2_states[2] = highgain_acro_DW.u0filter2_states[1];
  highgain_acro_DW.u0filter2_states[1] = highgain_acro_DW.u0filter2_states[0];
  highgain_acro_DW.u0filter2_states[0] = highgain_acro_B.rtb_TSamp_idx_1_m;

  /* Update for DiscreteFilter: '<S7>/30filter3' */
  highgain_acro_DW.u0filter3_states[2] = highgain_acro_DW.u0filter3_states[1];
  highgain_acro_DW.u0filter3_states[1] = highgain_acro_DW.u0filter3_states[0];
  highgain_acro_DW.u0filter3_states[0] = highgain_acro_B.a;

  /* Update for DiscreteIntegrator: '<S16>/Discrete-Time Integrator' */
  highgain_acro_DW.DiscreteTimeIntegrator_DSTATE += 0.001 *
    highgain_acro_B.TmpSignalConversionAtSFun_f[0];
  highgain_acro_DW.DiscreteTimeIntegrator_PrevRese = (int8_T)
    highgain_acro_B.Compare_f;

  /* Update for DiscreteIntegrator: '<S16>/Discrete-Time Integrator1' incorporates:
   *  DiscreteIntegrator: '<S16>/Discrete-Time Integrator'
   */
  highgain_acro_DW.DiscreteTimeIntegrator1_DSTATE += 0.001 *
    highgain_acro_B.TmpSignalConversionAtSFun_f[1];
  highgain_acro_DW.DiscreteTimeIntegrator1_PrevRes = (int8_T)
    highgain_acro_B.Compare_f;

  /* Update for DiscreteIntegrator: '<S16>/Discrete-Time Integrator2' incorporates:
   *  DiscreteIntegrator: '<S16>/Discrete-Time Integrator'
   */
  highgain_acro_DW.DiscreteTimeIntegrator2_DSTATE += 0.001 *
    highgain_acro_B.TmpSignalConversionAtSFun_f[2];
  highgain_acro_DW.DiscreteTimeIntegrator2_PrevRes = (int8_T)
    highgain_acro_B.Compare_f;

  /* Update for DiscreteIntegrator: '<S17>/Discrete-Time Integrator' incorporates:
   *  DiscreteIntegrator: '<S16>/Discrete-Time Integrator'
   */
  highgain_acro_DW.DiscreteTimeIntegrator_DSTATE_m += 0.001 *
    highgain_acro_B.temp[0];
  highgain_acro_DW.DiscreteTimeIntegrator_PrevRe_f = (int8_T)
    highgain_acro_B.Compare_f;

  /* Update for DiscreteIntegrator: '<S17>/Discrete-Time Integrator1' incorporates:
   *  DiscreteIntegrator: '<S16>/Discrete-Time Integrator'
   */
  highgain_acro_DW.DiscreteTimeIntegrator1_DSTAT_f += 0.001 *
    highgain_acro_B.temp[1];
  highgain_acro_DW.DiscreteTimeIntegrator1_PrevR_c = (int8_T)
    highgain_acro_B.Compare_f;

  /* Update for DiscreteIntegrator: '<S17>/Discrete-Time Integrator2' incorporates:
   *  DiscreteIntegrator: '<S16>/Discrete-Time Integrator'
   */
  highgain_acro_DW.DiscreteTimeIntegrator2_DSTAT_j += 0.001 *
    highgain_acro_B.temp[2];
  highgain_acro_DW.DiscreteTimeIntegrator2_PrevR_o = (int8_T)
    highgain_acro_B.Compare_f;

  /* Update for DiscreteIntegrator: '<S17>/Discrete-Time Integrator3' incorporates:
   *  DiscreteIntegrator: '<S16>/Discrete-Time Integrator'
   */
  highgain_acro_DW.DiscreteTimeIntegrator3_DSTATE += 0.001 *
    highgain_acro_B.temp[3];
  highgain_acro_DW.DiscreteTimeIntegrator3_PrevRes = (int8_T)
    highgain_acro_B.Compare_f;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.001, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  highgain_acro_M->Timing.clockTick0++;
  rate_scheduler();
}

/* Model initialize function */
void highgain_acro_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)highgain_acro_M, 0,
                sizeof(RT_MODEL_highgain_acro_T));

  /* block I/O */
  (void) memset(((void *) &highgain_acro_B), 0,
                sizeof(B_highgain_acro_T));

  {
    highgain_acro_B.Switch = SL_MODE_OFF;
    highgain_acro_B.Switch1 = SL_COLOR_OFF;
  }

  /* states (dwork) */
  (void) memset((void *)&highgain_acro_DW, 0,
                sizeof(DW_highgain_acro_T));

  /* Start for S-Function (sfun_px4_input_rc): '<Root>/input_rc'
   *
   * Block description for '<Root>/input_rc':
   *  RC Input Block
   *
   *  This block provides user input control to the model.
   *  It uses the input_rc uORB topic.
   *
   *  The user has the ability to choose which channels are available as outputs from this block and also some optional outputs. These include
   *  Channels 1 through 18
   *  double data type indicating the PWM value from the controller
   *  measured pulse widths for each of the supported channels
   *  Channel Count
   *  uint32 data type of the number of channels which are detector by the PX4
   *  RC Failsafe
   *  boolean data types indicating that the RC Tx is sending the FailSafe signal (if equipped and properly setup)
   *  explicit failsafe flag: true on TX failure or TX out of range , false otherwise.
   *  Only the true state is reliable, as there are some (PPM) receivers on the market going into failsafe without telling us explicitly.
   *  RC Input Source
   *  Enumeration data type indicating which source the RC input is from.
   *  Valid values are found in the ENUM file: RC_INPUT_SOURCE_ENUM.m
   *            Enumeration members for class 'RC_INPUT_SOURCE_ENUM':
   *            RCINPUT_SOURCE_UNKNOWN         (0)
   *            RCINPUT_SOURCE_PX4FMU_PPM      (1)
   *            RCINPUT_SOURCE_PX4IO_PPM       (2)
   *            RCINPUT_SOURCE_PX4IO_SPEKTRUM  (3)
   *            RCINPUT_SOURCE_PX4IO_SBUS      (4)
   *            RCINPUT_SOURCE_PX4IO_ST24      (5)
   *            RCINPUT_SOURCE_MAVLINK         (6)
   *            RCINPUT_SOURCE_QURT            (7)
   *            RCINPUT_SOURCE_PX4FMU_SPEKTRUM (8)
   *            RCINPUT_SOURCE_PX4FMU_SBUS     (9)
   *            RCINPUT_SOURCE_PX4FMU_ST24     (10)
   *            RCINPUT_SOURCE_PX4FMU_SUMD     (11)
   *            RCINPUT_SOURCE_PX4FMU_DSM      (12)
   *            RCINPUT_SOURCE_PX4IO_SUMD      (13)
   *
   *  RSSI - Receive signal strength index
   *  receive signal strength indicator (RSSI): < 0: Undefined, 0: no signal, 255: full reception
   *  RC Lost Connection
   *  boolean data type indicating RC receiver connection status
   *  True, if no frame has arrived in the expected time, false otherwise.
   *  True usally means that the receiver has been disconnected, but can also indicate a radio link loss on "stupid" systems.
   *  Will remain false, if a RX with failsafe option continues to transmit frames after a link loss.
   *
   *  Sample Model: px4demo_input_rc.slx
   */
  {
    /* S-Function Block: <Root>/input_rc */
    /* subscribe to PWM RC input topic */
    int fd = orb_subscribe(ORB_ID(input_rc));
    highgain_acro_DW.input_rc_input_rc_fd.fd = fd;
    highgain_acro_DW.input_rc_input_rc_fd.events = POLLIN;
    orb_set_interval(fd, 1);
    PX4_INFO("* Subscribed to input_rc topic (fd = %d)*\n", fd);
  }

  /* Start for S-Function (sfun_px4_sensor_combined): '<Root>/sensor_combined'
   *
   * Block description for '<Root>/sensor_combined':
   *  Sensor Combined Block
   *
   *  This block enables access to the various sensors available on the px4fmu-v2 hardware.
   *  The user can use these signals in the Simulink control model.
   *  The sample time needs to be provided in the mask dialog.
   *  Optional output ports can also be selected.
   *  Refer to the sample model: px4demo_attitude_control.slx
   *
   *  Signal definitions:
   *  Magnetometer (x,y,z) - single values 每 Magnetic field in NED body frame, in Gauss
   *  Accelerometer (x,y,z) - single values 每 Acceleration in NED body frame, in m/s^2
   *  Gyroscope (p,q,r) - single values 每 Angular velocity in radians per second
   *  Barometer (Altitude) - single value 每 Barometric pressure, already temperature compensated (millibars)
   *  RunTime (timestamp) - double value 每 Timestamp in microseconds since boot, from gyro
   *
   *  The sensor_combined block needs to have the px4io service running on the PX4 hardware in order to get valid signal values.

   */
  {
    /* S-Function Block: <Root>/sensor_combined */
    /* subscribe to sensor_combined topic */
    int fd = orb_subscribe(ORB_ID(sensor_combined));
    highgain_acro_DW.sensor_combined_sensor_fd.fd = fd;
    highgain_acro_DW.sensor_combined_sensor_fd.events = POLLIN;
    orb_set_interval(fd, 1);
    PX4_INFO("* Subscribed to sensor_combined topic (fd = %d)*\n", fd);
  }

  {
    /* S-Function Block: <Root>/sensor_combined */
    /* subscribe to vehicle_magnetometer topic */
    int fd1 = orb_subscribe(ORB_ID(vehicle_magnetometer));
    highgain_acro_DW.sensor_combined_mag_fd.fd = fd1;
    highgain_acro_DW.sensor_combined_mag_fd.events = POLLIN;
    orb_set_interval(fd1, 1);
    PX4_INFO("* Subscribed to vehicle_magnetometer topic (fd = %d)*\n", fd1);
  }

  {
    /* S-Function Block: <Root>/sensor_combined */
    /* subscribe to vehicle_air_data topic */
    int fd2 = orb_subscribe(ORB_ID(vehicle_air_data));
    highgain_acro_DW.sensor_combined_baro_fd.fd = fd2;
    highgain_acro_DW.sensor_combined_baro_fd.events = POLLIN;
    orb_set_interval(fd2, 1);
    PX4_INFO("* Subscribed to vehicle_air_data topic (fd = %d)*\n", fd2);
  }

  /* Start for S-Function (sfun_px4_pwm): '<Root>/PWM_output'
   *
   * Block description for '<Root>/PWM_output':
   *  This block allows the user to send the appropriate PWM signals out to the PX4 outputs.
   *  These are usually connected to the ESCs which control the motor speeds.
   *  In order for the flight control to arm (enable) the output from the software side, the ARM Output input must be held high (boolean TRUE).
   *  Only then will the PWM values be sent out the PX4 hardware ports.
   *  This is usually a function of the RC Tx in combination with other flight modes programmed in the Simulink model by the user.
   *  The block has 8 available ports (data type uint16) which can be selectively chosen.
   *  These correspond to the 8 PWM output ports on the px4fmu hardware.
   */
  {
    int rc;
    int pwm_rate = 400;                /* default PWM Rate is 400Hz */
    int chMask = 0x00;         /* change channel mask based on which are used */
    chMask |= 0x03;                    /* channel group 0 */
    chMask |= 0x0C;                    /* channel group 2 */

    /* enable pwm outputs, set to disarm  */
    g_pwm_fd = px4_open(g_pwm_device, 0);
    printf("OPEN PWM (%s) fd = %d\n", g_pwm_device, g_pwm_fd);
    rc = px4_ioctl(g_pwm_fd, PWM_SERVO_SET_UPDATE_RATE, pwm_rate);
    if (rc != OK) {
      printf("***Warning for PWM Module*** Set PWM_SERVO_SET_UPDATE_RATE Failed for %d Hz!\n",
             pwm_rate);
    } else {
      if (chMask > 0) {
        rc = px4_ioctl(g_pwm_fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, chMask);
        if (rc != OK) {
          printf("***Warning for PWM Module*** Set PWM_SERVO_SET_SELECT_UPDATE_RATE Failed for mask0x%08X!\n",
                 chMask);
        } else {
          printf("Set SERVO Rate (%dHz) Channel Mask:0x%08X\n", pwm_rate, chMask);
        }
      }
    }

    /* tell safety that its ok to disable it with the switch */
    rc = px4_ioctl(g_pwm_fd, PWM_SERVO_SET_ARM_OK, 0);
    if (rc != OK)
      err(1, "PWM_SERVO_SET_ARM_OK");
    rc = px4_ioctl(g_pwm_fd, PWM_SERVO_DISARM, 0);
    if (rc != OK)
      err(1, "PWM_SERVO_DISARM");

    /* advertise the mixed control outputs, insist on the first group output */
    g_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &g_outputs);
    g_armed.timestamp = hrt_absolute_time();

    /* Safely initialize armed flags */
    g_armed.armed = false;
    g_armed.prearmed = false;
    g_armed.ready_to_arm = true;
    g_armed.lockdown = false;
    g_armed.force_failsafe = false;
    g_armed.in_esc_calibration_mode = false;
    g_armed_pub = orb_advertise(ORB_ID(actuator_armed), &g_armed);
    g_pwm_enabled = false;
  }

  /* Start for S-Function (sfun_px4_rgbled): '<Root>/RGB_LED' */
  {
    // enable RGBLED, set intitial mode and color
    // more devices will be 1, 2, etc
    highgain_acro_DW.RGB_LED_sl_led_control_s.led_mask = 0xff;
    highgain_acro_DW.RGB_LED_sl_led_control_s.mode = SL_MODE_OFF;
    highgain_acro_DW.RGB_LED_sl_led_control_s.priority = 0;
    highgain_acro_DW.RGB_LED_sl_led_control_s.timestamp = hrt_absolute_time();
    highgain_acro_DW.RGB_LED_orb_advert_t = orb_advertise_queue(ORB_ID
      (led_control), &highgain_acro_DW.RGB_LED_sl_led_control_s,
      LED_UORB_QUEUE_LENGTH);
  }

  /* Start for S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced' */
  {
    /* S-Function Block: <S1>/uORB Write Advanced */
    /* Initializing topic: jim_attctrl */
    struct jim_attctrl_s initialize_topic;
    memset( &initialize_topic, 0, sizeof(initialize_topic));
    highgain_acro_DW.uORBWriteAdvanced_uorb_advert = orb_advertise(ORB_ID
      (jim_attctrl), &initialize_topic);
    if (highgain_acro_DW.uORBWriteAdvanced_uorb_advert != 0) {
      PX4_INFO("Started advertising jim_attctrl");
    }
  }

  /* Start for S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced1' */
  {
    /* S-Function Block: <S1>/uORB Write Advanced1 */
    /* Initializing topic: jim_actuator */
    struct jim_actuator_s initialize_topic;
    memset( &initialize_topic, 0, sizeof(initialize_topic));
    highgain_acro_DW.uORBWriteAdvanced1_uorb_advert = orb_advertise(ORB_ID
      (jim_actuator), &initialize_topic);
    if (highgain_acro_DW.uORBWriteAdvanced1_uorb_advert != 0) {
      PX4_INFO("Started advertising jim_actuator");
    }
  }

  /* Start for S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced2' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   *  Constant: '<S1>/Constant2'
   *  Constant: '<S1>/Constant3'
   */
  {
    /* S-Function Block: <S1>/uORB Write Advanced2 */
    /* Initializing topic: jim_gain */
    struct jim_gain_s initialize_topic;
    memset( &initialize_topic, 0, sizeof(initialize_topic));
    highgain_acro_DW.uORBWriteAdvanced2_uorb_advert = orb_advertise(ORB_ID
      (jim_gain), &initialize_topic);
    if (highgain_acro_DW.uORBWriteAdvanced2_uorb_advert != 0) {
      PX4_INFO("Started advertising jim_gain");
    }
  }

  /* Start for S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced3' */
  {
    /* S-Function Block: <S1>/uORB Write Advanced3 */
    /* Initializing topic: jim_pid_out */
    struct jim_pid_out_s initialize_topic;
    memset( &initialize_topic, 0, sizeof(initialize_topic));
    highgain_acro_DW.uORBWriteAdvanced3_uorb_advert = orb_advertise(ORB_ID
      (jim_pid_out), &initialize_topic);
    if (highgain_acro_DW.uORBWriteAdvanced3_uorb_advert != 0) {
      PX4_INFO("Started advertising jim_pid_out");
    }
  }

  /* Start for S-Function (sfun_px4_vehicle_attitude): '<Root>/vehicle_attitude'
   *
   * Block description for '<Root>/vehicle_attitude':
   *  This block gives access to the running service that calculates the vehicle＊s attitude.  A uORB topic (vehicle_attitude (attitude measurements)) publisher MUST be running in order for this block to provide valid signal values.  The available ones as of v1.3 are:
   *  attitude_estimator_ekf 每 EKF-Extended Kalman Filter for attitude estimation
   *  attitude_estimator_so3 每 SO(3)-attitude estimation by using accelerometer, gyroscopes and magnetometer
   *  One of these MUST be running on the px4fmu in order for this block to return valid values.  Refer to the sample model: px4demo_attitude_control.slx. Attitude in NED (North-East-Down) body frame in SI units.
   *  Signal definitions:
   *  Roll 每 single value, Roll angle (rad, Tait-Bryan, NED)
   *  Pitch 每 single value, Pitch angle (rad, Tait-Bryan, NED)
   *  Yaw 每 single value, Yaw angle (rad, Tait-Bryan, NED)
   *  Quaternion (NED) 每 single(4) values (optional based on the uORB publisher)
   *
   */
  {
    /* S-Function Block: <Root>/vehicle_attitude */
    /* subscribe to PWM RC input topic */
    int fd = orb_subscribe(ORB_ID(vehicle_attitude));
    highgain_acro_DW.vehicle_attitude_vehicle_attitu.fd = fd;
    highgain_acro_DW.vehicle_attitude_vehicle_attitu.events = POLLIN;
    orb_set_interval(fd, 1);
    PX4_INFO("* Subscribed to vehicle_attitude topic (fd = %d)*\n", fd);
  }

  /* user code (Start function Trailer) */
  InitParamFunction("MC_ROLLRATE_D",&MC_ROLLRATE_D);/*  Assign MC_ROLLRATE_D */
  InitParamFunction("MC_ROLLRATE_I",&MC_ROLLRATE_I);/*  Assign MC_ROLLRATE_I */
  InitParamFunction("MC_ROLLRATE_P",&MC_ROLLRATE_P);/*  Assign MC_ROLLRATE_P */
  InitParamFunction("MC_ROLL_P",&MC_ROLL_P);/*  Assign MC_ROLL_P */
  highgain_acro_PrevZCX.u0filter_Reset_ZCE = UNINITIALIZED_ZCSIG;
  highgain_acro_PrevZCX.u0filter1_Reset_ZCE = UNINITIALIZED_ZCSIG;
  highgain_acro_PrevZCX.u0filter2_Reset_ZCE = UNINITIALIZED_ZCSIG;
  highgain_acro_PrevZCX.u0filter3_Reset_ZCE = UNINITIALIZED_ZCSIG;

  /* InitializeConditions for DiscreteIntegrator: '<S16>/Discrete-Time Integrator' */
  highgain_acro_DW.DiscreteTimeIntegrator_PrevRese = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S16>/Discrete-Time Integrator1' */
  highgain_acro_DW.DiscreteTimeIntegrator1_PrevRes = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S16>/Discrete-Time Integrator2' */
  highgain_acro_DW.DiscreteTimeIntegrator2_PrevRes = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S17>/Discrete-Time Integrator' */
  highgain_acro_DW.DiscreteTimeIntegrator_PrevRe_f = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S17>/Discrete-Time Integrator1' */
  highgain_acro_DW.DiscreteTimeIntegrator1_PrevR_c = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S17>/Discrete-Time Integrator2' */
  highgain_acro_DW.DiscreteTimeIntegrator2_PrevR_o = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S17>/Discrete-Time Integrator3' */
  highgain_acro_DW.DiscreteTimeIntegrator3_PrevRes = 0;
}

/* Model terminate function */
void highgain_acro_terminate(void)
{
  /* Terminate for S-Function (sfun_px4_input_rc): '<Root>/input_rc'
   *
   * Block description for '<Root>/input_rc':
   *  RC Input Block
   *
   *  This block provides user input control to the model.
   *  It uses the input_rc uORB topic.
   *
   *  The user has the ability to choose which channels are available as outputs from this block and also some optional outputs. These include
   *  Channels 1 through 18
   *  double data type indicating the PWM value from the controller
   *  measured pulse widths for each of the supported channels
   *  Channel Count
   *  uint32 data type of the number of channels which are detector by the PX4
   *  RC Failsafe
   *  boolean data types indicating that the RC Tx is sending the FailSafe signal (if equipped and properly setup)
   *  explicit failsafe flag: true on TX failure or TX out of range , false otherwise.
   *  Only the true state is reliable, as there are some (PPM) receivers on the market going into failsafe without telling us explicitly.
   *  RC Input Source
   *  Enumeration data type indicating which source the RC input is from.
   *  Valid values are found in the ENUM file: RC_INPUT_SOURCE_ENUM.m
   *            Enumeration members for class 'RC_INPUT_SOURCE_ENUM':
   *            RCINPUT_SOURCE_UNKNOWN         (0)
   *            RCINPUT_SOURCE_PX4FMU_PPM      (1)
   *            RCINPUT_SOURCE_PX4IO_PPM       (2)
   *            RCINPUT_SOURCE_PX4IO_SPEKTRUM  (3)
   *            RCINPUT_SOURCE_PX4IO_SBUS      (4)
   *            RCINPUT_SOURCE_PX4IO_ST24      (5)
   *            RCINPUT_SOURCE_MAVLINK         (6)
   *            RCINPUT_SOURCE_QURT            (7)
   *            RCINPUT_SOURCE_PX4FMU_SPEKTRUM (8)
   *            RCINPUT_SOURCE_PX4FMU_SBUS     (9)
   *            RCINPUT_SOURCE_PX4FMU_ST24     (10)
   *            RCINPUT_SOURCE_PX4FMU_SUMD     (11)
   *            RCINPUT_SOURCE_PX4FMU_DSM      (12)
   *            RCINPUT_SOURCE_PX4IO_SUMD      (13)
   *
   *  RSSI - Receive signal strength index
   *  receive signal strength indicator (RSSI): < 0: Undefined, 0: no signal, 255: full reception
   *  RC Lost Connection
   *  boolean data type indicating RC receiver connection status
   *  True, if no frame has arrived in the expected time, false otherwise.
   *  True usally means that the receiver has been disconnected, but can also indicate a radio link loss on "stupid" systems.
   *  Will remain false, if a RX with failsafe option continues to transmit frames after a link loss.
   *
   *  Sample Model: px4demo_input_rc.slx
   */

  /* Close uORB service used in the S-Function Block: <Root>/input_rc */
  close(highgain_acro_DW.input_rc_input_rc_fd.fd);

  /* Terminate for S-Function (sfun_px4_sensor_combined): '<Root>/sensor_combined'
   *
   * Block description for '<Root>/sensor_combined':
   *  Sensor Combined Block
   *
   *  This block enables access to the various sensors available on the px4fmu-v2 hardware.
   *  The user can use these signals in the Simulink control model.
   *  The sample time needs to be provided in the mask dialog.
   *  Optional output ports can also be selected.
   *  Refer to the sample model: px4demo_attitude_control.slx
   *
   *  Signal definitions:
   *  Magnetometer (x,y,z) - single values 每 Magnetic field in NED body frame, in Gauss
   *  Accelerometer (x,y,z) - single values 每 Acceleration in NED body frame, in m/s^2
   *  Gyroscope (p,q,r) - single values 每 Angular velocity in radians per second
   *  Barometer (Altitude) - single value 每 Barometric pressure, already temperature compensated (millibars)
   *  RunTime (timestamp) - double value 每 Timestamp in microseconds since boot, from gyro
   *
   *  The sensor_combined block needs to have the px4io service running on the PX4 hardware in order to get valid signal values.

   */

  /* Close uORB service used in the S-Function Block: <Root>/sensor_combined */
  close(highgain_acro_DW.sensor_combined_sensor_fd.fd);

  /* Close uORB service used in the S-Function Block: <Root>/sensor_combined */
  close(highgain_acro_DW.sensor_combined_mag_fd.fd);

  /* Close uORB service used in the S-Function Block: <Root>/sensor_combined */
  close(highgain_acro_DW.sensor_combined_baro_fd.fd);

  /* Terminate for S-Function (sfun_px4_pwm): '<Root>/PWM_output'
   *
   * Block description for '<Root>/PWM_output':
   *  This block allows the user to send the appropriate PWM signals out to the PX4 outputs.
   *  These are usually connected to the ESCs which control the motor speeds.
   *  In order for the flight control to arm (enable) the output from the software side, the ARM Output input must be held high (boolean TRUE).
   *  Only then will the PWM values be sent out the PX4 hardware ports.
   *  This is usually a function of the RC Tx in combination with other flight modes programmed in the Simulink model by the user.
   *  The block has 8 available ports (data type uint16) which can be selectively chosen.
   *  These correspond to the 8 PWM output ports on the px4fmu hardware.
   */

  /* disable pwm outputs */
  px4_ioctl(g_pwm_fd, PWM_SERVO_SET(0), PWM_MOTOR_OFF);
  g_outputs.output[0] = PWM_MOTOR_OFF;
  px4_ioctl(g_pwm_fd, PWM_SERVO_SET(1), PWM_MOTOR_OFF);
  g_outputs.output[1] = PWM_MOTOR_OFF;
  px4_ioctl(g_pwm_fd, PWM_SERVO_SET(2), PWM_MOTOR_OFF);
  g_outputs.output[2] = PWM_MOTOR_OFF;
  px4_ioctl(g_pwm_fd, PWM_SERVO_SET(3), PWM_MOTOR_OFF);
  g_outputs.output[3] = PWM_MOTOR_OFF;
  g_outputs.timestamp = hrt_absolute_time();
  orb_publish(ORB_ID(actuator_outputs), g_outputs_pub, &g_outputs);

  /* Close uORB service used in the PWM S-Function Block: <Root>/PWM_output  */
  orb_unadvertise(g_outputs_pub);
  px4_ioctl(g_pwm_fd, PWM_SERVO_DISARM, 0);
  px4_ioctl(g_pwm_fd, PWM_SERVO_CLEAR_ARM_OK, 0);
  g_armed.timestamp = hrt_absolute_time();
  g_armed.armed = false;
  g_armed.ready_to_arm = false;
  orb_publish(ORB_ID(actuator_armed), g_armed_pub, &g_armed);
  orb_unadvertise(g_armed_pub);
  g_pwm_enabled = false;

  /* Close handle used in the PWM S-Function Block: <Root>/PWM_output */
  px4_close(g_pwm_fd);

  /* Terminate for S-Function (sfun_px4_rgbled): '<Root>/RGB_LED' */

  /* Turn off LED */
  highgain_acro_DW.RGB_LED_sl_led_control_s.led_mask = 0xff;
  highgain_acro_DW.RGB_LED_sl_led_control_s.mode = SL_MODE_OFF;
  highgain_acro_DW.RGB_LED_sl_led_control_s.priority = 0;
  highgain_acro_DW.RGB_LED_sl_led_control_s.timestamp = hrt_absolute_time();
  highgain_acro_DW.RGB_LED_orb_advert_t = orb_advertise_queue(ORB_ID(led_control),
    &highgain_acro_DW.RGB_LED_sl_led_control_s, LED_UORB_QUEUE_LENGTH);

  /* Close uORB service used in the S-Function Block: <Root>/RGB_LED */
  orb_unadvertise(highgain_acro_DW.RGB_LED_orb_advert_t);

  /* Terminate for S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced' */

  /* Close uORB service used in the S-Function Block: <S1>/uORB Write Advanced */
  orb_unadvertise(highgain_acro_DW.uORBWriteAdvanced_uorb_advert);

  /* Terminate for S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced1' */

  /* Close uORB service used in the S-Function Block: <S1>/uORB Write Advanced1 */
  orb_unadvertise(highgain_acro_DW.uORBWriteAdvanced1_uorb_advert);

  /* Terminate for S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced2' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   *  Constant: '<S1>/Constant2'
   *  Constant: '<S1>/Constant3'
   */

  /* Close uORB service used in the S-Function Block: <S1>/uORB Write Advanced2 */
  orb_unadvertise(highgain_acro_DW.uORBWriteAdvanced2_uorb_advert);

  /* Terminate for S-Function (sfun_px4_uorb_write): '<S1>/uORB Write Advanced3' */

  /* Close uORB service used in the S-Function Block: <S1>/uORB Write Advanced3 */
  orb_unadvertise(highgain_acro_DW.uORBWriteAdvanced3_uorb_advert);

  /* Terminate for S-Function (sfun_px4_vehicle_attitude): '<Root>/vehicle_attitude'
   *
   * Block description for '<Root>/vehicle_attitude':
   *  This block gives access to the running service that calculates the vehicle＊s attitude.  A uORB topic (vehicle_attitude (attitude measurements)) publisher MUST be running in order for this block to provide valid signal values.  The available ones as of v1.3 are:
   *  attitude_estimator_ekf 每 EKF-Extended Kalman Filter for attitude estimation
   *  attitude_estimator_so3 每 SO(3)-attitude estimation by using accelerometer, gyroscopes and magnetometer
   *  One of these MUST be running on the px4fmu in order for this block to return valid values.  Refer to the sample model: px4demo_attitude_control.slx. Attitude in NED (North-East-Down) body frame in SI units.
   *  Signal definitions:
   *  Roll 每 single value, Roll angle (rad, Tait-Bryan, NED)
   *  Pitch 每 single value, Pitch angle (rad, Tait-Bryan, NED)
   *  Yaw 每 single value, Yaw angle (rad, Tait-Bryan, NED)
   *  Quaternion (NED) 每 single(4) values (optional based on the uORB publisher)
   *
   */

  /* Close uORB service used in the S-Function Block: <Root>/vehicle_attitude */
  close(highgain_acro_DW.vehicle_attitude_vehicle_attitu.fd);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
