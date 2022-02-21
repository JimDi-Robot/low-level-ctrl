%%% AttitudeControl_FLY
clear
path('./icon/',path);

% constant value
RAD2DEG = 57.2957795;
DEG2RAD = 0.0174533;
% throttle when UAV is hovering
THR_HOVER = 0.609;

%% control parameter
% integral saturation
Saturation_I_RP_Max = 0.3;
Saturation_I_RP_Min = -0.3;
Saturation_I_Y_Max = 0.2;
Saturation_I_Y_Min = -0.2;
% throttle amplitude
MAX_MAN_THR = 0.9;
MIN_MAN_THR = 0.05;
% max control angle, default 35deg
MAX_CONTROL_ANGLE_ROLL = 35;
MAX_CONTROL_ANGLE_PITCH  = 35;
% max control angle rate, rad/s 
MAX_CONTROL_ANGLE_RATE_PITCH = 220;
MAX_CONTROL_ANGLE_RATE_ROLL = 220;
MAX_CONTROL_ANGLE_RATE_Y = 200;

MAX_ATTI_ROLLPITCH_CONTROL_COMMAND = 220 * DEG2RAD;
MAX_ATTI_YAW_CONTROL_COMMAND = 200 * DEG2RAD;

% MC_ROLL_P = 15;            %for atti rate control
% MC_ROLLRATE_P = 0.2;    %for atti control
% MC_ROLLRATE_I = 0.05;    %for atti control
% MC_ROLLRATE_D = 0;    %for atti control

MC_ROLL_P = Pixhawk_CSC.Parameter( {single(1), 'MC_ROLL_P'});            %for atti rate control
MC_ROLLRATE_P = Pixhawk_CSC.Parameter( {single(1), 'MC_ROLLRATE_P'});    %for atti control
MC_ROLLRATE_I = Pixhawk_CSC.Parameter( {single(1), 'MC_ROLLRATE_I'});    %for atti control
MC_ROLLRATE_D = Pixhawk_CSC.Parameter( {single(1), 'MC_ROLLRATE_D'});    %for atti control

MAX_ATTI_INTEGRAL_SAT_UPPPER = 1;   %rad
MAX_ATTI_INTEGRAL_SAT_LOWER  = -1;  %rad
% run simulink model
highgain_with_data_record_param_lowpass