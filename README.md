# Hign-gain-low-level-controller
Hig-Gain Observer Based Low-Level Control for Multirotor UAV
## Installation
### Requirements
Windows10   
Matlab2021b  
RflySim  
PX4 version=1.8.2  

### Step-by-Step Procedure
1. install Matlab2021b
2. install Rflysim  (https://rflysim.com/docs/#/)
3. replace the corresponding files in PX4 Firmware with the files in ./firmware_base

## Let's fly
### Step-by-step
1. open ./simulink_for_generate_autopilot_firmware/init.m
2. cmopile in simulink to generate PX4 firmware
3. connect the Pixhawk autopilot to your computer
4. use "PX4Upload" command in Matlab
5. prepare to fly after calibrating aircraft sensor parameters

### Some config
we have config channel 1 for roll, channel 2 for pitch channel 3 for thrust channel 4 for yaw  channel 5 for arm and disarm

| RC channel| UAV channel | 
| ------ | ------ | 
| 1 | roll direction |
| 2 | pitch direction|
| 3 | thrust |
| 4 | yaw direction|
| 5 | arm/disarm |

### Note
1. If you want to record the relative data, you need to change the SD card parameter into "from boot to shutdown" by QGC(http://qgroundcontrol.com/)
