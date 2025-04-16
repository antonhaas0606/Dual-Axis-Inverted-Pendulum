# Dual Axis Inverted Pendulum

https://github.com/user-attachments/assets/2ce915aa-1283-47be-880a-43116596d27a

## Overview



## Gantry
### H-Bot belt drive
The gantry system enables fast and precise two-axis motion to keep the pendulum balanced by constantly positioning the base under its center of mass. Built on an aluminum extrusion frame, the system uses custom 3D-printed components to hold together the mechanical parts, as shown in the image.

<img width="500" alt="image" src="https://github.com/user-attachments/assets/535618e7-6076-488c-b6ec-edb46b4c747e" /><img width="200" alt="image" src="https://github.com/user-attachments/assets/9bf78739-6bec-4b58-94ac-d5613dfc7bd2" />


Initially, I explored motion systems similar to those in 3D printers and laser cutters, but the need for high-speed movement required minimizing moving mass. This led me to adopt the H-bot architecture inspired by the image below, where both motors remain stationary.

A known challenge with H-bot systems is that lateral movement can introduce torque on the crossbeam, causing slight misalignment. To address this, I mounted precise linear guides on both axes using custom 3D-printed brackets to ensure rigidity and accuracy.

<img width="250" alt="image" src="https://github.com/user-attachments/assets/759edab8-2bf5-41ba-946d-4d369a11933d" />

### Stepper Motors and control
#### Linear accel/decel -eration
#### Coordinating multiple steppers with a single interrupt routine

## Pendulum Support
### Single Axis
### Dual Axis
<img width="200" alt="image" src="https://github.com/user-attachments/assets/96e273cc-b0df-4cd4-8c8c-75e214a7bf4d" />



## Angle sensing
### Single Axis - Rotary encoder then IMU
https://github.com/user-attachments/assets/bc1910cb-cfc5-4556-ab2c-7ff43eb2ddb9

https://github.com/user-attachments/assets/5a227c51-e89e-4291-961d-eea1162fdd3e


### Dual Axis - IMU (Combining and filtering data)




## Control system (Coupling of PIDs)
<img width="500" alt="image" src="https://github.com/user-attachments/assets/a70526a4-1b07-40af-b24d-f153e88ad73e" />


