# Setting up the physical EduSCARA robot

Copy of the course lab script (text only), from
<https://moodle.bath.ac.uk/pluginfile.php/3124005/mod_resource/content/3/Labwork/RobotSetup.html>
(University of Bath Moodle, EduSCARA Lab Scripts, made with AutoNOTE), saved 2026-11-02. The figures
are linked, not copied; Moodle needs a sign-in to open them.

## Introduction

This page describes how to setup the EduSCARA manipulator and motion controller.

## Prerequisites

- EduSCARA manipulator ([Figure 1](https://moodle.bath.ac.uk/pluginfile.php/3124005/mod_resource/content/3/images/Manipulator.jpg))
- EduSCARA motion controller ([Figure 2](https://moodle.bath.ac.uk/pluginfile.php/3124005/mod_resource/content/3/images/MotionController.jpg))
- Power supply (set to 5V)
- A breadboard
- Male-male jumper wires
- A Mini-B USB cable
- STM32CubeIDE software installed on your PC

## Wiring

Plug all servos and potentiometers on the manipulator into the motion controller using the cables
coming out the back of the manipulator. These cables should be labelled and will corrospond to
silkscreen labels on the motion controller connectors. The cables can either be plugged directly into
the header pins on the PCB or you can use male-male jumper wires to connect to the screw terminals
instead. Ensure that the brown wire (black for Axis 0) is connected to the GND pin (GNDA for pot 0 and
1) on the PCB for correct pin orientation.
([Figure 3: Example of Axis 1 wiring to motion controller](https://moodle.bath.ac.uk/pluginfile.php/3124005/mod_resource/content/3/images/Axis%20Wiring.png))

Set a lab power supply (or other power supply) to 5V and connect this to the +5V and GND pins on the
Servo PWR connector of the PCB. This can be done using a breadboard and jumper cables or any other
appropriate means. Once connected, check the current being drawn from the power supply. It should be
around 0.02-0.04A. If it is higher than 1A, immediatly disconnect power and check your wiring again as
either something is not plugged in correctly or there is a short circuit somewhere.
([Figure 4: Location of servo power connector](https://moodle.bath.ac.uk/pluginfile.php/3124005/mod_resource/content/3/images/PCBpic.png))

Plug the nucleo board into your PC using the Mini-B USB cable to both power and program the motion
controller.
([Figure 5: Example of power setup for EduSCARA](https://moodle.bath.ac.uk/pluginfile.php/3124005/mod_resource/content/3/images/CompleteWiring.png))

## Potentiometer Calibration

This only really needs to be done when an EduSCARA manipulator is first built or after being taken
apart for repairs. Follow these steps to calibrate pot 0 and 1.

1. Ensure that the nucleo board is plugged into your PC and that the 5V power supply for the servos is
   switched OFF.
2. Open the STM32CubeIDE and open the Servo_Motion_Controller project which can be downloaded below:
   [Download Motion Controller Firmware](https://moodle.bath.ac.uk/pluginfile.php/3124005/mod_resource/content/3/servo_motion_controller.zip)
3. Once open, open the application.c file and comment out the 'define API' line and start a debug
   session as shown below. You may get a popup asking about a perspective switch. If this happens,
   select 'switch'.
   ([Figure 6](https://moodle.bath.ac.uk/pluginfile.php/3124005/mod_resource/content/3/images/STM32Debug.png))
4. With the debug session running, open the live expressions tab and add 'adc_raw' to the list. After
   adding this you should see 2 numbers in this array. If not, you may need to press resume as seen
   below.
   ([Figure 7](https://moodle.bath.ac.uk/pluginfile.php/3124005/mod_resource/content/3/images/DebugScreen.png))
5. Open the potentiometers.c file and find the potentiometers_init() function.
6. Rotate Axis 0 to -90 degrees (anti-clockwise) using the markings on the robot as a guide. Note the
   value in 'adc_raw[0]' and change the 'pots[0].min_raw_value' value to match this.
7. Rotate Axis 0 to +90 degrees and put the 'adc_raw[0]' value into 'pots[0].max_raw_value'.
8. Rotate Axis 1 to 0 degrees (link 2 in line with link 1) and assign the 'adc_raw[1]' value to
   'pots[1].min_raw_value'.
9. Rotate Axis 1 to +90 degrees (clockwise) and assign the 'adc_raw[1]' value to
   'pots[1].max_raw_value'.
   ([Figure 8](https://moodle.bath.ac.uk/pluginfile.php/3124005/mod_resource/content/3/images/potcalib.png))
10. Stop the debug session, save the files, return to the application.c file and un-comment the
    'define API' line, rebuild the code and upload this to the nucleo board as shown below.
    ([Figure 9](https://moodle.bath.ac.uk/pluginfile.php/3124005/mod_resource/content/3/images/STM32Build.png))

The potentiometers should now be calibrated.

## Demo Sequence

To test that all electrical connections are working, switch the servo power supply on and move the mode
switch on the motion controller to the 'DEMO' position and press the 'RESET' button. This should
initiate a built-in demo sequence of movements. Check that all 4 axes are moving. If this is
successful, move the switch to the 'PROGRAM' position and press 'RESET'. The EduSCARA is now ready to
program. To do this, move on the the Robot Operation page.
