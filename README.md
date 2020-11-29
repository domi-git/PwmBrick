# PwmBrick

The aim of the project is to create a little box to control a DC-motor. The power should come from a 18650 LiIon-cell and the form-factor should be small.

**This is not a finished product! If you use anything from the project, you use it at your own risk! Be especially careful with the handling of LiIon-Batteries! If you don't know what you're doing there is always the risk of a thermal runaway which could lead to a fire or an explosion!**

## Overview

The current state of the prototype can be seen in the following picture:

![Prototype-Render](/Design-Files/Mechanics/Prototype-Rendered.png)

**Main Features:**

| Feature                                          | Value                            |
|--------------------------------------------------|----------------------------------|
| Max. Output Current                              | 3A                               |
| Output Voltage (depends on charging status)      | 3.4V - 4.2V                      |
| PWM-States (Can be switched via the buttons)     | 4                                |
| PWM frequency                                    | 10kHz                            |
| Undervoltage Lockout (going into)                | 3.4V                             |
| Undervoltage Lockout (leaving)                   | 3.7V                             |

## Mechanics

All the CAD-Drawings were carried out with Blender. 

toDos: 
* Explosion drawing with all components
* Explanation of the snap-fit design

## Electronics

Everything was done in KiCad. The goal was to keep the overall space of the electronics very close to the footprint of the battery-holder for the 18650 LiIon cell. The layouts for top and bottom are as follows:

![Layout-Top](/Design-Files/Electronics/PCB-Design/PwmBrick-Top.svg)
![Layout-Bot](/Design-Files/Electronics/PCB-Design/PwmBrick-Bot.svg)

toDo: schematics and explanations

## Software

toDos: 
* Explanation of the states of the PWM
* Explanation of the debugging capabilities with TermControl
* Explanation of the procedure to flash the controller and how the pin is used for UART afterwards

## Possible improvements

### Mechanics

* Incoporate a cutout for the micro-usb charging port. At the moment the cable does not fit.

### Electronics

For the current PCB:
* Accurately measure the undervoltage lockout thresholds. 
* Measure the charging current over time

For next PCB revision:
* Incorporate a way to use the charging LED from the microcontroller. It should not be exclusive to the charging IC.

### Software   

* Clean up the code
* Replace as much of generated code from the IDE with own implementations.
