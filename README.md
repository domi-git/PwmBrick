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

All the CAD-Drawings were carried out with Blender. The assembly consists of the following parts:
* top-part of the housing with cutouts for the buttons, the charging port and the LEDs
* bottom-part of the housing
* fibre-optics to guide the LED light
* plus- and minus-button
* pcb with soldered battery-holder for 18650-cell

You can see the listed parts in the following picture:

![Prototype-Render](/Design-Files/Mechanics/Explosion-Drawing.jpg)

## Electronics

The schematic and the layout was done in KiCad. The goal was to keep the overall space of the electronics very close to the footprint of the battery-holder for the 18650 LiIon cell. 

**Block diagram**


![Schematic](/Design-Files/Electronics/BlockDiagram.svg)

**Schematic**


![Schematic](/Design-Files/Electronics/PCB-Design/kicad/PwmBrick.svg)

**Layout**


![Layout-Top](/Design-Files/Electronics/PCB-Design/PwmBrick-Top.svg)
![Layout-Bot](/Design-Files/Electronics/PCB-Design/PwmBrick-Bot.svg)

## Software

TBD

## Possible improvements

### Documentation
* Snap-Fit Design
* The states of the PWM
* Debugging capabilities with TermControl
* How to flash new software


### Mechanics

* Incoporate a cutout for the micro-usb charging port. At the moment the cable does not fit.

### Electronics

For the current PCB:
* Accurately measure the undervoltage lockout thresholds
* Measure the charging current over time

For next PCB revision:
* Incorporate a way to use the charging LED from the microcontroller (It should not be exclusive to the charging IC)
* Low-pass for the measurement of the battery-voltage
* Pull-Downs/Pull-Ups for the Uart/Flash connector

### Software   

* Clean up the code
* Implement a soft transition of the pwm duty-cycle in state changes
* Replace as much of generated code from the IDE with own implementations
