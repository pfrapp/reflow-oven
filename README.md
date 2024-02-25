# reflow-oven
DIY reflow oven for SMD soldering

## Scope

In this project, we build an oven for reflow soldering of SMD (surface mount device)
parts, such as for instance microcontrollers etc.
In general, it will help to move away from THT (through hole technology)
boards and enter the realm of SMT (surface mount technology) boards.
This saves space, helps to get rid of evaluation-board-style setups,
and enables fully embedded circuits.

This project is inspired by a reflow oven that I had at the university.
It was a predecessor of this one that you can currently buy at Beta Layout.
https://eu.beta-layout.com/estore/order_product_details.html?wg=1&p=740

Another inspiration is this instructable
https://www.instructables.com/DIY-REFLOW-OVEN/
that shows how to build a DIY oven.

The latter resource ends up with a self-contained oven, as the author wants
to be able to use it as a tool.
It looks really professional and solid.

In this project, I just want to get the oven working in a first step.
Furthermore, I want to use the oven to build an improved version of the controller board.
Also, the oven shall be usable and controllable from my computer.
For this reason, I will not disassemble the oven itself nor use an LCD
display, but just create an MVP which is a controller that
uses the actual oven as is.

## Idea

The general idea is to use an off-the-shelf pizza oven as the central
heating element or oven.
In the process of reflow solering, we need to ensure that the temperature
follows a *reflow soldering profile*.

Therefore, we use a temperature probe to measure the temperature and
feed it back to a microcontroller.

The uC in turn generates a control signal. This is a PWM signal,
as the temperature dynamics is very slow and hence acts as a natural
low-pass filter.
As the uC can only provide the signal, but not the actual power,
we use a solid-state relay (SSR) in order to transform the uC signal
to a mains voltage signal.

## Components

Here is an overview of the essential required components.

| Component            | Manufacturer  | Description                                        | Distributor  | Order no.       | Price (Euro)  |
|----------------------|---------------|----------------------------------------------------|--------------|-----------------|---------------|
| SSR                  | Finder        | Steck-/Printrelais, 1x UM, 250V/10A, 5V, RM3,2     | Reichelt     | FIN 43.41.7 5V  | 5.95          |
| Temperatur probe     | UNI-T         | Temperaturfuehler, Typ K, universal                | Reichelt     | UT TF-K         | 3.95          |
| Arduino Uno          | Arduino       | Arduino A000066 Board UNO Rev3 DIL Core ATMega328  | Voelkner     | A435401         | 22.29         |
| Pizza oven           | Severin       | TO 2045                                            | eBay         |                 |               |
| Fuse                 |               |                                                    |              |                 |               |
| Fuse holder          |               |                                                    |              |                 |               |
| SPI module           |               | ARD SEN MAX6675 (incl. cold junction compensation) |              |                 |               |

### SSR

If there is no voltage applied at the input, then pins 11 and 12
(12 being the one close to A1 and A2) are connected.
Pins 11 and 14 are not connected.

The measured resistance between A1 and A2 is 97 Ohm for the 5V DC relay.

Hypothesis: If voltage is applied between A1 and A2 (use additional 22 Ohm resistor, transistor and impedance converter),
then pins 11 and 14 are connected.

This hypothesis is confirmed (using the lab supply):
Attaching 5 V via a 22 Ohm resistor to A1, and putting A2 on ground, makes pins 11 and 14 connected (no
resistance, connectivity test is positive).
Pins 11 and 12 are then disconnected.


### Temperature probe

When the type K thermocouple is plugged into a volt meter,
one can see 0.0 mV.
This is because both the tip as well as the cold junction
are at the same temperature.

If the tip is heated by breathing at it, the voltage rises from 0.1 mV up to 0.3 mV.
When the tip is being put into an open oven that had a nominal temperature of 225 deg C,
the voltage rises to 2.0 mV or 2.1 mV.

Looking at the table from the second reference, this means that the temperature delta
is around 50 deg C (that is, ca. 70 deg C), which seems plausible given that the oven is open
and one can easily put the hand into it.

Resources
1. https://blog.beamex.com/de/thermoelement-kaltstellenkompensation-vergleichsstelle
2. https://www.omega.de/temperature/Z/pdf/z204-205iec.pdf

### Microcontroller

We use an Arduino Uno.

## Controller PCB

### First version

The first version (revision 1.0.0) of the board has some flaws:
* The transistor connection between collector and emitter is shorted. Potential reasons
    * Solder bridges
    * Defect part
    * Defect board
  
  As a consequence, the red LED is always on.
* The yellow LED is not bright enough with 3.3 V (not even with 5.0 V)
* The LEDs have a much higher forward voltage than I thought they have (red and yellow around 2 V,
  green even 3.3 V)
* Not enough voltage to drive the relay

  Note: The voltage between TP4 and TP5 is 0.566 V, which (at 22 Ohm) corresponds to 25.7 mA.
  According to the datasheet of the 333-2SURC/S400-A8 (red LED), the forward voltage is 2 V, which
  is also what I measured.


Analysis of the defects:

* To address the shorted transistor:
    * Leave for some space between PCB and transistor to reduce the chance of solder bridges
    * Measure another PCB
    * Change the part
* To address the yellow LED: reduce value of R1 (smaller than 1k)
* To have enough voltage for driving the relay (without removing the red LED and replace it by a connection):
    Use +7 V instead of + 5 V to drive the relay.
* Also: Use less current wherever possible for the LEDs. Measure what current they need to be "bright".

## Controller software

