# reflow-oven
DIY reflow oven for SMD soldering

## Scope

In this project, we build an oven for reflow soldering of SMD (surface mount device)
parts, such as for instance microcontrollers etc.
In general, it will help to move away from THT (through hole technology)
boards and enter the realm of SMT (surface mount technology) boards.
This saves space, helps to get rid of evaluation-board-style setups,
and enables fully embedded circuits.
It basically makes the PCBs more professional and less tinker-ish.

This project is inspired by a reflow oven that I had at the university.
It was a predecessor of this one that you can currently buy at Beta Layout.
https://eu.beta-layout.com/estore/order_product_details.html?wg=1&p=740

Another inspiration is this instructable
https://www.instructables.com/DIY-REFLOW-OVEN/
that shows how to build a DIY oven.

The latter resource ends up with a self-contained oven, as the author wants
to be able to use it as a stand-alone tool.
It looks really professional and solid.

In this project, I just want to get the oven working in a first step.
Furthermore, I want to use the oven to build an improved version of the controller board
(kind of in a bootstrapping manner).
Also, the oven shall be usable and controllable from my computer.
For this reason, I will not disassemble the oven itself nor use an LCD
display, but just create an MVP which is a controller that
uses the actual oven as is.

## Idea

The general idea is to use an off-the-shelf pizza oven as the central
heating element or oven.
In the process of reflow soldering, we need to ensure that the temperature
follows a *reflow soldering profile*.

Therefore, we use a temperature probe to measure the temperature and
feed it back to a controller which generates the control signal.

This is a PWM signal,
as the temperature dynamics is very slow and hence acts as a natural
low-pass filter.
As the controller can only provide the signal, but not the actual power,
we use a solid-state relay (SSR) as the actuator
in order to transform the uC signal
to a mains voltage signal.

The controller algorithm runs on a Raspberry Pi. It is connected via
USB to a Cortex M4 microcontroller. The latter is used
to acquire the temperature signal and to create the PWM signal.

## Components

Here is an overview of the essential required components.

| Component                | Manufacturer      | Description                                            | Distributor   | Order no.       | Price (Euro)  |
|--------------------------|-------------------|--------------------------------------------------------|---------------|-----------------|---------------|
| SSR                      | Finder            | Steck-/Printrelais, 1x UM, 250V/10A, 5V, RM3,2         | Reichelt      | FIN 43.41.7 5V  | 5.95          |
| Temperatur probe         | UNI-T             | Temperaturfuehler, Typ K, universal                    | Reichelt      | UT TF-K         | 3.95          |
| Cortex M4 Eval Board     | TI                | TM4C1236 Evaluation Board with breakout pins           | DigiKey       | 2314937         | ca. 18        |
| Pizza oven               | Severin           | TO 2045 Oven, 1500 W, 100 to 230 deg C                 | Amazon        |                 | 60            |
| Fuse                     |                   | Feinsicherung 5x20mm, superflink (ff), 8A              | Reichelt      | ESKA 520.126    | ca. 2         |
| Fuse holder              |                   | 5 x 20 mm, 250 V, 10 A, black                          | Reichelt      | SCH 31010045    | 1.50          |
| Temperature converter    | Maxim Integrated  | MAX6675 (incl. cold junction compensation)             | Voelkner      | R025452         | ca. 7         |

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

We use a Cortex M4 on a Tiva evaluation board.
It can be connected to the Raspberry Pi via USB.
The Raspberry Pi is connected to the host PC via Wi-Fi.

This allows
for the complete galvanic isolation between host PC
and reflow oven controller.

## Controller PCB

The controller board is a two-stage amplifier.
The signal from the microcontroller is first amplified using a BJT.
This signal is then used to control a relay.

Yet another stage is present on the Tiva board: It turned out that the
uC pins cannot reliably drive the BJT, so I introduced an impedance converter
using an OpAmp, see the following subsection.


### Impedance converter

Due to the switching of the relay, there are some voltage spikes on the transistor,
that is, on the low voltage side of the relay.

Those spikes are less pronounced when using the lab supply and more
pronounced when using the AD/DC converter.
They are there in any case though.

For this reason, an impedance converter or buffer is needed between the
uC pin and the controller board.

So far I tested the setup using this impedance converter and a lab supply
and it works.
In oder to test it without the lab supply I need a second AC/DC converter,
as the OpAmp needs a dual rail supply.

## Controller software

### Modelling of the reflow oven dynamics

The plant is modelled by a time delay followed by two first-order lag elements.

The parameters of the plant are identified by applying a step function
onto the oven. A 30% and 50% PWM signal is used for the step height.
Also, a 100% PWM step is applyied until a temperature of 100 deg C is reached.
In that case, also the cooling down is measured.

Here are some more details for the motivation of a first-order lag.
Note that we assume that there are two in order to model the
different behavior when heating and when cooling.
```
d_theta/dt = q_zu - q_ab

q_zu = K*u
q_ab = alpha*theta

K ... Proportional zur Leistung des Ofens
theta ... Abkuehlkoeffizient

d_theta/dt = K*u - alpha*theta

d_theta/dt + alpha*theta = K*u

s*Theta + alpha*Theta = K*U

(s + alpha) * Theta = K * U

G = Theta / U = K/(s + alpha)

```

Later a more complex model is used. You can find it in the Jupyter notebook.

### Control design

A PID controller is used for temperature control.
The parameters of a PI controller have been parameterized by the Ziegler-Nichols method.
Based on that, a PID controller has been manually tuned using the experimentally
identified plant.


## Tiva pin usage and connection

| Pin on Tiva   | Connected with                                  | Description        |
|---------------|------------------------------------------------:|-------------------:|
| GND           | Analog Thermo amplifier GND                     | Ground             |
| PE3           | Analog Thermo amplifier between 6.8k and 18k    | ADC input          |
| GND           | GND of BMP280 evaluation board                  | Ground             |
| +3.3V         | VCC of BMP280 evaluation board                  | Supply voltage     |
| PA6           | SCL of BMP280 evaluation board                  | I2C clock          |
| PA7           | SDA of BMP280 evaluation board                  | I2C data           |
| PB6           | Digital Thermo amplifier MISO                   | SPI data           |
| PB5           | Digital Thermo amplifier CS                     | SPI CS             |
| PB4           | Digital Thermo amplifier Clock                  | SPI clock          |
| PF4           | Controller board pin 2                          | Relay control      |

See also the following block diagram overview.

![Block diagram overview](blockdiagram_overview.png)

## General todos

- Add note to add LED signaling open thermocouple
- Add note to add LED signaling request to open door (manually)
- Adjust profile to allow for
  - more preheat time
  - higher peak temperature
  - longer reflow time
- Add manual with important points, for instance
  - check for polarity of diodes and LEDs
  - Alignment of ICs

## Reference solder profiles

- https://www.7pcb.com/blog/lead-free-reflow-profile
- Data sheet of my solder paste
  https://de.beta-layout.com/elektronik-shop/loetzinn-lotpaste/10753-smd-lotpaste-iso-cream/

## Solder reports

### First SMD soldering with this setup (Oct-30, 2024)

- Lead-free HASL PCB (that is, no ENIG)
- No uC soldered (otherwise ENIG required)
- Stencil is required
- Soldered components: 0805 C, 0805 R (contain labels), Diodes, LEDs, OpAmps and other ICs
- Some parts on the PCB did not solder well --> longer preheat time, higher peak temperature, longer reflow time
- Fragmented setup was quite a mess --> higher integration of controller boards, fixture of the BMP280
- Power via lab supply --> Use TDK Lambdas in the future
- Timing of door opening to be made easier
- Application of solder paste --> not too much, also remove where there is too much before placing the part
- Add note to check for polarity of diodes and LEDs
- Reworking requires (no clean) flux

Time above 217 deg C: 52.5 seconds (with opening the door)<br />
Time between 150 and 180 deg C: 77.0 seconds<br />
Peak temperature: 237.0 deg C

