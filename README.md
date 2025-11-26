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

Bei 5 V und 120 Ohm in Summe (97 vom Relais, 22 vom Vorwiderstand) erreichen wir den Bemessungsstrom von ca. 40 mA.
5000 mV / 120 Ohm = 42 mA.

Bei 12 V und 300 Ohm in Summe erhalten wir ebenfalls 12000 mV / 300 Ohm = 40 mA.
Die 300 Ohm setzen sich zusammen aus den 97 Ohm des Relais sowie aus 200 Ohm Vorwiderstand.
Dieser wiederum aus 150 Ohm sowie der Parallelschaltung einer Diode und ca. 100 Ohm.
Die Diode hat 100 Ohm, so dass diese Parallelschaltung 50 Ohm hat.
Die SMD Diode sollte, wie auch die bisherige THT Diode, 100 Ohm haben, da sie ca. 2.0 (THT 2.1) Volt
Forward Voltage bei 20 mA hat.
Parallelschaltung ist notwendig, damit der Bemessungsstrom von 40 mA erreicht wird, obwohl
die Diode nur 20 mA hat.


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
- Add manual with important points, for instance
  - check for polarity of diodes and LEDs
  - Alignment of ICs

## Reference solder profiles

- https://www.7pcb.com/blog/lead-free-reflow-profile
- Data sheet of my solder paste
  https://de.beta-layout.com/elektronik-shop/loetzinn-lotpaste/10753-smd-lotpaste-iso-cream/
- TI application report
  **SPRABY1A MSL Ratings and Reflow Profiles** \
  https://www.ti.com/lit/an/spraby1a/spraby1a.pdf?ts=1741440364541


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

### Second SMD soldering (Dec-23, 2024)

#### Compared with the first SMD soldering, I changed the following things:
- Adjusted solder temperature profile with the goal to allow for
  - more preheat time (goal reached)
  - higher peak temperature (not reached, peak temperature was actually slightly lower)
  - longer reflow time (goal reached)
- Better solder paste application
  - I made sure to apply the solder paste in a single round. That means,
    I ensured that I had enough of the paste on the upper edge
    of the stencil.
  - Then I went down slowly *once* while pressing firmly onto the stencil.
    It is important to not go back and forth (potentially allowing for too much solder paste).
  - The described procedure favors less solder paste being applied.
    As a consequence, I did not have to remove any surplus solder paste.
- I did *not* use a separate PCB to attach the thermocouple.
  Rather than that, I braided the thermocouple wire trough the grating of the grillage.
  I made sure that the tip was directly in between two grates, thus avoiding any contact
  of the termocouple tip with the metal grates (or with a dedicated PCB).
- I made the hint to open the door more prominent so that it could be
  seen when standing at the oven and looking on to the computer screen.
- I did solder a microcontroller. It was the 64-pin TM4C123GH6PM.
  Note that I ordered lead-free HASL (that is, the ENIG finish was not necessary).

#### Observations and facts
  - When opening the door, the thermocouple was detected as open, hence
    resulting in non-usable temperature measurements.
    As this was the cool-down phase anyway, no harm was induced.
  - I slipped while placing the Cortex onto the PCB.
    Apparently that was not a show-stopper.
    As can be seen on the magnified pictures I took, the solder paste
    had not been smeared away from the Cortex, but rather only between the pins.
  - The overall solder result was surprisingly good. I did not have to do any
    reworking and hence also did not need to use the no-clean flux.
  - It was raining, but I wanted to finish before Christmas, so I used an umbrella
    to protect the oven.

#### Open todos and fixes after this second soldering round
- USB Stencil checken (also ob Anmerkungen von jlcpcb umgesetzt wurden, erst in KiCad Lib, dann im Projekt)
- Highlight beim Bestuecken (z.B. eine PDF Seite pro Teil, erstellen via KiCad Python Script)
- Sanft positionieren (z.B. Halter fuer Pinzette konstruieren?)
- Dioden Polarity dokumentieren
- LEDs Polarity dokumentieren (fuer jede einzelne LED, da potentiell unterschiedlich pro Typ und zumindest unterschiedlich markiert)
  - Bei "Tetris" artigem Bild ist das kleine Quadrat die Kathode
  - Bei Dreieck ist die Spitze die Kathode
  - Auf der Vorderseite gibt es einen kleine gruene Markierung bei der Kathode
  - Auf dem Silk ist die geschlossene Seite die Kathode, die Anode ist offen
- LED Farben fixen im Schematic (2 mehr blue, 2 weniger gruen)
- Silk fuer Montage USB (also genaue Ausrichtung)
- Cortex Stencil checken (unterschiedlich grosse Luecken auf den Seiten)
  --> hat letztlich nichts gemacht
- SMD Bauteile einsortieren
- Inventur und nachbestellen, auch Resistors, Caps.
- Art Kreuze am Stencil fuer Ausrichtung (in allen 4 Ecken)
- Thermocouple open/closed sollte beides ueber LED direkt sichtbar sein
- Reflow Oven braucht einen Baking Modus (verschiedene Dauern und Temperaturen sollten auswaehlbar sein)
- Spannungsversorgung +12V/-12V auf einen Stecker packen, damit keine unipolare Spannungsversorgung moeglich ist (die mir schon mal den uC kaputt gemacht hat)
- Widerstand oder Zener auf PF4 Signal legen
```
(GPIO Output) --- [ 4.7 kΩ Resistor ] ---+--- (OpAmp Non-Inverting Input)
                                          |
                                    [ 3.3 V Zener Diode ]
                                          |
                                         GND
```

#### Output of the Jupyter script for solder profile analysis

Time above 217 deg C: 114.99984027777778 seconds (with opening the door)<br />
Time between 150 and 180 deg C: 91.99987222222222 seconds<br />
Peak temperature: 1023.75 deg C

Note that there was an error due to the thermocouple being open.
The values are rather<br />

Time above 217 deg C: 60 seconds (with opening the door)<br />
Time between 150 and 180 deg C: 91.99987222222222 seconds<br />
Peak temperature: 233 deg C
