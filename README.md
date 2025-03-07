```cpp
/* ©2024 kd9fww. ADF435x using ATMEGA328 hardware SPI.
https://github.com/151octal/adf435x/blob/main/LICENSE
https://github.com/151octal/adf435x/blob/main/adf435x.ino
https://www.analog.com/ADF4351
https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet
US$30, for an assembled pll module from a major online
discount household retail store. As host for the ATMEGA328,
the Adafruit 'metro_mini' is used which has i2c connectoring
and is configurable for 3V operation - the Arduino Nano
product is 5V. Beyond stating that the ADF435x logic level
is 3V, I assume that you know what you're doing - dont blame
me if you BRICK yours.
------------------------------------------------------------
Recognized sequences:
    1st (unShifted): 1)Press 1st 2)Release 1st
    2nd   (Shifted): 1)Press hold SHFT 2)Press 2nd
     INC     | INC: Increment    | SFT+INC: Save
 LFT SFT RGT | LFT: Cursor left  | SFT+LFT: Toggle Edit
     DEC     | DEC: Decrement    | SFT+DEC: Toggle RF
             | RGT: Cursor right | SFT+RGT: Details
    KNOB: Next/Prev Axis
------------------------------------------------------------
Accomodated are 1) optional i2c serial eemem for overide of
defaults with saved settings atstartup, 2) oled for
displaying settings, and 3) five momentary pushbuttons via
an Adafruit "ANO"encoder i2c SeeSaw module for (optionally
persitent) runtime settings modification. Ajustable are
Power, Frequency, Phase, and Oscillator frequency (for
reference error correction), with a Hold feature which
prevents digit editing. Displayable are Pll model internal
state values. These are made availabe both on the display
and out the serial port (115200 baud) when Details are
selected. Communication over SPI occurs at full speed.
If user inactivity exceeds 10 seconds, the oled is
extinguished and the cpu enters low power sleep mode. Closed
loop control over the reference frequency is made by
measuring the oscillator's temperature with a thermistor
connected to the metro_mini ADC.
------------------------------------------------------------
Wire-wrap, limit to 5cm, and common mode choke the aggregate
of (qty:8) wires connecting the ADF435x module. Configure
the metro_mini for 3V. Solder a wrap pin to the center pin
of the coaxial power connector of the ADF435x module to
supply it from the metro_mini on-board 5v regulated output.
Tack it on the connector rear and oriented it parallel to
the pcb. This renders the coaxial connector redundant. The
total supply current is ~0.15A. Supply the system via the
metro_mini any way you like - USB alone is adequate. For
Faraday enclosures, utilize a Z5U or X7R dielectric
capacitor for an AC short from GND (5V return) to earth,
thereby eliminating the creation of any DC ground loop(s).
A single (interrupt) singal is required from the button
assembly to the ATMEGA328 host.
------------------------------------------------------------
          ADF435x module pin header     
              component side          
              |---------------|
              | 0 GND | 3v3 9 | •THERE IS NO 3V CONNECTION•
              | 8 ce  | GND 7 |---GND
  D10(SS*)----| 6 le  | dat 5 |---D11(MOSI)
  D13(SCK)----| 4 clk | mux 3 |---D4  •optional•
  D7----------| 2 ld  | pdr 1 |---D6
              |---------------|
 metro_mini 5V---wire---ADF435x module (coaxial center pin)
 PID5740_SeeSaw_INT_PIN---wire---metro_mini_D2 <- REQUIRED!
 The pll will be loaded with either saved (if eemem present)
 or defaults if the SeeSaw module is not present. The host
 uses Ext Interrupt 0 (D2). Buttons are ignored if this
 signal is not connected.
------------------------------------------------------------
The LED (on D13) appears to be in contention with the default
SPI clock line and is not easily open circuited. Look,
"Let It Be." and "Fughet about it.", SPI will work regardless.
```
