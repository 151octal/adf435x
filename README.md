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
 LFT SFT RGT | LFT: Cursor left  | SFT+LFT: Next Axis
     DEC     | DEC: Decrement    | SFT+DEC: Toggle RF
             | RGT: Cursor right | SFT+RGT: Calibration
------------------------------------------------------------
Accomodated are 1) optional i2c serial eemem for overide of
defaults with saved settings atstartup, 2) oled for
displaying settings, and 3) five momentary pushbuttons via
an Adafruit "ANO"encoder i2c SeeSaw module for (optionally
persitent) runtime settings modification. Ajustable are
Power, Frequency, Phase, and Oscillator frequency (for
reference error correction), with a Hold feature which
prevents (unShifted) modification. Displayable are Pll model
internal state values. These are made availabe both on the
display and out the serial port (1000000 baud) when the
error correction (calibration) axis is selected. The runtime
code size is large (26k), but it handles per digit editable
numbers of specified length capable of representing a 2**64
magnitude, with ease. The human interface elements (display,
print, editable persistent settings) sum to more than half
of the code size. Possible are the (unimplemented) sum of
adjacent buttons such as LFT+INC. Communication over SPI
occurs at full speed. If user inactivity exceeds ~7 seconds,
the oled is extinguished and the cpu enters low power sleep
mode.
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
------------------------------------------------------------
This code is a single pll version only. A second pll would
be accommodated with {le, ld} on {D9, D8}, respectively. And
sharing their {REF, 5v, clk, dat, pdr, GND} signals. This
scheme does not preclude the possibilty of supporting two
plls. Be aware that there exist ADF435x modules that do not
have the REF signal brought out on (what is a dummy)
connector. That is, the REF connector is not connected.
------------------------------------------------------------
The LED (on D13) appears to be in contention with the default
SPI clock line and is not easily open circuited. Look,
"Let It Be." and "Fughet about it.", SPI will work regardless.
```
