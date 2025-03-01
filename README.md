```cpp
/* ©2024 kd9fww. ADF435x using ATMEGA328 hardware SPI.
https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Where you got this code.
https://www.analog.com/ADF4351 <- The device for which this code is specifically tailored.
https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet
US$30, for an assembled pll module from a major online discount household retail store.
As host for the ATMEGA328, the Adafruit 'metro_mini' is used which has i2c connectoring and is
configurable for 3V operation - the Arduino Nano product is 5V. Beyond stating that the ADF435x
logic level is 3V, I assume that you know what you're doing - dont blame me if you BRICK yours.
------------------------------------------------------------------------------------------------  
Wire-wrap, limit to 5cm, and common mode choke the aggregate of (qty:8) wires connecting the
ADF435x module. Configure the metro_mini for 3V. Solder a wrap pin to the center pin of the
coaxial power connector of the ADF435x module to supply it from the metro_mini on-board 5v
regulated output. Tack it on the connector rear and oriented it parallel to the pcb. This renders
the coaxial connector redundant. The total supply current is ~0.15A. Supply the system via the 
metro_mini any way you like - USB alone is adequate. For Faraday enclosures, utilize a Z5U or X7R
dielectric capacitor for an AC short from GND (5V return) to earth ground, thereby eliminating
the creation of any DC ground loop(s).
------------------------------------------------------------------------------------------------
          ADF435x module pin header     
              component side          metro_mini 5V---wire---ADF435x module (coaxial center pin)
              |---------------|
              | 0 GND | 3v3 9 |       • THERE IS NO 3V CONNECTION •
              | 8 ce  | GND 7 |---GND
  D10(SS*)----| 6 le  | dat 5 |---D11(MOSI)
  D13(SCK)----| 4 clk | mux 3 |---D4  •optional• Mux is coded as an input but is ignored.
  D7----------| 2 ld  | pdr 1 |---D6
              |---------------|
  Leave  0  open to provide [[sleeve]] jumper to  8  Forces a hard disable when present.
------------------------------------------------------------------------------------------------
Accomodated are 1) optional i2c serial eemem for overide of defaults with saved settings at
startup, 2) optional oled for displaying settings, and 3) optional momentary pushbuttons via an
Adafruit i2c SeeSaw module for (optionally persitent) runtime settings modification. Ajustable
are Power, Frequency, Phase, and Oscillator frequency (calibration), with a Hold feature which
prevents (unShifted) modification. Displayable are Pll model internal (state) values. Calibration
is only permitted when these values are selected to be displayed. The runtime code size is large,
but it handles per digit editable numbers of specified length with ease. The human interface
elements (display, print, editable persistent settings) sum to more than half of the code size.
The least impact button action change would be to drop the Toggle RF feature and implement
something else. Also possible are the (unimplemented) sum of adjacent buttons such as LFT+UP.
Communication over SPI occurs at full speed. The pll is updated at every recognized sequence:
------------------------------------------------------------------------------------------------
    1st (unShifted) Sequence: 1)Press 1st 2)Release 1st
    2nd   (Shifted) Sequence: 1)Press hold SHFT 2)Press hold 2nd 3)Release SHFT 4)Release 2nd
              |-- unShifted -------------------|--------Shifted---------------------------------
      UP      |  UP: Increment digit at Cursor |  SHFT+UP: Save
 LFT SHFT RGT | LFT: Move digit Cursor left    | SHFT+LFT: Next Axis
      DN      |  DN: Decrement digit at Cursor |  SHFT+DN: Toggle RF
              | RGT: Move digit Cursor right   | SHFT+RGT: Show internal state (RefOsc edit)
------------------------------------------------------------------------------------------------
This code is a single pll version only. A second pll would be accommodated with {le, ld} on
{D9, D8}, respectively. And sharing their {REF, 5v, clk, dat, pdr, GND} signals. This scheme
does not preclude the possibilty of supporting two plls. Be aware that there exist ADF435x
modules that do not have the REF signal brought out on (what is a dummy) connector. That is, the
REF connector is not connected.
------------------------------------------------------------------------------------------------
The LED (on D13) appears to be in contention with the default SPI clock line and is not easily
open circuited. Look, "Let It Be." and "Fughet about it.", SPI will work regardless.
```
