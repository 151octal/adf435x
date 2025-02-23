
```cpp
  /* ©2024 kd9fww. ADF435x stand alone using ATMEGA328 hardware SPI.
  https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Where you got this code.
  https://www.analog.com/ADF4351 <- The device for which this code is specifically tailored.
  https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet
  US$30, for an assembled pll module from a major online discount household retail store.
  As host for ATMEGA328, the Adafruit 'metro mini' is used which has i2c connectoring and is
  configurable for 3V operation - the Nano product is 5V. Beyond stating that the ADF435x logic
  level is 3V, I assume that you know what you're doing - dont blame me if you BRICK yours.
  ------------------------------------------------------------------------------------------------
  This code is a single pll version only. A second pll would be accommodated with {le, ld} on
  {D9, D8}, respectively. And sharing their {REF, 5v, clk, dat, pdr, GND} signals. This scheme
  does not preclude the possibilty of supporting two plls See below. Be aware that there exist
  ADF435x modules that do not have the REF signal brought out on (what is a dummy) SMA connector.
  That is, the REF connector is not connected.
  ------------------------------------------------------------------------------------------------
  Wire-wrap and limit to 5cm and common mode choke the aggregate of (qty:9) wires connecting the
  ADF435x module. Supply the ADF435x module from the mini on-board 5v reg output.
  ------------------------------------------------------------------------------------------------
                  ADF435x module pin header     h.x: mini pcb header pin number
                      component side       5V---h.4:mini.reg5---To pll.reg3.3 input •5.5V MAX•
                        |-------|
                  [[•]] | 0 | 9 | 3v3 -do not connect-
                  [[•]] | 8 | 7 | GND--h.5:(GND)
    h.25:D10(SS*)----le | 6 | 5 | dat--h.26:D11(MOSI)
    h.28:D13(SCK)---clk | 4 | 3 | mux--h19:D4 [optional]
    h.22:D7----------ld | 2 | 1 | pdr--h.21:D6
                        |-------|
    Leave  0  open to provide [[sleeve]] jumper to  8 .
  ------------------------------------------------------------------------------------------------
  The LED (on D13) appears to be in contention with the default SPI clock line and is not easily
  open circuited. Look, "Let It Be." and "Fughet about it.", OK? SPI will work regardless.
```
