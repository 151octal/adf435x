
```cpp
  /* ©2024 kd9fww. ADF435x stand alone using Arduino Nano hardware SPI.
  See •BRICK• Warning on last line of this file.
  https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Where you got this code.
  https://www.analog.com/ADF4351 <- The device for which this code is specifically tailored.
  https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet
  US$30, for an assembled pll module from a major online discount household retail store.
  Bi-directional level shifter module assy., P/N: TXS0108E hereafter referred to as: Shfty;
  https://www.ti.com/lit/ds/symlink/txs0108e.pdf  No documentation is available for the (shifter
  chip + bypass cap) assembly. The pinout is labeled. I acquired mine, and the Nano, for cheap
  from the same aforementioned company.
  ------------------------------------------------------------------------------------------------
  This code is a single pll version only. A second pll would be accommodated with {le, ld} on
  {D9, D8}, respectively. And sharing their {REF, 5v, clk, dat, pdr, GND} signals. With {3v3, mux}
  from one pll only. This scheme does not preclude the possibilty of supporting two plls See below
  ------------------------------------------------------------------------------------------------
  A mechanism for runtime frequency and phase control is provided.
  ------------------------------------------------------------------------------------------------
  The Shfty is post † soldered to the Nano such that the pins b1..b6 directly connect to the
  Nano pcb pins h9..h14. Wire-wrap the rest. Limit to 7cm and common mode choke the aggregate
  of (qty:9) wires connecting the ADF435x module. Supply the ADF435x module from the Nano on-
  board 5v reg output. Use the ADF435x modules onboard 3v reg output to supply the Shfty.
  Implement NO DC ground loop(s) ††.
  https://en.wikipedia.org/w/index.php?title=Ground_loop_(electricity)
  Pin mapping legend: { -w-wire-w-, =p=post=p= }
  Scheme: h.x:(Nano NAME) -/- b.label | a.label(pll name) -/- num (X)  Where:
    h.x: Nano pcb header pin number     Notes:  i) Pin h.1 has a square solder pad
    b.x: Shfty 5V logic pins                   ii) Shifty supplies: v.b > v.a  and  v.a >= 0
    a.x: Shfty 3V logic pins                        --> (nominals) 5.1 = v.b & 3.3 = v.a <--
  Nano                                 Shfty                 pll module
  30 pins                             18 pins                 9 wires. Denoted (A) thru (I)
  ---              -/-                  ---         -/-      --- ________________________________
  //         (single point)-ground-w-GND-w---------------w-7 (A) |  ADF435x module pin header   |
  h.4:(GND)-w----------------------w-GND | oe-w-----w-v.a        |     component side view      |
  h.16:D13(SCK)-w-------------CLK--w-b.8 | a.8(clk)-w----w-4 (B) |------------------------------|
  h.15:D12(MISO) <- open circuit                                 |     [[•]] | 0 | 9 | 3v3 (H)  |
  h.7:D4(T0)-w----------------MUX--w-b.7 | a.7(mux)-w----w-3 (C) |     [[•]] | 8 | 7 | GND (A)  |
  h.14:D11(MOSI)=p============DAT==p=b.6 | a.6(dat)-w----w-5 (D) |   (E)  le | 6 | 5 | dat (D)  |
  h.13:D10(SS*)=p==============LE==p=b.5 | a.5(le)--w----w-6 (E) |   (B) clk | 4 | 3 | mux (C)  |
  h.12:D9=p========================p=b.4 | a.4  available.0      |   (F)  ld | 2 | 1 | pdr (G)  |
  h.11:D8=p========================p=b.3 | a.3  available.1      |------------------------------|
  h.10:D7=p====================LD==p=b.2 | a.2(ld)--w----w-2 (F) | Leave  0  open to provide a  |
  h.9:D6=p====================PDR==p=b.1 | a.1(pdr)-w----w-1 (G) | [[sleeve]] jumper to  8 .    |
  h.27:(5V From Nano.reg5)-w--5V---w-b.v | v.a(3v3)-w----w-9 (H) <- (NOT h.17)
  //                   (single point)-b.v-w-------------w-5V (I) <- To pll.reg3.3 input •5.5V MAX•
  h.29:(system.pwr.return-GND: Nano.reg5 return)
  h.30:(system.pwr.supply-VIN: Nano.reg5 input)
  ------------------------------------------------------------------------------------------------
  † posts: equal length, STIFF, solderable, conductors that fit in the holes - dont use bus wire.
  †† Faraday enclosures bonded to earth: a Z5U between GND and earth is better than a DC short.
  ------------------------------------------------------------------------------------------------
  The LED (on D13) appears to be in contention with the default SPI clock line and is not easily
  open circuited. Look, "Let It Be." and "Fughet about it.", OK? SPI will work regardless.
  ------------------------------------------------------------------------------------------------
  The scheme depicted makes it possible to power the system (Nano, Shfty, PLL) from these sources:
  1) USB, 2) The coaxial power connector on the pll assembly, 3) The Nano power pins, as above.
  A schottky diode (on the Nano) blocks current flowing to the USB hosts 5V. Therefor, simul-
  taneous operation with USB and (one) external power supply, is not a (contention) issue.
  Note: avoid supplying the system power at a voltage near the Nano 5V regultor dropout. Noise
  may cause gross modulation of the Nano 5V which in turn modulates the 3v3 to the extent of the
  3v3 regulator line rejection. This effect is not present with the supply sufficiently above
  the 5V input dropout (or below the 5V input dropout but enough above 3v3 regulator dropout).
  A USB host can do 5V @ 500 mA. For debug, power from: USB, only; Benchmark: opt 3, >6V, only.

  •BRICK• Dont exceed 5.5V for option 2 •BRICK• •Dont use options 2 AND 3• Man make fire. Ugh.  */

```
