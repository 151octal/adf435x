/* ©2024 kd9fww. ADF435x stand alone using Arduino Nano hardware SPI (in 300 lines, 3k memory).
  https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Where you got this code.
  https://www.analog.com/ADF4351 <- The device for which this code is specifically tailored.
  https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet
  US$45, for an assembled pll module from a company with the same name as a South American river.
  Bi-directional level shifter module assy., P/N: TXS0108E hereafter referred to as: Shfty;
  https://www.ti.com/lit/ds/symlink/txs0108e.pdf  No documentation is available for the (shifter
  chip + bypass cap) assembly. It's pinout is labeled. I acquired mine, and the Nano, for cheap
  from the same aforementioned company. */
#include <ArxContainer.h>  // https://github.com/hideakitai/ArxContainer
#include <SPI.h>  /* Circuitry in a nutshell:
  The Shfty is post † soldered to the Nano such that the pins b1..b6 directly connect to the
  Nano's pcb pins h9..h14. Wire-wrap the rest. Limit to 7cm and common mode choke the aggregate
  of (qty:9) wires connecting the ADF435x module. Supply the ADF435x module from the Nano's on
  board 5v reg output. Use the ADF435x module's onboard 3v reg output to supply the Shfty.
  Implement NO DC ground loop(s) ††.
  https://en.wikipedia.org/w/index.php?title=Ground_loop_(electricity)
  Pin mapping legend: { -w-wire-w-, =p=post=p= }
  Scheme: h.x:(Nano NAME) -/- b.label | a.label(pll name) -/- num (X)  Where:
    h.x: Nano pcb 'header' pin number   Notes:  i) Pin h.1 has a square solder pad
    b.x: Shfty 5V logic pins                   ii) Shifty's supplies: v.b > v.a  and  v.a >= 0
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
  h.10:D7=p====================LD==p=b.2 | a.2(ld)--w----w-2 (F) | Leave '0' open to provide a  |
  h.9:D6=p====================PDR==p=b.1 | a.1(pdr)-w----w-1 (G) | [[sleeve]] jumper to '8'.    |
  h.27:(5V From Nano.reg5)-w--5V---w-b.v | v.a(3v3)-w----w-9 (H) <- (NOT h.17)
  //                   (single point)-b.v-w-------------w-5V (I) <- To pll.reg3.3 input •5.5V MAX•
  h.29:(system.pwr.return-GND: Nano.reg5 return)
  h.30:(system.pwr.supply-VIN: Nano.reg5 input)
  ------------------------------------------------------------------------------------------------
  † posts: equal length, STIFF, solderable, conductors that fit in the holes - don't use bus wire.
  †† Faraday enclosures bonded to earth: a Z5U between GND and earth is better than a DC short.
  • Yes. The LED (on D13) appears to be in contention with the default SPI clock line and is not
  easily open circuited. Look, 'Let It Be.' and 'Fughet about it.', OK? SPI will work regardless.
  ------------------------------------------------------------------------------------------------
  The scheme depicted makes it possible to power the system (Nano, Shfty, PLL) from these sources:
  1) USB, 2) The coaxial power connector on the pll assembly, 3) The Nano power pins, as above.
•SMOKE Don't exceed 5.5V for option 2 SMOKE• *** •Don't use options 2 and 3 at the same time• Ugh.
  A schottky diode (on the Nano) blocks current flowing to the USB host's 5V. Therefor, simul-
  taneous operation with USB and (one) external power supply, is not a (contention) issue.
  Note: avoid supplying the system power at a voltage near the Nano's 5V regultor dropout. Noise
  may cause gross modulation of the Nano 5V which in turn modulates the 3v3 to the extent of the
  3v3 regulator's line rejection. This effect is not present with the supply sufficiently above
  the 5V input dropout (or below the 5V input dropout but enough above 3v3 regulator dropout).
  A USB host can do 5V @ 500 mA. For debug, power from: USB, only; Benchmark: opt 3, >6V, only. */
  // Commented out, but wired:   D4                                      D11       D13
enum class PIN : u8 {    /* MUX = 4, */ PDR = 6, LD = 7, LE = 10 /* DAT = 11, CLK = 13 */ };
auto wait4lock = []() { while( !digitalRead( static_cast<u8>(PIN::LD) )); }; // Block until lock.
auto tx(void *pByte, int nByte) -> void { // SPI stuff here (only)
  auto p = static_cast<u8*>(pByte) + nByte;       // most significant BYTE first
  digitalWrite( static_cast<u8>(PIN::LE), 0 );    // predicate condition for data transfer
  while( nByte-- ) SPI.transfer( *(--p) );        // return value is ignored
  digitalWrite( static_cast<u8>(PIN::LE), 1 ); }; // data is latched on the rising edge
enum Symbol : u8 {  // human readable register 'field' identifiers
    // in datasheet order. Enumerant names do NOT mirror datasheet's names exactly
    fraction,     integer,          // register 0 has 2 symbols
    modulus,      phase,
    prescaler,    phase_adjust,     // 4
    counterReset, cp3state,
    idle,         pdPolarity,
    ldp,          ldf,
    cpIndex,      dblBfr,
    rCounter,     refToggler,
    refDoubler,   muxOut,
    LnLsModes,                      // 13
    clkDivider,   clkDivMode,
    csr,          chrgCancel,
    abp,          bscMode,          // 6
    rfOutPwr,     rfOutEnable,
    auxOutPwr,    auxOutEnable,
    auxFBselect,  muteTillLD,
    vcoPwrDown,   bandSelectClkDiv,
    rfDivSelect,  rfFBselect,       // 10
    led_mode,                       // 1
  _end
  };  static constexpr auto nSymbol{ Symbol::_end }; // for subsequent 'sanity check' only
using S = Symbol;
static constexpr struct Specification { const u8 RANK, OFFSET, WIDTH; } ADF435x[] = { /*
  This entire struct is human deduced via inspection of the datasheet. Unique to the ADF435x.
    N:      Number of (32 bit) "registers": 6
    RANK:   Datasheet Register Number = N - 1 - RANK
            tx() in ascending RANK order, unless not dirty. Thus, datasheet register '0' is
            always tx()'d last (and will always need to be tx()'d). See flush() below.
    OFFSET: Zero based position of the field's least significant bit.
    WIDTH:  Correct. The number of bits in a field (and is at least one). •You get a gold star•
   { fraction }, { integer }, */
  {5,  3, 12}, {5, 15, 16}, /* r0                                      // begin taedium #1 of two
   { modulus }, {  phase  },  Et cetera. */
  {4,  3, 12}, {4, 15, 12}, {4, 27,  1}, {4, 28,  1}, // r1
  {3,  3,  1}, {3,  4,  1}, {3,  5,  1}, {3,  6,  1}, {3,  7,  1}, {3,  8,  1}, {3,  9,  4},
  {3, 13,  1}, {3, 14, 10}, {3, 24,  1}, {3, 25,  1}, {3, 26,  3}, {3, 29,  2}, // r2
  {2,  3, 12}, {2, 15,  2}, {2, 18,  1}, {2, 21,  1}, {2, 22,  1}, {2, 23,  1}, // r3
  {1,  3,  2}, {1,  5,  1}, {1,  6,  2}, {1,  8,  1}, {1,  9,  1}, {1, 10,  1},
  {1, 11,  1}, {1, 12,  8}, {1, 20,  3}, {1, 23,  1}, // r4
  {0, 22,  2} }; // r5                                                    // end taedium #1 of two
  static_assert(nSymbol == (sizeof(ADF435x) / sizeof(ADF435x[0])));       // sane, at last, hahaha!
  // ©2024 kd9fww
struct SpecifiedOverlay {
  struct Device {
    static constexpr auto N{ 6 };
    using RegArray = std::array<u32, N>; /*
      With the exception of r5 bits 19 and 20, all "reserved" bits are to be set to zero. 
      This mechanism adheres to the principle of 'Resource Aquisition Is Initialization'. As such,
      is principled, clear, concise, and unencumbered. Translation: No speed bumps. No barriers. */
  u8 durty; SPISettings settings; RegArray reg; } dev =
  { 0, SPISettings(4000000, MSBFIRST, SPI_MODE0),  Device::RegArray{ 0x180005, 4, 3, 2, 1, 0 } };
      // usage: object.set( symA,valA ).set( symB,valB ) ••• ad infinitum
  auto set( S symbol,u16 value ) -> decltype(*this) {
    static constexpr u32 MASK[] = {
      0, 1, 3, 7, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383, 32767, 65535 };
    auto pSpec = &ADF435x[ static_cast<const u8>( symbol ) ];
    dev.reg[pSpec->RANK] &= ( ~(        MASK[pSpec->WIDTH]   << pSpec->OFFSET) ); // First, off.
    dev.reg[pSpec->RANK] |= (  (value & MASK[pSpec->WIDTH] ) << pSpec->OFFSET  ); // Then, on.
    static constexpr u8 WEIGHT[] = { 1, 2, 4, 8, 16, 32 };
    dev.durty |= WEIGHT[ (dev.N - 1) - pSpec->RANK ]; // Encode which dev.reg was dirty'd.
    return *this; }
  auto flush() -> void {  // When it is appropriate to do so, flush() 'it' to the target (pll).
    char cx{ 0 };
    switch( dev.durty ) { // Avoid the undirty'd. Well, almost.
      default: break;                   /* Otherwise: say they're all dirty. */
      case 0: return;                   /* None dirty. */
      case 1: cx = dev.N - 1; break;    /* r0 ••• */
      case 2: /* fall thru */           /* r1 ••• */
      case 3: cx = dev.N - 2; break;    /* r1 and r0 ••• */ }
    dev.durty = 0;
    SPI.beginTransaction( dev.settings );
    for(/* empty */; dev.N != cx; ++cx) tx( &dev.reg[cx], sizeof(dev.reg[cx]) );
    SPI.endTransaction(); /* Works and plays well with others. */ }
};  using Overlay = SpecifiedOverlay;
Overlay pll;  // Global scope in order to accomodate the setup();loop(); paradigm. Sigh.
  enum Enable { OFF = 0, ON = 1 };  using E = Enable;
  constexpr auto  FLAG{ E::ON };
  constexpr auto  CONSTRAINT{ 1e1 };                // Assertion failure avoidance.
  constexpr auto  USER_TRIM{ -12 * CONSTRAINT };    // Zero based, via human working in -
  constexpr auto  REF_ERROR{ (FLAG) * USER_TRIM };  // reverse from the 'REF' measurement, below.
  constexpr auto  OSC{ 25.000000e6 },           // Nominal reference freq. Yours may be different.
                  REF{ OSC + REF_ERROR };       // Directly measured. YOURS WILL BE DIFFERENT.
  static_assert( 0 == (REF - OSC) - REF_ERROR, "Least significant digit(s) lost." );
   constexpr auto MIN_VCO{ 2.2e9 }, MAX_VCO{ 4.4e9 }, // Manifest constants ...
                  MIN_PFD{ 125e3 }, MAX_PFD{ 045e6 }; // ... from the datasheet
constexpr enum CHANNEL { EVAL, CM23, CM33, CM70, OOK, TEK, M2, M3, M4, M5, M6, BOT } CHAN = M6;
  // "... how shall I tell you the story?" And the King replied: "Start at the beginning. Proceed
  // until the end. Then stop." Lewis Carroll. "Alice's Adventures in Wonderland". 1865.
constexpr struct { double FREQ; size_t RF_DTAB_INDEX; } TUNE[] = { // Table of CHANNELs.
/* Start here. Add your member (and assign it), or modify one of these (and assign it).
  { [enum] = {  frequency, divisor table index } */
    [EVAL] = { 2500.000000e6, 0 },    // The default freq. setting in the mfr's evaluation program
    [CM23] = { 1300.000000e6, 1 },    // (1240 - 1300) MHz <- 23cm ham band.
    [CM33] = { 0910.000000e6, 2 },
    [CM70] = { 0446.000000e6, 3 },    // (420 - 450) MHz <- call, 70cm ham band.
     [OOK] = { 0433.920000e6, 3 },    // To get 430MHz, divide the VCO by 8 (= 2 * 2 * 2) <- 3 .
     [TEK] = { 0430.350000e6, 3 },    // My crystal controlled data radio.
      [M2] = { 0146.000000e6, 4 },    // (144 - 148) MHz <- call, 2m ham band.
      [M3] = { 0098.765432e6, 5 },    // (88 - 108) MHz <- FM broadcast band in the US.
      [M4] = { 0075.757575e6, 5 },
      [M5] = { 0066.666666e6, 6 },    // <- VCO divided by 64. See FYI FAUX 'declarations' below.
      [M6] = { 0045.678901e6, 6 },    // (50 - 54) MHz <- 6m ham band.
     [BOT] = { 0034.375000e6, 6 } };  // Lowest possible output frequency.
    //
  constexpr auto STEP{ (CHANNEL::EVAL == CHAN) ? 100e3 : (REF) / 50e3 };
  constexpr   u8 RF_DIVISOR_TABLE[] = { 1, 2, 4, 8, 16, 32, 64 };
  constexpr auto RF_DIVISOR_TABLE_INDEX{ TUNE[ CHAN ].RF_DTAB_INDEX };   /* ToDo: calc RF_DIVISOR
      That is, remove the need for the TUNE table entries' second data member. I'm having
      a temporary lapse of insight. */
  constexpr auto RF_DIVISOR = RF_DIVISOR_TABLE[ RF_DIVISOR_TABLE_INDEX ]; /* Use ToDo result here
    or otherwise arrive here, determining RF_DIVISOR by a means as yet to be determined */
  /* FYI FAUX. All permitted freq. ranges. Note: freq. range is limited by VCO, not by REF.
    constexpr auto mid0{ (MAX_VCO - MIN_VCO) / 2 + MIN_VCO };// 3300 ± 1100 = {  4400, 2200   }
    constexpr auto mid1{ mid0 / RF_DIVISOR_TABLE[1] }; //    1650 ± 550     = {  2200, 1100   }
    constexpr auto mid2{ mid0 / RF_DIVISOR_TABLE[2] }; //     825 ± 275     = {  1100, 550    }
    constexpr auto mid3{ mid0 / RF_DIVISOR_TABLE[3] }; //   412.5 ± 137.5   = {   550, 275    }
    constexpr auto mid4{ mid0 / RF_DIVISOR_TABLE[4] }; //  206.25 ± 68.75   = {   275, 137.5  }
    constexpr auto mid5{ mid0 / RF_DIVISOR_TABLE[5] }; // 103.125 ± 34.375  = { 137.5, 68.75  }
    constexpr auto mid6{ mid0 / RF_DIVISOR_TABLE[6] }; // 51.5625 ± 17.1875 = { 68.75, 34.375 } */
  constexpr auto VCO = TUNE[ CHAN ].FREQ * RF_DIVISOR;
    static_assert((MAX_VCO >= VCO) && (MIN_VCO <= VCO));
  constexpr auto REF_DBLR{ E::OFF },  REF_TGLR{ E::OFF };
  constexpr  u16 REF_COUNTER{ (CHANNEL::EVAL == CHAN) ? 1 : 80 };
    static_assert( (0 < REF_COUNTER) && (1024 > REF_COUNTER) );   // Non-zero, 10 bit value.
  constexpr auto PFD = REF * (1 + REF_DBLR) / (1 + REF_TGLR) / REF_COUNTER;
    static_assert((MIN_PFD <= PFD) && (MAX_PFD >= PFD));
  constexpr auto FRACTIONAL_N = TUNE[ CHAN ].FREQ * RF_DIVISOR / PFD;
  constexpr auto WHOLE32{ u32( FRACTIONAL_N ) };
    static_assert( 65536 > WHOLE32 ); // 16 bits.
  constexpr auto WHOLE{ u16( WHOLE32 ) };
    static_assert( 22 < WHOLE );  // Minimum value.
  constexpr auto MOD32 = round(PFD / STEP);
    static_assert( (1 < MOD32) && (4096 > MOD32) ); // 12 bits, with a minimum value.
    static_assert((EVAL == CHAN) ? true : (MOD32 % 2) && (MOD32 % 3)); // NOT factorable by {2,3}.
  constexpr auto MODULUS = u16( MOD32 );
  constexpr auto CLKDIV32 = 150;  // I (clearly) don't understand this yet.
    //= round( PFD / MODULUS * 400e-6 /* Seconds */ ); // from datasheets'
    // 'Phase Resync' text: tSYNC = CLK_DIV_VALUE × MOD × tPFD
  constexpr auto CLKDIV{ (1 > CLKDIV32) ? 1 : u16(CLKDIV32) };
    static_assert( (0 < CLKDIV) && (4096 > CLKDIV) ); // Non-zero, 12 bit value.
  constexpr auto REMAINS = (FRACTIONAL_N - WHOLE);
  constexpr auto FRACTION = u16( round( REMAINS * MODULUS ) );
    static_assert(MODULUS > FRACTION);
      // Yeah. It's a cute const too but stop staring.
constexpr auto frEEqCHECK = (WHOLE + double(FRACTION) / MODULUS) * PFD / RF_DIVISOR;
constexpr auto ERROR = TUNE[ CHAN ].FREQ - frEEqCHECK;  // <- Eyeball this.
  constexpr auto ABSOLUTE_ERROR = (0 <= ERROR) ? ERROR : -ERROR;
  static_assert( ABSOLUTE_ERROR < STEP / 2 );  // For lack of something better that is non-zero.
  // "And away we go!" Gleason.
auto setup() -> void {  /* Up to this point, computation has been accomplished by the compiler. */
  SPI.begin();
  pinMode(static_cast<u8>(PIN::PDR), OUTPUT); // rf output enable. Lock is attainable disabled.
    // For OnOffKeying (OOK) start with 'key' off.
  digitalWrite(static_cast<u8>(PIN::PDR), (CHANNEL::OOK == CHAN) ? E::OFF : E::ON );
  pinMode(static_cast<u8>(PIN::LE), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::LE), 1);  /* Latch on rising edge:
    To accomplish SPI, first LE(1 to 0), 'wiggle' the data line, then LE(0 to 1). See tx(). */
  pinMode(static_cast<u8>(PIN::LD), INPUT);   // Lock detect.
    /* digitalWrite(static_cast<u8>(PIN::MUX), INPUT_PULLUP); */
{ /* Enter another scope. */ Overlay temp; /* Setup a temporary, Specified Overlay.
  (Qty:S::_end) calls of set() are required, in any order. Be sure to flush() after saving. */
    // r0
  temp.set( S::fraction, FRACTION );                              // Begin taedium #2 of two. (1)
  temp.set( S::integer, WHOLE );                                                           // (2)
    // r1
  temp.set( S::modulus, MODULUS );                                                         // (3)
  temp.set( S::phase, 1);  // Adjust phase AFTER loop lock.                                   (4)
  temp.set( S::phase_adjust, E::OFF );                                                     // (5)
  enum PRSCL { four5ths = 0, eight9ths };
  temp.set( S::prescaler, (75 < WHOLE) ? PRSCL::eight9ths : PRSCL::four5ths );             // (6)
    // r2
  temp.set( S::counterReset, E::OFF );                                                     // (7)
  temp.set( S::cp3state, E::OFF );                                                         // (8)
  temp.set( S::idle, E::OFF );                                                             // (9)
  enum PDpolarity { negative = 0, positive };
  temp.set( S::pdPolarity, PDpolarity::positive );                                         // (10)
  enum LDPnS { ten = 0, six };  // Lock Detect Precision nanoSeconds
  temp.set( S::ldp, LDPnS::ten );                                                          // (11)
  enum LockDetectFunction{ fracN = 0, intN }; 
  temp.set( S::ldf, LockDetectFunction::fracN );                                           // (12)
  temp.set( S::cpIndex, 7 );  // 0 thru 15, 2.5mA = '7', more increases loop bandwidth.       (13)
  temp.set( S::dblBfr, E::ON );                                                            // (14)
  temp.set( S::rCounter, REF_COUNTER );                                                    // (15)
  temp.set( S::refToggler, REF_TGLR );                                                     // (16)
  temp.set( S::refDoubler, REF_DBLR );                                                     // (17)
  enum MuxOut { HiZ = 0, DVdd, DGnd, RcountOut, NdivOut, analogLock, digitalLock };
  temp.set( S::muxOut, MuxOut::HiZ ); // see 'cheat sheet'                                    (18)
  constexpr enum NoiseSpurMode { lowNoise = 0, lowSpur = 3 } nsMode = lowNoise;
    static_assert(( NoiseSpurMode::lowSpur == nsMode) ? (49 < MODULUS ? 1 : 0) : 1 );
  temp.set( S::LnLsModes, nsMode );                                                        // (19)
    // r3
  temp.set( S::clkDivider, CLKDIV );                                                       // (20)
  enum ClockingMode { dividerOff = 0, fastLock, phResync }; // dunno, still
  temp.set( S::clkDivMode, ClockingMode::dividerOff );                                     // (21)
  temp.set( S::csr, E::ON ); // Cycle Slip reduction                                          (22)
  temp.set( S::chrgCancel, E::OFF );                                                       // (23)
  enum ABPnS { nS6fracN = 0, nS3intN };   // AntiBacklash Pulse nanoSeconds
  temp.set( S::abp, ABPnS::nS6fracN );                                                     // (24)
  enum BandSelMd { automatic = 0, programmed };
  temp.set( S::bscMode, (MIN_PFD < PFD) ? BandSelMd::programmed : BandSelMd::automatic );  // (25)
    // r4
  constexpr enum dBm { minus4, minus1, plus2, plus5 } auxPower = minus4, outPower = plus5;
  temp.set( S::rfOutPwr, outPower );                                                       // (26)
  temp.set( S::rfOutEnable, E::ON );                                                       // (27)
  temp.set( S::auxOutPwr, auxPower );                                                      // (28)
  temp.set( S::auxOutEnable, E::OFF );                                                     // (29)
  constexpr enum FDBK { divided = 0, fundamental } Feedback = divided;
  temp.set( S::auxFBselect, Feedback ); // Untested. Because, I can't.                        (30)
  temp.set( S::muteTillLD, E::ON );                                                        // (31)
  temp.set( S::vcoPwrDown, E::OFF );                                                       // (32)
  constexpr auto BscClkDiv = round(PFD / MIN_PFD); // Round-off bug, my code.
    static_assert( (0 < BscClkDiv) && (256 > BscClkDiv) ); // Non-zero, 8 bit value.
  temp.set( S::bandSelectClkDiv, u8(BscClkDiv) );                                          // (33)
  temp.set( S::rfDivSelect, RF_DIVISOR_TABLE_INDEX );                                      // (34)
  temp.set( S::rfFBselect, !Feedback );  /* EEK! Why the negation?                            (35)
    It works NEGATED. I'm stumped. Perhaps I've been daVinci'd. */
    // r5
  enum LedMode { low = 0, lockDetect = 1, high = 3 };
  temp.set( S::led_mode, LedMode::lockDetect );       // Ding. Winner! End taedium #two of 2. (36)
pll = temp;  /* Save and exit scope (discarding temp). */ }
pll.flush();  wait4lock();  // That pretty blue led indicates phase lock.
  // Now, set phase (at 180º). Can't test this, yet.
  // pll.set( S::phase_adjust,E::ON ).set( S::phase,(MODULUS >> 1) ).flush();
/* End setup() */ }
  // Jettson[George]: "Jane! JANE! Stop this crazy thing! JANE! !!!".
auto loop() -> void {  }  //  kd9fww. Known for lotsa things. 'Gotcha' code isn't one of them.
