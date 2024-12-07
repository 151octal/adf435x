/* ©kd9fww 2024. ADF435x stand alone using Arduino Nano hardware SPI. (in 300 lines)
  https://github.com/151octal/adf435x/blob/main/adf435x.ino <- where you got this code. */
#include <SPI.h>
  // https://github.com/hideakitai/ArxContainer
#include <ArxContainer.h>
  /* https://www.analog.com/ADF4351 <- the device for which this code is specifically tailored.
     https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet
  I purchased an assembled module for $45 US from the company named after a South American river.
  bi-directional level shifter module assy. [ P/N: TXS0108E hereafter referred to as: Shfty ]
    https://www.ti.com/lit/ds/symlink/txs0108e.pdf
  No documentation is available for the (smd shifter chip + bypass cap) shifter assembly. I
  obtained mine (and the nano) from that same aforementioned company.

  In A Nutshell: The Shfty is post † soldered to the nano such that the pins b1..b6 directly
  connect to the nano's pcb pins h9..h14. Wire-wrap the rest. Limit to 7cm and common mode choke
  the aggregate of (qty:9) wires connecting the ADF435x module. Supply the ADF435x module from the
  nano's on board 5v regulator output. Use the ADF435x module's onboard 3v regulator output to
  supply the 3v3 needed by the Shfty. Implement NO DC ground loop(s) ††.
    https://en.wikipedia.org/w/index.php?title=Ground_loop_(electricity)
  pin mapping legend: { -w-wire-w-, =p=post=p= }
  scheme: h.x:(nano name) -/- b.label | a.label(pll name) -/- num (X)  where:
    h.x: nano pcb 'header' pin number   notes:  i) pin h.1 has a square solder pad
    b.x: level Shfty 5V logic pins              ii) shifter's supplies: v.b > v.a  and  v.a >= 0
    a.x: level Shfty 3V logic pins                    --> (nominals) 5.1 = v.b & 3.3 = v.a <--
  nano                                 Shfty                  pll module
  30 pins                              18 pins                 9 wires. labeled (A) thru (I)
  ---               -/-                  ---         -/-      --- ________________________________
  //          (single point) ground-w-GND-w---------------w-7 (A) |  ADF435x module pin header   |
  h.4:(GND)-w-----------------------w-GND | oe-w-----w-v.a        |     component side view      |
  h.16:D13(SCK)-w--------------CLK--w-b.8 | a.8(clk)-w----w-4 (B) |------------------------------|
  h.15:D12(MISO) <- open circuit: ignore data FROM the pll        |     [[•]] | 0 | 9 | 3v3 (H)  |
  h.7:D4(T0)-w-----------------MUX--w-b.7 | a.7(mux)-w----w-3 (C) |     [[•]] | 8 | 7 | GND (A)  |
  h.14:D11(MOSI)=p=============DAT==p=b.6 | a.6(dat)-w----w-5 (D) |   (E)  le | 6 | 5 | dat (D)  |
  h.13:D10(SS*)=p===============LE==p=b.5 | a.5(le)--w----w-6 (E) |   (B) clk | 4 | 3 | mux (C)  |
  h.12:D9=p=========================p=b.4 | a.4  available.0      |   (F)  ld | 2 | 1 | pdr (G)  |
  h.11:D8=p=========================p=b.3 | a.3  available.1      |------------------------------|
  h.10:D7=p=====================LD==p=b.2 | a.2(ld)--w----w-2 (F) | leave '0' open to provide a  |
  h.9:D6=p=====================PDR==p=b.1 | a.1(pdr)-w----w-1 (G) | [[sleeve]] jumper to '8'     |
  h.27:(5V from nano.reg5)----w-----w-b.v | v.a(3v3)-w----w-9 (H) <- (NOT h.17)
  //                   (single point) b.v-w--------------w-5v (I) <- (to pll.reg3.3 input)
  h.29:(system.pwr.return-GND: nano.reg5 return)
  h.30:(system.pwr.supply-VIN: nano.reg5 input)
  Note: avoid supplying the system power at a voltage near the nano's 5V regultor dropout as noise
  might cause gross modulation of the nano 5V which in turn modulates the 3v3 to the extent of the
  3v3 regulator's line rejection. This effect is not present with the supply sufficiently above
  the 5V input dropout (or below the 5V input dropout but sufficiently above 3v3 regulator input).
  Mine works with 5.1V @ < 500 mA.
  † posts: equal length, STIFF, solderable, conductors that fit in the holes - don't use bus wire.
  †† Faraday enclosures bonded to earth: one high value X7R between GND and EARTH IS better than
  a DC short. */ /* Yes. The nano LED is in contention with the default SPI clock line and is not
  easily open circuited. Look, let it be and 'fughet about it', OK? SPI will work regardless. */
  // commented out, but wired:   D4                                      D11       D13
enum class PIN : u8 {    /* MUX = 4, */ PDR = 6, LD = 7, LE = 10 /* DAT = 11, CLK = 13 */ };
auto wait4lock = []() { while( !digitalRead( static_cast<u8>(PIN::LD) )); }; // block until lock
auto tx(void *pBytes, int nBytes) -> void { // SPI stuff here
  auto p = static_cast<u8*>(pBytes) + nBytes;     // most significant BYTE first
  digitalWrite( static_cast<u8>(PIN::LE), 0 );    // predicate condition for data transfer
  while( nBytes-- ) SPI.transfer( *(--p) );       // return value is ignored
  digitalWrite( static_cast<u8>(PIN::LE), 1 ); }; // data is latched on the rising edge
enum Symbol : u8 {  // human readable register 'field' identifiers
  // in datasheet order. Enumerant names do NOT mirror datasheet's names exactly
  r0 = 0,
      fraction,     integer,          // register 0 has 3 symbols
  r1, modulus,      phase,
      prescaler,    phase_adjust,     // 5
  r2, counterReset, cp3state,
      idle,         pdPolarity,
      ldp,          ldf,
      cpIndex,      dblBfr,
      rCounter,     refToggler,
      refDoubler,   muxOut,
      LnLsModes,                      // 14
  r3, clkDivider,   clkDivMode,
      csr,          chrgCancel,
      abp,          bscMode,          // 7
  r4, rfOutPwr,     rfOutEnable,
      auxOutPwr,    auxOutEnable,
      auxFBselect,  muteTillLD,
      vcoPwrDown,   bandSelectClkDiv,
      rfDivSelect,  rfFBselect,       // 11
  r5, _reserved5,    led_mode,        // 3
  end
  };  static constexpr auto nSymbol{ Symbol::end }; // for subsequent sanity check only
using S = Symbol;
static constexpr struct Specification { const u8 rank, offset, width; } ADF435x[] = { /*
  deduced via datasheet inspection
    N:      number of (32 bit width) "registers": 6
    rank:   datasheetRegisterNumber = N - 1 - rank
            tx() in ascending rank order, unless not dirty. Thus, datasheet register '0' is
            always tx()'d last (and will always need to be tx()'d). See flush() below.
    offset: the zero based, position of field's least significant bit within a 32 bit register.
    width:  Correct, the number of bits (and is never zero). You get a gold star! *//*
   {  r0   }, { integer }, { fraction }, */
  {5,  0, 3}, {5,  3, 12}, {5, 15, 16}, /* r0 has 3 'fields'            // begin taedium #1 of two
   {  r1   }, {  phase  }, { modulus },  et cetera                                              */
  {4,  0, 3}, {4,  3, 12}, {4, 15, 12}, {4, 27,  1}, {4, 28,  1},
   // r2
  {3,  0, 3}, {3,  3,  1}, {3,  4,  1}, {3,  5,  1}, {3,  6,  1},
  {3,  7, 1}, {3,  8,  1}, {3,  9,  4}, {3, 13,  1}, {3, 14, 10},
  {3, 24, 1}, {3, 25,  1}, {3, 26,  3}, {3, 29,  2},
   // r3
  {2,  0, 3}, {2,  3, 12}, {2, 15,  2}, {2, 18,  1}, {2, 21,  1},
  {2, 22, 1}, {2, 23,  1},
   // r4
  {1,  0, 3}, {1,  3,  2}, {1,  5,  1}, {1,  6,  2}, {1,  8,  1},
  {1,  9, 1}, {1, 10,  1}, {1, 11,  1}, {1, 12,  8}, {1, 20,  3},
  {1, 23, 1}, /*
   {  r5   }, { resv'd  }, { LEDmode }                                                          */
  {0,  0, 3}, {0, 19,  2}, {0, 22,  2}
  };  // human tallied number of fields: 43                             // end taedium #1 of two
  static_assert(nSymbol == (sizeof(ADF435x) / sizeof(ADF435x[0])));     // sane, at last, haha!
  // ©kd9fww 2024
struct SpecifiedOverlay {
  struct Frame {
    static constexpr auto N{ 6 };
    using Buffer = std::array<u32, N>;
    // allocation and initialization ("reserved" & "contol" bit 'fields')
      // with the exception of r5 bits 19 and 20, all "reserved" bits must be set to zero. 
      // NB: this is my old school 'constructor'. don't use the set() method to overwrite these
      // An admittedly vulnerable coding but its simple and clear.
  u8 durty; Buffer bfr; } frame = { 0, Frame::Buffer{ 0x180005, 4, 3, 2, 1, 0 } };
      // usage: object.set(symA,valA).set(symB,valB) ••• ad infinitum
  auto set( S symbol, u16 value ) -> decltype(*this) {
    constexpr auto weight = [](int index) {
      static constexpr u8 WeightTable[] = { 1, 2, 4, 8, 16, 32 };
      return WeightTable[index]; };
    static constexpr u32 Mask[] = {
              0,    1,    3,    7,   15,   31,    63,   127,
      255,  511, 1023, 2047, 4095, 8191, 16383, 32767, 65535 };
    auto STp = &ADF435x[ static_cast<const u8>( symbol ) ];
    frame.bfr[ STp->rank ] &= ( ~(        Mask[ STp->width ]   << STp->offset) ); // first off
    frame.bfr[ STp->rank ] |= (  (value & Mask[ STp->width ] ) << STp->offset  ); // then on.
    frame.durty |= weight( (frame.N - 1) - STp->rank ); // encode which u32 was dirty'd
    return *this; }
      // When it is appropriate to do so, flush() 'it' to the pll module.
  auto flush() -> void {
    char cx{ 0 };
    switch( frame.durty ) { // 'vacuum assist' as it were, by avoiding the undirty'd
      default: break;                   /* otherwise: say they're all dirty */
      case 0: return;                   /* none dirty */
      case 1: cx = frame.N - 1; break;  /* r0 ••• */
      case 2: /* fall thru */           /* r1 ••• */
      case 3: cx = frame.N - 2; break;  /* r1 and r0 ••• */ }
    frame.durty = 0;
    for(/* empty */; frame.N != cx; ++cx) tx( &frame.bfr[cx], sizeof(frame.bfr[cx]) ); }
};  using Overlay = SpecifiedOverlay;
Overlay pll;  // global scope in order to accomodate the setup();loop(); paradigm. groan.
  constexpr auto  fOSCerror{ -0110e0 },             // value determined indirectly by measurement
                  fOSC{ 025e6 + fOSCerror },        // fOSC is the actual aforementioned measurement
                  minVCO{ 2.2e9 }, maxVCO{ 4.4e9 }, // manifest constants ...
                  minPFD{ 125e3 }, maxPFD{ 045e6 }; // ... from datasheet prose regarding the PFD
constexpr enum CHANNEL { EVAL, CM23, BIRD, OOK, TEK, /* CM70, M2, */ M3, M4, M5, M6 } chan = M3;
  // "... how shall I tell you the story?" And the King replied: "Start at the beginning. Proceed
  // until the end. Then stop." Lewis Carroll. "Alice's Adventures in Wonderland". 1865.
constexpr struct { double fSet; size_t rfDtabIndex; } tune[] = { // table of channel frequencies
/* start here. add your member and assign it to chan, or modify one of these
  { [enum] = {  frequency, divisor table index } */
    [EVAL] = { 2500.000e6, 0 }, /* the default freq. setting in the mfr's evaluation software */
    [CM23] = { 1240.000e6, 1 }, /* (1240 - 1300) MHz <- 23cm ham band */
    [BIRD] = { 0910.000e6, 2 }, /* my neighborhood 'birdie' */
     [OOK] = { 0433.920e6, 3 }, /* to get 430MHz, divide the vco by 8 (= 2 * 2 * 2) <- 3 */
     [TEK] = { 0430.350e6, 3 }, /* crystal controlled transceiver //
    [CM70] = { 0446.000e6, 3 }, /* (420 - 450) MHz <- 70cm ham band //
      [M2] = { 0146.000e6, 4 }, /* (144 - 148) MHz <- 2m ham band */
      [M3] = { 0100.000e6, 5 }, /* (88 - 108) MHz <- FM broadcast band in US */
      [M4] = { 0075.000e6, 5 },
      [M5] = { 0060.000e6, 6 }, /* <- fVCO divided by 64. see FYI midX's declarations below */
      [M6] = { 0050.000e6, 6 }  /* (50 - 54) MHz <- 6m ham band */ };
    //
  constexpr auto fStep{ (CHANNEL::EVAL == chan) ? 100e3 : (fOSC / 20e3) };
  constexpr u8 rfDivisorTable[] = { 1, 2, 4, 8, 16, 32, 64 };
  constexpr auto RfDivisorTableIndex{ tune[ chan ].rfDtabIndex };   /* todo: calc rfDivisor
      that is, remove the need for the tune table entries' second data member. I'm having
      a temporary lapse of insight. */
  constexpr auto rfDivisor = rfDivisorTable[ RfDivisorTableIndex ]; /* use result of todo here
    or otherwise arrive here, determining rfDivisor by a means as yet to be determined */
    //  FYI. all possible fSet ranges. Note: fSet range is fVCO limited, not by fOSC.
    constexpr auto mid0{ (maxVCO - minVCO) / 2 + minVCO };  // 3300 ± 1100 = { 4400, 2200 } MHz
    constexpr auto mid1{ mid0 / rfDivisorTable[1] };  // 1650 ± 550 = { 2200, 1100 }
    constexpr auto mid2{ mid0 / rfDivisorTable[2] };  // 825 ± 275 = { 1100, 550 }
    constexpr auto mid3{ mid0 / rfDivisorTable[3] };  // 412.5 ± 137.5 = { 550, 275 }
    constexpr auto mid4{ mid0 / rfDivisorTable[4] };  // 206.25 ± 68.75 = { 275, 137.5 }
    constexpr auto mid5{ mid0 / rfDivisorTable[5] };  // 103.125 ± 34.375 = { 137.5, 68.75 }
    constexpr auto mid6{ mid0 / rfDivisorTable[6] };  // 51.5625 ± 17.1875 = { 68.75, 34.375 }
  constexpr auto fVCO = tune[ chan ].fSet * rfDivisor;
    static_assert((maxVCO >= fVCO) && (minVCO <= fVCO));
  enum Enable { OFF = 0, ON };  using E = Enable;
  constexpr auto oscDoubler{ E::OFF },  oscToggler{ E::OFF };
  constexpr u16 OSCcounter{ (CHANNEL::EVAL == chan) ? 1 : 15 }; // such that Modulus is prime
    static_assert( (0 < OSCcounter) && (1024 > OSCcounter) );   // non-zero, 10 bit value
  constexpr auto fPFD = fOSC * (1 + oscDoubler) / (1 + oscToggler) / OSCcounter;
    static_assert((minPFD <= fPFD) && (maxPFD >= fPFD));
  constexpr auto fracN = tune[ chan ].fSet * rfDivisor / fPFD;
  constexpr auto Whole = u16( fracN );
    static_assert(( 22 < Whole ) && (4096 > Whole));  // 12 bits, with a minimum value
  constexpr auto Mod32 = round(fPFD / fStep); // prefer Modulus value NOT factorable by { 2, 3 }
    static_assert( (1 < Mod32) && (4096 > Mod32) );   // 12 bits, with a minimum value
  constexpr auto Modulus = u16( Mod32 );
  constexpr auto clkDiv32 = round( 400e-6 /* Seconds */ * fPFD / Modulus ); // from the datasheet
    // in the 'Phase Resync' paragraphs: tSYNC = CLK_DIV_VALUE × MOD × tPFD
  constexpr auto clkDiv{ (0 == clkDiv32) ? 1 : u16(clkDiv32) };
    static_assert( (0 < clkDiv) && (4096 > clkDiv) ); // non-zero, 12 bit value
  constexpr auto frac = (fracN - Whole);
  constexpr auto Fraction = u16( round( frac * Modulus ) );
    static_assert(Modulus > Fraction);
    // yeah, it's a cute const too but stop staring
constexpr auto freq = (Whole + double(Fraction) / Modulus) * fPFD / rfDivisor;
  constexpr auto error = tune[ chan ].fSet - freq;
  constexpr auto absErr = (0 <= error) ? error : -error;
    static_assert( absErr < fStep / 2 );  // for lack of something better
  // "And away we go!" Gleason.
auto setup() -> void {  /* begin execution
  Up to this point, computation has been accomplished by the compiler. */
  SPI.begin();
  pinMode(static_cast<u8>(PIN::PDR), OUTPUT); // rf output enable. lock is attainable disabled.
    // for OnOffKeying (OOK) start with 'key' off.
  digitalWrite(static_cast<u8>(PIN::PDR), (CHANNEL::OOK == chan) ? E::OFF : E::ON );
  pinMode(static_cast<u8>(PIN::LE), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::LE), 1);  /* latch on rising edge:
    to accomplish SPI, first LE(1 to 0), 'wiggle' the data line, then LE(0 to 1). see tx() */
  pinMode(static_cast<u8>(PIN::LD), INPUT);   // lock detect
  /* digitalWrite(static_cast<u8>(PIN::MUX), INPUT_PULLUP); */
{ /* enter another scope */ Overlay temp; /* setup a temporary, Specified Overlay
  (qty:36) unique calls of set() are required, in any order. Note: set() does NOT write
  directly to the pll; flush() does that, in the correct order.   •••   begin taedium #2 of two */
    // r0
  temp.set( S::fraction, Fraction );                                                       // (1)
  temp.set( S::integer, Whole );                                                           // (2)
    // r1
  temp.set( S::modulus, Modulus );                                                         // (3)
  temp.set( S::phase, 1);  // adjust phase AFTER loop lock                                 // (4)
  temp.set( S::phase_adjust, E::OFF );                                                     // (5)
  enum PreScale { four5ths = 0, eight9ths };
  temp.set( S::prescaler, (75 < Whole) ? PreScale::eight9ths : PreScale::four5ths );       // (6)
    // r2
  temp.set( S::counterReset, E::OFF );                                                     // (7)
  temp.set( S::cp3state, E::OFF );                                                         // (8)
  temp.set( S::idle, E::OFF );                                                             // (9)
  enum PDpolarity { negative = 0, positive };
  temp.set( S::pdPolarity, PDpolarity::positive );                                         // (10)
  enum LDPnS { ten = 0, six };
  temp.set( S::ldp, LDPnS::ten );                                                          // (11)
  enum LDF { fracN = 0, intN };
  temp.set( S::ldf, LDF::fracN );                                                          // (12)
  temp.set( S::cpIndex, 7 );      // 0 thru 15. more increases loop bandwidth              // (13)
  temp.set( S::dblBfr, E::ON );   /*  Why not double buffer?                               // (14)
    it's probably because I am lacking knowledge of some aspect of the pll. this annoys me.
    "There is no substitute for fundamental understanding." E. Neville Pickering. Bradley U. */
  temp.set( S::rCounter, OSCcounter );                                                     // (15)
  temp.set( S::refToggler, oscToggler );                                                   // (16)
  temp.set( S::refDoubler, oscDoubler );                                                   // (17)
  enum MuxOut { HiZ = 0, DVdd, DGnd, RcountOut, NdivOut, analogLock, digitalLock };
  temp.set( S::muxOut, MuxOut::HiZ ); // see 'cheat sheet'                                 // (18)
  constexpr enum NoiseSpurMode { lowNoise = 0, lowSpur = 3 } nsMode = lowNoise;
    static_assert(( NoiseSpurMode::lowSpur == nsMode) ? (49 < Modulus ? 1 : 0) : 1 );
  temp.set( S::LnLsModes, nsMode );                                                        // (19)
    // r3
  temp.set( S::clkDivider, u16(clkDiv) );                                                  // (20)
  enum ClockingMode { dividerOff = 0, fastLock, phResync };
  temp.set( S::clkDivMode, ClockingMode::phResync );                                       // (21)
  temp.set( S::csr, E::OFF );                                                              // (22)
  temp.set( S::chrgCancel, E::OFF );                                                       // (23)
  enum ABPnS { nS6fracN = 0, nS3intN };
  temp.set( S::abp, ABPnS::nS6fracN );                                                     // (24)
  enum BandSelectMode { automatic = 0, programmed };
  temp.set( S::bscMode, BandSelectMode::automatic );                                       // (25)
    // r4
  constexpr enum dBm { minus4, minus1, plus2, plus5 } auxPower = minus4, outPower = plus5;
  temp.set( S::rfOutPwr, outPower );                                                       // (26)
  temp.set( S::rfOutEnable, E::ON );                                                       // (27)
  temp.set( S::auxOutPwr, auxPower );                                                      // (28)
  temp.set( S::auxOutEnable, E::OFF );                                                     // (29)
  constexpr enum FDBK { divided = 0, fundamental } Feedback = divided;
  temp.set( S::auxFBselect, Feedback );                                                    // (30)
  temp.set( S::muteTillLD, E::ON );                                                        // (31)
  temp.set( S::vcoPwrDown, E::OFF );                                                       // (32)
  constexpr auto BscClkDiv = round(fPFD / 125e3);
    static_assert( (0 < BscClkDiv) && (256 > BscClkDiv) ); // non-zero 8 bit value
  temp.set( S::bandSelectClkDiv, u8(BscClkDiv) );                                          // (33)
  temp.set( S::rfDivSelect, RfDivisorTableIndex );                                         // (34)
  temp.set( S::rfFBselect, !Feedback );  /* EEK! Why the negation?                         // (35)
    It works (for all possible divisors) NEGATED. I'm stumped. Perhaps I've been daVinci'd.     */
    // r5
  enum LedMode { low = 0, lockDetect = 1, high = 3 };
  temp.set( S::led_mode, LedMode::lockDetect );                           // ding. winner!    (36)
  //                                                                      // end taedium #two of 2
pll = temp;  /* save and exit scope (discarding temp). */ }
pll.flush();
wait4lock();  // that pretty blue led indicates phase lock. now, set phase at 180º.
pll.set( S::phase_adjust,E::ON ).set( S::phase,(Modulus >> 1) ).flush();
/* end setup() */ }
  // jettson[George]: "Jane! JANE! Stop this crazy thing! JANE! !!!"
auto loop() -> void {  }  //  73's, kd9fww.
