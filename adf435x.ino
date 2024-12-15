/* ©2024 kd9fww. ADF435x stand alone using Arduino Nano hardware SPI (in 300 lines, 7K memory).
  https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Where you got this code.
  https://www.analog.com/ADF4351 <- The device for which this code is specifically tailored.
  https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet
  US$30, for an assembled pll module from a major online discount household retail store.
  Bi-directional level shifter module assy., P/N: TXS0108E hereafter referred to as: Shfty;
  https://www.ti.com/lit/ds/symlink/txs0108e.pdf  No documentation is available for the (shifter
  chip + bypass cap) assembly. It's pinout is labeled. I acquired mine, and the Nano, for cheap
  from the same aforementioned company.
  ------------------------------------------------------------------------------------------------
  This code is a single pll version only. A second pll would be accommodated with it's {le, ld} on
  {D9, D8}, respectively. And sharing their {REF, 5v, clk, dat, pdr, GND} signals. With {3v3, mux}
  from one pll only. •This scheme does not preclude the possibilty of supporting two plls• See below.
  ------------------------------------------------------------------------------------------------
  A mechanism for runtime frequency and phase control is provided.
  --------------------------------------------------------------------------------------------- */
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
  ------------------------------------------------------------------------------------------------
  The LED (on D13) appears to be in contention with the default SPI clock line and is not easily
  open circuited. Look, 'Let It Be.' and 'Fughet about it.', OK? SPI will work regardless.
  ------------------------------------------------------------------------------------------------
  The scheme depicted makes it possible to power the system (Nano, Shfty, PLL) from these sources:
  1) USB, 2) The coaxial power connector on the pll assembly, 3) The Nano power pins, as above.
  A schottky diode (on the Nano) blocks current flowing to the USB host's 5V. Therefor, simul-
  taneous operation with USB and (one) external power supply, is not a (contention) issue.
  Note: avoid supplying the system power at a voltage near the Nano's 5V regultor dropout. Noise
  may cause gross modulation of the Nano 5V which in turn modulates the 3v3 to the extent of the
  3v3 regulator's line rejection. This effect is not present with the supply sufficiently above
  the 5V input dropout (or below the 5V input dropout but enough above 3v3 regulator dropout).
  A USB host can do 5V @ 500 mA. For debug, power from: USB, only; Benchmark: opt 3, >6V, only.
•BRICK• Don't exceed 5.5V for option 2 •BRICK• •Don't use options 2 AND 3• Man make fire. Ugh. */
  // Commented out, but wired:   D4                                      D11       D13 
enum class PIN : u8 {    /* MUX = 4, */ PDR = 6, LD = 7, LE = 10 /* DAT = 11, CLK = 13 */ };
const auto log2 = [](double arg) { return log10(arg) / log10(2); };
const auto wait4lock = []() { while( !digitalRead( static_cast<u8>(PIN::LD) )); }; // Busy wait.
enum Enable { OFF = 0, ON = 1 };  using E = Enable;
const auto rfHardEnable = [](E enable) { digitalWrite( static_cast<u8>(PIN::PDR), enable ); };
const auto txSPI = [](void *pByte, int nByte) {
  auto p = static_cast<u8*>(pByte) + nByte;       // Most significant BYTE first.
  digitalWrite( static_cast<u8>(PIN::LE), 0 );    // Predicate condition for data transfer.
  while( nByte-- ) SPI.transfer( *(--p) );        // Return value is ignored.
  digitalWrite( static_cast<u8>(PIN::LE), 1 ); }; // Data is latched on the rising edge.
enum Symbol : u8 {  // Human readable register 'field' identifiers.
    // In datasheet order. Enumerant names do NOT mirror datasheet's names exactly.
    fraction,     integer,      modulus,
    phase,        prescaler,    phase_adjust,
    counterReset, cp3state,     idle,
    pdPolarity,   ldp,          ldf,
    cpIndex,      dblBfr,       rCounter,
    refToggler,   refDoubler,   muxOut,
    LnLsModes,    clkDivider,   clkDivMode,
    csr,          chrgCancel,   abp,
    bscMode,      rfOutPwr,     rfSoftEnable,
    auxOutPwr,    auxOutEnable, auxFBselect,
    muteTillLD,   vcoPwrDown,   bandSelClkDiv,
    rfDivSelect,  rfFBselect,   led_mode,
    _end
  };  using S = Symbol;
struct Specification { const u8 RANK, OFFSET, WIDTH; }; /*
    N:      Number of (32 bit) "registers": 6
    RANK:   Datasheet Register Number = N - 1 - RANK
            txSPI() in ascending RANK order, unless not dirty. Thus, datasheet register '0' is
            always txSPI()'d last (and will always need to be txSPI()'d). See flush() below.
    OFFSET: Zero based position of the field's least significant bit.
    WIDTH:  Correct. The number of bits in a field (and is at least one). •You get a gold star• */
constexpr Specification ADF435x[] = { /*
  Human deduced via inspection of the datasheet. Unique to the ADF435x. */
  [S::fraction] = {5, 3, 12},     [S::integer] = {5, 15, 16},     [S::modulus] = {4, 3, 12},
  [S::phase] = {4, 15, 12},       [S::prescaler] = {4, 27, 1},    [S::phase_adjust] = {4, 28, 1},
  [S::counterReset] = {3, 3, 1},  [S::cp3state] = {3, 4, 1},      [S::idle] = {3, 5, 1},
  [S::pdPolarity] = {3, 6, 1},    [S::ldp] = {3, 7, 1},           [S::ldf] = {3, 8, 1},
  [S::cpIndex] = {3, 9, 4},       [S::dblBfr] = {3, 13, 1},       [S::rCounter] = {3, 14, 10},
  [S::refToggler] = {3, 24, 1},   [S::refDoubler] = {3, 25, 1},   [S::muxOut] = {3, 26, 3},
  [S::LnLsModes] = {3, 29, 2},    [S::clkDivider] = {2, 3, 12},   [S::clkDivMode] = {2, 15, 2},
  [S::csr] = {2, 18, 1},          [S::chrgCancel] = {2, 21, 1},   [S::abp] = {2, 22, 1},
  [S::bscMode] = {2, 23, 1},      [S::rfOutPwr] = {1, 3, 2},      [S::rfSoftEnable] = {1, 5, 1},
  [S::auxOutPwr] = {1, 6, 2},     [S::auxOutEnable] = {1, 8, 1},  [S::auxFBselect] = {1, 9, 1},
  [S::muteTillLD] = {1, 10, 1},   [S::vcoPwrDown] = {1, 11, 1},   [S::bandSelClkDiv] = {1, 12, 8},
  [S::rfDivSelect] = {1, 20, 3},  [S::rfFBselect] = {1, 23, 1},   [S::led_mode] = {0, 22, 2} };                                                  // End taedium #1 of two.
  static_assert(S::_end == (sizeof(ADF435x) / sizeof(ADF435x[0])));
struct StateParameters { u16 divis, whole, denom, numer, propo; };
  /* ©2024 kd9fww */
struct SpecifiedOverlay {
  static const Specification* spec;
  static StateParameters memory;
  struct Device {
    static constexpr auto N{ 6 };
    using RegArray = std::array<u32, N>; /*
      With the exception of r5 bits 19 and 20, all "reserved" bits are to be set to zero. These
      regions become 'invariants' by not providing fields for them in the Specification. As such,
      this mechanism adheres to the principle of 'Resource Aquisition Is Initialization' (RAII),
      via the containing class' constructor with an (embedded, fixed) initializer-list. See: "The
      C++ Programming Language". Fourth Edition. Stroustrup. 2013. §3.2.1.2, §3.2.1.3, §17.3.4 */
  u8 durty; SPISettings settings;RegArray reg; } dev =
  { 0, SPISettings(4000000, MSBFIRST, SPI_MODE0), Device::RegArray{ 0x180005,4,3,2,1,0 } };
  auto phaseAdjust( const E& e ) -> decltype(*this) { raw( S::phase_adjust,e ); return *this; }
      // usage: object.raw( symA,valA ).raw( symB,valB ) ••• ad infinitum
  auto raw( const S& symbol,const u16& value ) -> decltype(*this) {
    static constexpr u32 MASK[] = {
      0, 1, 3, 7, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383, 32767, 65535 };
    auto pSpec = &spec[ static_cast<const u8>( symbol ) ];
    dev.reg[pSpec->RANK] &= ( ~(        MASK[pSpec->WIDTH]   << pSpec->OFFSET) ); // First, off.
    dev.reg[pSpec->RANK] |= (  (value & MASK[pSpec->WIDTH] ) << pSpec->OFFSET  ); // Then, on.
    static constexpr u8 WEIGHT[] = { 1, 2, 4, 8, 16, 32 };
    dev.durty |= WEIGHT[ (dev.N - 1) - pSpec->RANK ]; // Encode which dev.reg was dirty'd.
    return *this; }
      // usage: object.set( symA,valA ).set( symB,valB ) ••• ad infinitum
  auto set( const S& sym,const u16& val ) -> decltype(*this) {  // wrapper for raw()
    switch(sym) {
      default: return raw( sym,val );
      case S::fraction:     if(val != memory.numer) return raw( sym,memory.numer = val ); break;
      case S::integer:      if(val != memory.whole) return raw( sym,memory.whole = val ); break;
      case S::phase:        if(val != memory.propo) return raw( sym,memory.propo = val ); break;
      case S::modulus:      if(val != memory.denom) return raw( sym,memory.denom = val ); break;
      case S::rfDivSelect:  if(val != memory.divis) return raw( sym,memory.divis = val ); break; 
     }  }
  auto set( const StateParameters& sp ) -> decltype(*this) {
    set( S::fraction,sp.numer ).set( S::integer,sp.whole );
    set( S::modulus,sp.denom ).set( S::phase,sp.propo );
    set( S::rfDivSelect,sp.divis );
    return *this;  }
  auto flush() -> void {  // When it is appropriate to do so, flush() 'it' to the target (pll).
    char cx{ 0 };
    switch( dev.durty ) { // Avoid the undirty'd. Well, almost.
      default: break;                   /* Otherwise: say they're all dirty. */
      case 0: return;                   /* None dirty. */
      case 1: cx = dev.N - 1; break;    /* r0 ••• */
      case 2: /* fall thru */           /* r1 ••• */
      case 3: cx = dev.N - 2; break;    /* r1 and r0 ••• */
      case 16: cx = dev.N - 4; break;   /* r4 ••• */ }
    dev.durty = 0;
    SPI.beginTransaction( dev.settings );
    for(/* empty */; dev.N != cx; ++cx) txSPI( &dev.reg[cx], sizeof(dev.reg[cx]) );
    SPI.endTransaction(); /* Works and plays well with others. 8-P */ }
}; /* End struct SpecifiedOverlay */
const Specification* SpecifiedOverlay::spec{ ADF435x };
constexpr StateParameters initialStateParameters{ 0, 0, 0, 0, 1 };
StateParameters SpecifiedOverlay::memory{ initialStateParameters };
  /* ©2024 kd9fww */
SpecifiedOverlay pll;
  constexpr auto  FLAG{ E::ON };
  constexpr auto  CONSTRAINT{ 1e1 };                // 'digit(s) lost' Assertion failure avoidance
  constexpr auto  USER_TRIM{ -13 * CONSTRAINT };    // Zero based, via human working in reverse,
  constexpr auto  REF_ERROR{ (FLAG) * USER_TRIM };  // from the 'REF' measurement, below.
  constexpr auto  OSC{ 25.000000e6 };               // Nominal ref. freq. Yours may be different.
  constexpr auto  REF{ OSC + REF_ERROR };           // Measured. YOURS WILL BE DIFFERENT.
  static_assert( 0 == (REF - OSC) - REF_ERROR, "Least significant digit(s) lost." );
  constexpr auto  MIN_PFD{ 125e3 }, MAX_PFD{ 045e6 }; // Manifest constants ...
  constexpr auto  MIN_VCO{ 2.2e9 }, MAX_VCO{ 4.4e9 }; // ... from the datasheet
  constexpr auto  MIN_FREQ{ MIN_VCO / 64 },  MAX_FREQ{ MAX_VCO };
  constexpr  u16  REF_COUNTER{ 8 };
  static_assert( (0 < REF_COUNTER) && (1024 > REF_COUNTER) );   // Non-zero, 10 bit value.
  constexpr auto  REF_TGLR{ E::OFF }, REF_DBLR{ REF_TGLR };     // OFF if OSC IS a 50% square wave
  constexpr auto  PFD = REF * (1 + REF_DBLR) / (1 + REF_TGLR) / REF_COUNTER;  // For completeness.
  static_assert((MIN_PFD <= PFD) && (MAX_PFD >= PFD));
class Marker {
  private:
    double pfd, step;
    StateParameters sp{ initialStateParameters };
  public:
  virtual ~Marker() {}
  explicit Marker(const double& _pfd, const double& _step) : pfd{_pfd}, step{_step} {}
  #undef degrees                                // "Good grief, Charlie Brown." C.M. Schulz.
  auto phase(double degrees) -> decltype(sp) {  // (degrees / 360) = (propo / (denom-1))
    degrees = { (360 < degrees) ? (360-degrees) : (0 > degrees) ? (360 + degrees) : degrees };
    auto proportion{ (degrees / 360 * (sp.denom - 1)) };
    sp.propo = u16( (proportion > sp.denom - 1) ? sp.denom - 1 : proportion );
    return sp;  }
  auto phase() -> double { return sp.propo / double(sp.denom - 1) * 360; }
  auto freq(double Hertz) -> decltype(sp) {
    Hertz = ((MIN_FREQ < Hertz) ? ((MAX_FREQ > Hertz) ? Hertz : MAX_FREQ) : MIN_FREQ);
    sp.divis = u16( floor( log2(MAX_VCO / Hertz) ) );
    auto fractional_N{ Hertz / pfd * pow(2, sp.divis) };
    sp.whole = u16( floor( fractional_N ) );
    sp.whole = (22 < sp.whole) ? sp.whole : 22;
    sp.denom = u16( round( pfd / step ) );
    sp.numer = u16( round( (fractional_N - sp.whole) * sp.denom) );
    return sp;  }
  using DBL = double;
  auto freq() -> DBL { return pfd * (sp.whole + DBL(sp.numer) / sp.denom) / pow(2,sp.divis); };
}; /* End class Marker */ Marker m( PFD, OSC / 5e3 );
  /* "... how shall I tell you the story?" And the King replied: "Start at the beginning. Proceed
     until the end. Then stop." Lewis Carroll. "Alice's Adventures in Wonderland". 1865. */
auto setup() -> void {
  SPI.begin();
  pinMode(static_cast<u8>(PIN::PDR), OUTPUT); // rf output enable. Lock is attainable disabled.
  rfHardEnable( E::ON );                      // For OnOffKeying (OOK) start with E::OFF.
  pinMode(static_cast<u8>(PIN::LE), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::LE), 1);  /* Latch on rising edge:
    To accomplish SPI, first LE(1 to 0), 'wiggle' the data line, then LE(0 to 1). See txSPI(). */
  pinMode(static_cast<u8>(PIN::LD), INPUT);   // Lock detect.
    /* digitalWrite(static_cast<u8>(PIN::MUX), INPUT_PULLUP); */
{ /* Enter another scope. */ SpecifiedOverlay temp; /* Setup a temporary, Specified Overlay.
  (Qty:S::_end) calls of set() are required, in any order. Be sure to flush() after saving.
  Four set() calls are made for each m.freq(double). So, S::_end - 4, remaining. */
  //                                         S::fraction, S::integer, S::modulus      (1) (2) (3)
  temp.set( S::phase, 1);                 // Adjust phase AFTER loop lock.         REDUNDANT? (4)
  temp.set( S::phase_adjust, E::OFF );                                                     // (5)
  enum PRSCL { four5ths = 0, eight9ths };
  // (75 < WHOLE) ? PRSCL::eight9ths : PRSCL::four5ths);
  temp.set( S::prescaler,PRSCL::eight9ths );                                               // (6)
  temp.set( S::counterReset, E::OFF );                                                     // (7)
  temp.set( S::cp3state, E::OFF );                                                         // (8)
  temp.set( S::idle, E::OFF );                                                             // (9)
  enum PDpolarity { negative = 0, positive };
  temp.set( S::pdPolarity, PDpolarity::positive );                                         // (10)
  enum LDPnS { ten = 0, six };            // Lock Detect Precision nanoSeconds
  temp.set( S::ldp, LDPnS::ten );                                                          // (11)
  enum LockDetectFunction{ fracN = 0, intN };
  temp.set( S::ldf, LockDetectFunction::fracN );                                           // (12)
  temp.set( S::cpIndex, 7 );  // 0 thru 15, 2.5mA = '7', more increases loop bandwidth.       (13)
  temp.set( S::dblBfr, E::ON );                                                            // (14)
  temp.set( S::rCounter, REF_COUNTER );                                                    // (15)
  temp.set( S::refToggler, REF_TGLR );                                                     // (16)
  temp.set( S::refDoubler, REF_DBLR );                                                     // (17)
  enum MuxOut { HiZ = 0, DVdd, DGnd, RcountOut, NdivOut, analogLock, digitalLock };
  temp.set( S::muxOut, MuxOut::HiZ );     // see 'cheat sheet'                                (18)
  constexpr enum NoiseSpurMode { lowNoise = 0, lowSpur = 3 } nsMode = lowNoise;
  //static_assert(( NoiseSpurMode::lowSpur == nsMode) ? (49 < MODULUS ? 1 : 0) : 1 );
  temp.set( S::LnLsModes, nsMode );                                                        // (19)
  constexpr auto CLKDIV32 = 150;          // I don't understand this YET.
  //= round( PFD / MODULUS * 400e-6 ); // from datasheets'
  // 'Phase Resync' text: tSYNC = CLK_DIV_VALUE × MOD × tPFD
  constexpr auto CLKDIV{ u16(CLKDIV32) };
  static_assert( (0 < CLKDIV) && (4096 > CLKDIV) ); // Non-zero, 12 bit value.
  temp.set( S::clkDivider, CLKDIV );                                                       // (20)
  enum ClockingMode { dividerOff = 0, fastLock, phResync };
  temp.set( S::clkDivMode, ClockingMode::dividerOff );                                     // (21)
  temp.set( S::csr, E::ON );              // Cycle Slip reduction                             (22)
  temp.set( S::chrgCancel, E::OFF );                                                       // (23)
  enum ABPnS { nS6fracN = 0, nS3intN };   // AntiBacklash Pulse nanoSeconds
  temp.set( S::abp, ABPnS::nS6fracN );                                                     // (24)
  enum BandSelMd { automatic = 0, programmed };
  temp.set( S::bscMode, (MIN_PFD < PFD) ? BandSelMd::programmed : BandSelMd::automatic );  // (25)
  constexpr enum dBm { minus4, minus1, plus2, plus5 } auxPower = minus4, outPower = plus5;
  temp.set( S::rfOutPwr, outPower );                                                       // (26)
  temp.set( S::rfSoftEnable, E::ON );                                                      // (27)
  temp.set( S::auxOutPwr, auxPower );                                                      // (28)
  temp.set( S::auxOutEnable, E::OFF );    // Signal unavailable. So, I can't test it.         (29)
  constexpr enum FDBK { divided = 0, fundamental } Feedback = divided;
  temp.set( S::auxFBselect, Feedback );                                                    // (30)
  temp.set( S::muteTillLD, E::ON );                                                        // (31)
  temp.set( S::vcoPwrDown, E::OFF );                                                       // (32)
  constexpr auto BscClkDiv = ceil(PFD / MIN_PFD);
  static_assert( (0 < BscClkDiv) && (256 > BscClkDiv) ); // Non-zero, 8 bit value.
  temp.set( S::bandSelClkDiv, u8(BscClkDiv) );                                             // (33)
                                          // S::rfDivSelect                                   (34)
  temp.set( S::rfFBselect, !Feedback );   /* EEK! Why the negation?                           (35)
    It works NEGATED. I'm stumped. Perhaps I've been daVinci'd. */
  enum LedMode { low = 0, lockDetect = 1, high = 3 };
  temp.set( S::led_mode, LedMode::lockDetect );                             // Ding. Winner!  (36)
pll = temp;  /* Save and exit scope (discarding temp). */ }
/* End setup() */ }
  void pr(   const u32& arg, int num = DEC) { Serial.print(arg,num); Serial.print(' '); };
  void pr(const double& arg, int num = 0  ) { Serial.print(arg,num); Serial.print(' '); };
  void pl(   const u32& arg, int num = DEC) { Serial.println(arg,num); };
  void pl(const double& arg, int num = 0  ) { Serial.println(arg,num); };
    // Jettson[George]: "Jane! JANE! Stop this crazy thing! JANE! !!!".
auto loop() -> void { double f0{ 65.4321e6 };//  Serial.begin(1000000L); delay(1000L);
pll.set(m.freq( f0 )).flush(); wait4lock();// pr(m.freq()); pl(m.freq()-f0);
  /*  Todo: A means to (physically) measure phase adjustment (with one pll, only). 
      It locks.                   I think it is correct.                  Use it as follows. */
    //  pll.phaseAdjust(E::ON).set(m.phase(270)).flush();  
        //  pr(m.phase()); pl(m.phase()-270);
while(1); }    /* { // Alternate loop(): (up-dn frequency sweep)
    auto df{ 5e3 * 1 }, f{ 34.5e6 - df };
    pll.set( m.freq(f += df) ).flush(); wait4lock();
    pr( m.freq() ); pl( m.freq() - f );
    if( (34.625e6 <= f) || (MIN_FREQ >= f) ) df = -df;
    delay(3000L); } */
    /*  kd9fww. Known for lotsa things. 'Gotcha' code isn't one of them. */
