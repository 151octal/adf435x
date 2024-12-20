/* ©2024 kd9fww. ADF435x stand alone using Arduino Nano hardware SPI (in ~300 lines, ~10K mem).
  https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Where you got this code.
  https://github.com/151octal/adf435x/blob/main/README.md <- Circuitry notes.
  https://www.analog.com/ADF4351 <- The device for which this code is specifically tailored.
  https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet */
#include <ArxContainer.h>
#include <SPI.h>
namespace System {
  // Shorthand for debugging with Serial.print()
  void pr(                  const char& cc) { Serial.print(cc); Serial.print(' '); }
  void pr(const    u16& arg, int num = DEC) { Serial.print(arg, num); pr(' '); };
  void pr(const    u32& arg, int num = DEC) { Serial.print(arg, num); pr(' '); };
  void pr(const double& arg, int num = 0  ) { Serial.print(arg, num); pr(' '); };
  enum PIN : u8 { encJ = 2, encK = 3, MUX = 4, PDR = 6, LD_A = 7, LE_A = 10 };
  using P = PIN;
  enum UNIT { A, /* B, */ _END };
  struct PLL_IO { PIN le, ld; };
  constexpr PLL_IO ctrl[] = { [A] = { LE_A, LD_A } };//, [B] = { LE_B, LD_B } };
  static_assert(_END == sizeof(ctrl) / sizeof(ctrl[0]));
  const auto wait = [](const PIN& ld) { while( !digitalRead( static_cast<u8>(ld) )); };
  const auto gate = [](bool enbl) { digitalWrite( static_cast<u8>(PIN::PDR), enbl ); };
  const auto txSPI = [](const PIN& le, void *pByte, int nByte) {
    auto p = static_cast<u8*>(pByte) + nByte;       // Most significant BYTE first.
    digitalWrite( static_cast<u8>(le), 0 );         // Predicate condition for data transfer.
    while( nByte-- ) SPI.transfer( *(--p) );        // Return value is ignored.
    digitalWrite( static_cast<u8>(le), 1 ); };      /* Data is latched on the rising edge. */
} namespace HW = System; namespace Synthesis {
  enum dBm : u8 { minus4, minus1, plus2, plus5 }; 
  enum PRSCL { four5ths = 0, eight9ths };
  enum NoiseSpurMode { lowNoise = 0, lowSpur = 3 };
enum Symbol : u8 {  // Human readable register 'field' identifiers.
    // In datasheet order. Enumerant names do NOT mirror datasheet's names exactly.
    fraction,     integer,      modulus,
    phase,        prescaler,    phAdj,
    counterReset, cp3state,     idle,
    pdPolarity,   ldp,          ldf,
    cpIndex,      dblBfr,       rCounter,
    refToggler,   refDoubler,   muxOut,
    LnLsModes,    clkDivider,   clkDivMode,
    csr,          chrgCancel,   abp,
    bscMode,      rfOutPwr,     rfSoftEnable,
    auxOutPwr,    auxOutEnable, auxFBselect,
    muteTillLD,   vcoPwrDown,   bndSelClkDv,
    rfDivSelect,  rfFBselect,   ledMode,
    _end
  };  using S = Symbol;
constexpr struct LayoutSpecification { const u8 RANK, OFFSET, WIDTH; } ADF435x[] = { /*
  Human deduced via inspection.
    N:      Number of (32 bit) "registers": 6
    RANK:   Datasheet Register Number = N - 1 - RANK
            txSPI() in ascending RANK order, unless not dirty. Thus, datasheet register '0' is
            always txSPI()'d last (and will always need to be txSPI()'d). See flush() below.
    OFFSET: Zero based position of the field's least significant bit.
    WIDTH:  Correct. The number of bits in a field (and is at least one). •You get a gold star• */
  [S::fraction] = {5, 3, 12},     [S::integer] = {5, 15, 16},     [S::modulus] = {4, 3, 12},
  [S::phase] = {4, 15, 12},       [S::prescaler] = {4, 27, 1},    [S::phAdj] = {4, 28, 1},
  [S::counterReset] = {3, 3, 1},  [S::cp3state] = {3, 4, 1},      [S::idle] = {3, 5, 1},
  [S::pdPolarity] = {3, 6, 1},    [S::ldp] = {3, 7, 1},           [S::ldf] = {3, 8, 1},
  [S::cpIndex] = {3, 9, 4},       [S::dblBfr] = {3, 13, 1},       [S::rCounter] = {3, 14, 10},
  [S::refToggler] = {3, 24, 1},   [S::refDoubler] = {3, 25, 1},   [S::muxOut] = {3, 26, 3},
  [S::LnLsModes] = {3, 29, 2},    [S::clkDivider] = {2, 3, 12},   [S::clkDivMode] = {2, 15, 2},
  [S::csr] = {2, 18, 1},          [S::chrgCancel] = {2, 21, 1},   [S::abp] = {2, 22, 1},
  [S::bscMode] = {2, 23, 1},      [S::rfOutPwr] = {1, 3, 2},      [S::rfSoftEnable] = {1, 5, 1},
  [S::auxOutPwr] = {1, 6, 2},     [S::auxOutEnable] = {1, 8, 1},  [S::auxFBselect] = {1, 9, 1},
  [S::muteTillLD] = {1, 10, 1},   [S::vcoPwrDown] = {1, 11, 1},   [S::bndSelClkDv] = {1, 12, 8},
  [S::rfDivSelect] = {1, 20, 3},  [S::rfFBselect] = {1, 23, 1},   [S::ledMode] = {0, 22, 2} };
  static_assert(S::_end == (sizeof(ADF435x) / sizeof(ADF435x[0])));
} namespace State { struct Parameters { u16 divis, whole, denom, numer, propo; u8 outpwr; };
  constexpr Parameters INIT{ 0,0,0,0,1,Synthesis::minus4 };
} enum Enable { OFF = 0, ON = 1 };  using E = Enable;
namespace Synthesis {
  /* ©2024 kd9fww */
class Overlay {
  private:
    System::PIN le, ld;
    NoiseSpurMode nsMode = lowNoise;
    static const LayoutSpecification* const layoutSpec;
    State::Parameters store{ State::INIT };
    struct Device {
      static constexpr auto N{ 6 };
      using RegArray = std::array<u32, N>; /*
        With the exception of r5 bits 19 and 20, all "reserved" bits are to be set to zero. These
        regions become 'invariants' by not providing fields for them in the Specification. As
        such, this mechanism adheres to the principle of 'Resource Aquisition Is Initialization',
        via the containing class' constructor with an (embedded, fixed) initializer-list. See:
        "The C++ Programming Language". Fourth Edition. Stroustrup. 2013.
        §3.2.1.2, §3.2.1.3, §17.3.4 */
    u8 durty; SPISettings settings;RegArray reg; } dev =
      { 0, SPISettings(4000000, MSBFIRST, SPI_MODE0), Device::RegArray{ 0x180005,4,3,2,1,0 } };
      // usage: object.raw( symA,valA ).raw( symB,valB ) ••• ad infinitum
    auto raw( const S& symbol,const u16& value ) -> decltype(*this) {
      static constexpr u32 MASK[] = { // Speed has priority over size. (exp2() - 1) <- Slow.
        0, 1, 3, 7, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383, 32767, 65535 };
      auto pSpec = &layoutSpec[ static_cast<const u8>( symbol ) ];
      dev.reg[pSpec->RANK] &= ( ~(        MASK[pSpec->WIDTH]   << pSpec->OFFSET) ); // First, off.
      dev.reg[pSpec->RANK] |= (  (value & MASK[pSpec->WIDTH] ) << pSpec->OFFSET  ); // Then, on.
      static constexpr u8 WEIGHT[] = { 1, 2, 4, 8, 16, 32 };
      dev.durty |= WEIGHT[ (dev.N - 1) - pSpec->RANK ]; // Encode which dev.reg was dirty'd.
      return *this; }
  public:
  auto flush() -> decltype(*this) {
    char cx{ 0 };
    switch( dev.durty ) { // Avoid the undirty'd. Well, almost.
      default:  break;                    /* Otherwise: say they're all dirty. */
      case  0:  return *this;;            /* None dirty. */
      case  1:  cx = dev.N - 1; break;    /* r0 ••• */
      case  2:  /* fall thru */           /* r1 ••• */
      case  3:  cx = dev.N - 2; break;    /* r1 and r0 ••• */
      case 16:  cx = dev.N - 4; break;    /* r4 ••• */ }
    dev.durty = 0;
    SPI.beginTransaction( dev.settings );
    for(/* empty */; dev.N != cx; ++cx) HW::txSPI(le, &dev.reg[cx], sizeof(dev.reg[cx]) );
    SPI.endTransaction();
    return *this; }
  auto lock() -> void { System::wait(ld); }
  auto phaseAdjust( const bool& e ) -> decltype(*this) { raw( S::phAdj,e ); return *this; }
  auto operator()( const System::PIN& _le, System::PIN& _ld ) -> void { le = _le; ld =_ld; }
  auto operator()( const System::PLL_IO& io ) -> void { le = io.le; ld = io.ld; }
    // Parameter Store interceptor for Symbol dispatch. usage:
    // object( symA,valA ).operator()( symB,valB ) ••• ad infinitum
  auto operator()( const S& sym,const u16& val ) -> decltype(*this) { switch(sym) {
      default: return raw( sym,val );
      case S::fraction:     if(val !=  store.numer) return raw( sym,store.numer  = val ); break;
      case S::integer:      if(val !=  store.whole) { // S::prescaler side effect
        raw( S::prescaler,(75 < val) ? PRSCL::eight9ths : PRSCL::four5ths );
        return raw( sym,store.whole  = val ); break; }
      case S::phase:        if(val !=  store.propo) return raw( sym,store.propo  = val ); break;
      case S::modulus:      if(val !=  store.denom) { // S::LnLsModes side effect
        raw( S::LnLsModes,nsMode =
        (lowSpur == nsMode) ? ((50 > val) ? lowNoise : nsMode) : nsMode);
        return raw( sym,store.denom  = val ); break; }
      case S::rfDivSelect:  if(val !=  store.divis) return raw( sym,store.divis  = val ); break;
      case S::rfOutPwr:     if(static_cast<dBm>(val) != store.outpwr){// S::rfSoftEnable side effect
        raw( S::rfSoftEnable, E::ON );
        return raw( sym,store.outpwr = static_cast<dBm>(val) );   break; } } }
    // Usage: object( loci ).operator()( loci ) ••• ad infinitum
  auto operator()( const State::Parameters& loci ) -> decltype(*this) {
    operator()( S::fraction,loci.numer ).operator()( S::integer,loci.whole );
    operator()( S::modulus,loci.denom ).operator()( S::phase,loci.propo );
    operator()( S::rfDivSelect,loci.divis ).operator()( S::rfOutPwr,loci.outpwr );
    return *this;  }
    // Wrapper for operator()(State::Parameters)
  auto set( const State::Parameters& loci ) -> decltype(*this) { return operator()( loci ); }
  } final; using OVL = Overlay;
const LayoutSpecification * const Overlay::layoutSpec{ ADF435x }; 
} namespace Manifest {
    // PFD: Phase Frequency Detector                          // Manifest data ...
    constexpr auto  MIN_PFD{ 125e3 }, MAX_PFD{ 045e6 };       // (in units of Hertz)
    constexpr auto  MIN_VCO{ 2.2e9 }, MAX_VCO{ 4.4e9 };       // ... from the datasheet
    constexpr auto  MIN_FREQ{ MIN_VCO / 64 },  MAX_FREQ{ MAX_VCO };
} namespace Synthesis {
  constexpr  u16  REF_COUNTER{ 8 };                 // Use 80 for 10e6 = OSC.
  static_assert( (0 < REF_COUNTER) && (1024 > REF_COUNTER) ); // Non-zero, 10 bit value.
; constexpr auto  RESOLUTION{ 5e3 };
  constexpr auto  REF_TGLR{ E::ON };                // OFF: Only IFF OSC IS a 50% square wave.
  constexpr auto  REF_DBLR{ REF_TGLR };
  constexpr auto  COMP{ E::ON };                    // OFF: No OSCillator error COMPensation.
  constexpr auto  CONSTRAINT{ 1e1 };                // 'digit(s) lost' Assertion failure avoidance
  constexpr auto  CORRECTION{ -13 * CONSTRAINT };   // Determined by working in reverse, from the
  constexpr auto  REF_ERROR{ (COMP) * CORRECTION }; // value of REF, as measured, below.
  constexpr auto  OSC{ 25e6 };                      // Nominal osc. freq. Yours may be different.
  constexpr auto  REF{ OSC + REF_ERROR };           // Measured osc. freq. YOURS WILL BE DIFFERENT
  static_assert( 0 == (REF - OSC) - REF_ERROR, "Least significant digit(s) lost." );
  constexpr auto  PFD = REF * (1 + REF_DBLR) / (1 + REF_TGLR) / REF_COUNTER;
  // Important: REF_ERROR is propagated by it's inclusion in the calculation of PFD <- actual_pfd
  static_assert( (Manifest::MIN_PFD <= PFD) && (Manifest::MAX_PFD >= PFD) );
enum Axis { FREQ, PHAS, AMPL, STEP };
  /* ©2024 kd9fww */
class Marker {
  using DBL = double;
  private:
    DBL pfd, spacing;
    State::Parameters loci{ State::INIT };
    static auto log2(DBL arg) -> DBL { return log10(arg) / log10(2); };
    // Rotating phasor: f(t) = |magnitude| * pow( euleran, j( omega*t + phi(t) ))
    //  Where: Amplitude <- |magnitude|, Frequency <- omega, Phase <- phi.
    auto amplitude() -> const u8 { return loci.outpwr; }
    auto amplitude(const dBm& a) -> const decltype(loci) { loci.outpwr = a; return loci; }
    auto delta() -> const decltype(spacing) { return spacing; }
    auto delta(const DBL& step) -> const decltype(loci) { spacing = step; return loci; }
    auto phi() -> const DBL { return (loci.propo / DBL(loci.denom - 1)); }
      #ifdef degrees
      #define execrable_macro_name
      #endif  // "What a bummer, eh?" Ian Anderson. "Good grief, Charlie Brown." C.M. Schulz.
    auto phi(DBL normalized) -> const decltype(loci){ // nomalized is a function scope copy.
        normalized = (0 > normalized) ? 0 : normalized;
        normalized = (1 < normalized) ? 1 : normalized;
        auto proportion{ u16(round(normalized * (loci.denom - 1))) };
        loci.propo = (1 > proportion) ? 1 : proportion;
        return loci; }
    auto omega() -> const DBL {
      return pfd * (loci.whole + DBL(loci.numer) / loci.denom) / pow(2,loci.divis); }
    auto omega(DBL freq) -> decltype(loci) {          // freq is a function scope copy.
      freq = (Manifest::MIN_FREQ < freq) ? freq : Manifest::MIN_FREQ;
      freq = (Manifest::MAX_FREQ > freq) ? freq : Manifest::MAX_FREQ;
      loci.divis = u16( floor( log2(Manifest::MAX_VCO / freq) ) );
      auto fractional_N{ freq / pfd * pow(2, loci.divis) };
      loci.whole = u16( floor( fractional_N ) );
      loci.whole = (22 < loci.whole) ? loci.whole : 22;
      loci.denom = u16( round( pfd / spacing ) );
      loci.numer = u16( round( (fractional_N - loci.whole) * loci.denom) );
      return loci;  }
  public:
  Marker( const DBL& actual_pfd, const DBL& step )
    : pfd{ actual_pfd }, spacing{ step } {}
  auto dump() -> void {
    using namespace System;
    pr(operator()()); pr(operator()(AMPL)); pr(operator()(PHAS),3); pr('\n'); }
      // Marker argument dispatcher. Returns Axis selective loci of State::Parameters
  auto operator()(DBL arg, Axis axis = FREQ) -> const decltype(loci) { switch(axis) {
    default:
    case FREQ:  return omega(arg);
    case PHAS:  return phi(arg);
    case STEP:  return delta(arg);
    case AMPL:  return amplitude(static_cast<dBm>(arg)); } }
      // Marker value dispatcher. Returns Axis selective value from State::Parameters
  const auto operator()(Axis axis = FREQ) -> const decltype(omega()) {
    switch (axis) {
      default:
      case FREQ:  return omega();
      case PHAS:  return phi();
      case STEP:  return delta();
      case AMPL:  return static_cast<double>(amplitude()); } }
      // Wrapper for operator()(DBL,Axis)
  const auto set(DBL arg, Axis axis = FREQ) -> const decltype(loci) {
    return operator()(arg, axis); } }; }
    /* "... how shall I tell you the story?" The King replied, "Start at the beginning. Proceed
    until the end. Then stop." Lewis Carroll. "Alice's Adventures in Wonderland". 1865. */
auto setup() -> void {  // "And away we go." Gleason.
  using namespace System;
  SPI.begin();
  /* digitalWrite(static_cast<u8>(PIN::MUX), INPUT_PULLUP); */
  pinMode(static_cast<u8>(PIN::PDR), OUTPUT); // Rf output enable.
  gate( E::OFF );
  pinMode(static_cast<u8>(PIN::LE_A), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::LE_A), 1);
  pinMode(static_cast<u8>(PIN::LD_A), INPUT); }
     // Jettson[George]: "Jane! JANE! Stop this crazy thing! JANE! !!!".
auto loop() -> void {
  Serial.begin(1000000L); delay(1000L);
  using namespace Synthesis;
; OVL pll;    { /* Enter another scope. */ OVL temp; /*
  Quantiy S::_end calls of set() are required, in any order. Four set() calls are made for each
  m.freq(double). So, S::_end - 4, remaining. Be sure to flush() after saving. */
  //                                         S::fraction, S::integer, S::modulus       (1) (2) (3)
  temp( S::phase, 1);                     // Adjust phase AFTER loop lock. Not redundant.      (4)
  temp( S::phAdj, E::OFF );                                                                 // (5)
  temp( S::prescaler,PRSCL::eight9ths );  // Possiblly redundant                            // (6)
  temp( S::counterReset, E::OFF );                                                          // (7)
  temp( S::cp3state, E::OFF );                                                              // (8)
  temp( S::idle, E::OFF );                                                                  // (9)
  enum PDpolarity { negative = 0, positive };
  temp( S::pdPolarity, PDpolarity::positive );                                             // (10)
  enum LDPnS { ten = 0, six };            // Lock Detect Precision nanoSeconds
  temp( S::ldp, LDPnS::ten );                                                              // (11)
  enum LockDetectFunction{ fracN = 0, intN };
  temp( S::ldf, LockDetectFunction::fracN );                                               // (12)
  temp( S::cpIndex, 7 );  // 0 thru 15, 2.5mA = '7', more increases loop bandwidth.           (13)
  temp( S::dblBfr, E::ON );                                                                // (14)
  temp( S::rCounter, Synthesis::REF_COUNTER );                                             // (15)
  temp( S::refToggler, Synthesis::REF_TGLR );                                              // (16)
  temp( S::refDoubler, Synthesis::REF_DBLR );                                              // (17)
  enum MuxOut { HiZ = 0, DVdd, DGnd, RcountOut, NdivOut, analogLock, digitalLock };
  temp( S::muxOut, MuxOut::HiZ );     // see 'cheat sheet'                                    (18)
  temp( S::LnLsModes, lowNoise );                                                          // (19)
  constexpr auto CLKDIV32 = 150;          // I don't understand this, YET.
  //= round( PFD / MODULUS * 400e-6 ); // from datasheets'
  // 'Phase Resync' text: tSYNC = CLK_DIV_VALUE × MOD × tPFD
  constexpr auto CLKDIV{ u16(CLKDIV32) };
  static_assert( (0 < CLKDIV) && (4096 > CLKDIV) ); // Non-zero, 12 bit value.
  temp( S::clkDivider, CLKDIV );                                                           // (20)
  enum ClockingMode { dividerOff = 0, fastLock, phResync };
  temp( S::clkDivMode, ClockingMode::dividerOff );                                         // (21)
  temp( S::csr, E::ON );                  // Cycle Slip reduction                             (22)
  temp( S::chrgCancel, E::OFF );                                                           // (23)
  enum ABPnS { nS6fracN = 0, nS3intN };   // AntiBacklash Pulse nanoSeconds
  temp( S::abp, ABPnS::nS6fracN );                                                         // (24)
  enum BndSelClkMd { automatic = 0, programmed };
  temp( S::bscMode,
  (Manifest::MIN_PFD < Synthesis::PFD) ? BndSelClkMd::programmed : BndSelClkMd::automatic);// (25)
  temp( S::rfOutPwr, minus4 );            // Possiblly redundant                           // (26)
  temp( S::rfSoftEnable, E::OFF );                                                         // (27)
  temp( S::auxOutPwr, minus4 );                                                            // (28)
  temp( S::auxOutEnable, E::OFF );        // Pin not connected. So, I can't test it.          (29)
  constexpr enum FDBK { divided = 0, fundamental } Feedback = divided;
  temp( S::auxFBselect, Feedback );                                                        // (30)
  temp( S::muteTillLD, E::ON );                                                            // (31)
  temp( S::vcoPwrDown, E::OFF );                                                           // (32)
  constexpr auto BscClkDiv = ceil(Synthesis::PFD / Manifest::MIN_PFD);
  static_assert( (0 < BscClkDiv) && (256 > BscClkDiv) ); // Non-zero, 8 bit value.
  temp( S::bndSelClkDv, u8(BscClkDiv) );                                                   // (33)
  // S::rfDivSelect                                                                           (34)
  temp( S::rfFBselect, !Feedback );   /* EEK! Why the negation?                               (35)
  It works NEGATED. I'm stumped. Perhaps I've been daVinci'd. */
  enum LEDmode { low = 0, lockDetect = 1, high = 3 };
  temp( S::ledMode, LEDmode::lockDetect );                                 // Ding. Winner!   (36)
; pll = temp; } /* Save and exit scope (discarding temp). */
  Marker m( PFD, RESOLUTION );
  using namespace System;
  pll( ctrl[A] );
  auto F{ 99.5e6 }, dF{ 3.125e3 };
  F -= dF;
  pll( m(minus1,AMPL) ).set( m(F) ).set( m(0/360.,PHAS) ).phaseAdjust(E::OFF).flush().lock();
  gate(E::ON);
  pr(' '); m.dump();
  //pll( ctrl[B] ); pll( m(270/360.,PHAS) ).flush().lock(); pr(' '); m.dump(); pll( ctrl[A] );
  bool dir{ true };
; while(1) {
    if(100e6 < F) dir = false;
    else if (Manifest::MIN_FREQ > F) dir = true;
    pll(m( F += (true == dir) ? dF : -dF )).flush().lock(); pr(' '); m.dump();
    delay(100); } } // kd9fww