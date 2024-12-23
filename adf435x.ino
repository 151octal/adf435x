/* ©2024 kd9fww. ADF435x stand alone using Arduino Nano hardware SPI (in ~350 lines, ~10K mem).
  https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Where you got this code.
  https://www.analog.com/ADF4351 <- The device for which this code is specifically tailored.
  https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet */
#include <SPI.h>
#include <ArxContainer.h>
  using ULL = unsigned long long;
  enum Enable { OFF = 0, ON = 1 };
  // Debug shorthand. Remove me prior to release.
  void pr( const char& cc ) { Serial.print(cc); }
  void ps( const char* const s ) { Serial.print(s); }
  void pr( const double& arg, int num = 0 ) { Serial.print(arg, num); pr(' '); }
  void pr( const u16& arg, int num = DEC ) { Serial.print(arg, num); pr(' '); }
  void pr( const u32& arg, int num = DEC ) { Serial.print(arg, num); pr(' '); }
  void pd( const char* const s, const double& arg, int num = 0 ) { ps(s), pr(arg,num); }
  void pr( const char* const s, const u16& arg, int num = DEC ) { ps(s); pr(arg,num); }
  void pb( const u32& arg, u8 nb = 32 ) { while( nb ) { switch(nb) {  // Print binary ...
      default: break; case 8: case 16: case 24: pr(' '); }            // in byte sized chunks,
    pr( ((1UL << --nb) & arg ) ? '1' : '0' ); } pr(' '); }            // one bit at a time.
#include <Arduino.h>
#include <BasicEncoder.h>
namespace Hardware {
  // See https://github.com/151octal/adf435x/blob/main/README.md for circuitry notes.
  enum PIN : u8 { eJ = 2, eK = 3, MUX = 4, PDR = 6, LD_A = 7, LE_A = 10 };
  enum UNIT { A, /* B, */ _end };
  constexpr struct CTRL { PIN le, ld; } ctrl[] = { [A] = { LE_A, LD_A } };/*
    , [B] = { LE_B, LD_B } };*/
  static_assert(UNIT::_end == sizeof(ctrl) / sizeof(ctrl[0]));
  const auto hardWait = [](const PIN& pin) { while( !digitalRead( static_cast<u8>(pin) )); };
  const auto rf = [](bool enable) { digitalWrite( static_cast<u8>(PIN::PDR), enable ); };
  const auto tx = [](const PIN& le, void *pByte, int nByte) {
    auto p = static_cast<u8*>(pByte) + nByte;       // Most significant BYTE first.
    digitalWrite( static_cast<u8>(le), 0 );         // Predicate condition for data transfer.
    while( nByte-- ) SPI.transfer( *(--p) );        // Return value is ignored.
    digitalWrite( static_cast<u8>(le), 1 ); };      /* Data is latched on the rising edge. */
} namespace HW =  Hardware; namespace Manifest  {   // Constants. Direct or derived from datasheet
  constexpr ULL   MAX_VCO{ 4400000000 };            // 4400 MHz. Unsigned long long.
  constexpr u32   MIN_VCO{ MAX_VCO / 2 };           // 2200 MHz.
  constexpr u32   MIN_PFD{ MIN_VCO / 17600 };       // 125 kHz.
  constexpr u32   MAX_PFD{ MIN_VCO / 50 };          // ≈45 MHz (Found in datasheet fine print).
  constexpr u32   MIN_FREQ{ MIN_VCO / 64 };         // 34375 kHz (64: maximum rf divider value).
  constexpr ULL   MAX_FREQ{ MAX_VCO };
/* End Manifest::  */ }     namespace Synthesis {   // Detailed because compilers ignore comments.
  constexpr auto  TGLR{ OFF }, DBLR{ TGLR };        // OFF: ONLY if OSC IS a 50% duty square wave
  constexpr auto  COMP{ ON };                       // OFF: No OSCillator error COMPensation.
; constexpr auto  CORRECTION{ -129 };               // Determined by working in reverse, from
  constexpr auto  REF_ERROR{ (COMP) * CORRECTION }; // the value of REF, as measured, below.
  constexpr auto  OSC{ 25000000U };                 // Nominal osc. freq. Yours may be different.
  constexpr auto  REF{ OSC + REF_ERROR };           // Measured osc. freq. Yours WILL BE different
   constexpr auto M0{ 5U }, M4{ M0*M0*M0*M0 };      
  constexpr enum  PICK { SML = 0, LRG } size = LRG; // Pick a modulus not divisible by {2,3}.
  constexpr auto  MOD{ size ? M4 * M0 : M4 };       // SML: Longer lock time.
    static_assert((4096>MOD) && (MOD%2) && (MOD%3));// 12 bits with spur avoidance
; constexpr  u16  STEP_SIZE{ 100 };                 // REF / Rcounter = PFD = Modulus * STEP_SIZE
  constexpr auto  REF_COUNTER{ u16(OSC / STEP_SIZE / MOD) };        // This quotient must have
    static_assert(REF_COUNTER * STEP_SIZE == OSC / MOD);            // a zero remainder.
    static_assert( (0 < REF_COUNTER) && (1024 > REF_COUNTER) );     // Non-zero, 10 bits.
  constexpr auto  PFD = REF * (1+DBLR) / (1+TGLR) / REF_COUNTER;    // Phase Frequency Detector
    static_assert(Manifest::MAX_PFD >= PFD);
 enum   ABPnS { nS6fracN = 0, nS3intN };            // AntiBacklash Pulse
  enum  Axis { FREQ = 0, PHAS, AMPL, STEP };
  enum  BSCmd { programmed = 0, automatic };        // Band Select Clock mode
  enum  ClockingMode { dividerOff = 0, fastLock, phResync };
  enum  dBm : u8 { minus4 = 0, minus1, plus2, plus5 }; 
  constexpr enum FDBK { divided = 0, fundamental } Feedback = divided;
  enum  LDPnS { ten = 0, six };                     // Lock Detect Precision
  enum  LEDmode { low = 0, lockDetect = 1, high = 3 };
  auto  log2(double arg) -> double { return log10(arg) / log10(2); };
  enum  LockDetectFunction{ fracN = 0, intN };
  enum  MuxOut { HiZ = 0, DVdd, DGnd, RcountOut, NdivOut, analogLock, digitalLock };
  enum  NoiseSpurMode { lowNoise = 0, lowSpur = 3 };
  enum  PRSCL { four5ths = 0, eight9ths };
  enum  PDpolarity { negative = 0, positive };
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
            tx() in ascending RANK order, unless not dirty. Thus, datasheet register '0' is
            always tx()'d last (and will always need to be tx()'d). See flush() below.
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
  static_assert(Symbol::_end == (sizeof(ADF435x) / sizeof(ADF435x[0])));
struct Parameters { u8 rfpwr, rfdiv; u16 denom, whole, numer, propo; };
  constexpr Parameters INIT{ 0, 0, 0, 0, 1, dBm::minus4 };
  constexpr auto OVERLAYED_REGISTERS{ 6 };
  /* ©2024 kd9fww */
class SpecifiedOvelay {
  private:
    HW::PIN le{ HW::ctrl[HW::UNIT::A].le }, ld{ HW::ctrl[HW::UNIT::A].ld };
    NoiseSpurMode nsMode = lowNoise;
    static const LayoutSpecification* const layoutSpec;
    Parameters store{ INIT };
    struct Overlay {
      static constexpr size_t N{ OVERLAYED_REGISTERS };
      using RegArray = std::array<u32, N>; /*
        With the exception of r5 bits 19 and 20, all 'reserved' bits are to be set to zero. These
        regions become 'invariants' by not providing fields for them in the Specification. */
    u8 durty; SPISettings settings;RegArray reg; };
    using OVL = Overlay;
    OVL ovl{ 0, SPISettings(4000000, MSBFIRST, SPI_MODE0), OVL::RegArray{ 0x180005,4,3,2,1,0} };
    auto raw( const S& symbol,const u16& value ) -> decltype(*this) {
      static constexpr u32 MASK[] = {
        0, 1, 3, 7, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383, 32767, 65535 };
      auto pSpec = &layoutSpec[ static_cast<const u8>( symbol ) ];
      ovl.reg[pSpec->RANK] &= ( ~(        MASK[pSpec->WIDTH]   << pSpec->OFFSET) ); // First, off.
      ovl.reg[pSpec->RANK] |= (  (value & MASK[pSpec->WIDTH] ) << pSpec->OFFSET  ); // Then, on.
      static constexpr u8 WEIGHT[] = { 1, 2, 4, 8, 16, 32 };
      ovl.durty |= WEIGHT[ (ovl.N - 1) - pSpec->RANK ]; // Encode which ovl.reg was dirty'd.
      return *this; }
  public:
  auto dump() -> void { 
    pr("rfpwr:",store.rfpwr); pr("rfdiv:",store.rfdiv); pr("denom:",store.denom);
    pr("whole:",store.whole); pr("numer:",store.numer); pr("propo:",store.propo); }
  auto flush() -> decltype(*this) {
    u8 cx{ 0 };
    switch( ovl.durty ) { // Avoid the undirty'd. Well, almost.
      default:  break;                    /* Otherwise: say they're all dirty. */
      case  0:  return *this;;            /* None dirty. */
      case  1:  cx = ovl.N - 1; break;    /* r0 ••• */
      case  2:  /* fall thru */           /* r1 ••• */
      case  3:  cx = ovl.N - 2; break;    /* r1 and r0 ••• */
      case 16:  cx = ovl.N - 4; break;    /* r4 ••• */ }
    ovl.durty = 0;
    SPI.beginTransaction( ovl.settings );
    for(/* empty */; ovl.N != cx; ++cx) HW::tx(le, &ovl.reg[cx], sizeof(ovl.reg[cx]) );
    SPI.endTransaction();
    return *this; }
  auto lock() -> void { HW::hardWait(ld); } // Wait on active ld pin, until lock is indicated.
  auto operator()( const HW::PIN _le, HW::PIN _ld ) -> decltype(*this) {
    le = _le; ld =_ld; return *this; }
  auto operator()( const HW::CTRL& io ) -> decltype(operator()(io.le, io.ld)) {
    return operator()(io.le, io.ld); }
    // Parameter Storage Intercept
  auto operator()( const S& sym,const u16& val ) -> decltype(*this) { switch(sym) {
    default:                return raw( sym,val );  // Beware of { case: fall thru }
    case S::fraction:     if(val !=  store.numer) {
                            return raw( sym,store.numer  = val ); }
                      else  return raw( sym,val );
    case S::integer:      if(val !=  store.whole) {
      raw( S::prescaler,(75 < val) ? PRSCL::eight9ths : PRSCL::four5ths );
                            return raw( sym,store.whole = val ); }
                      else  return raw( sym,val );
    case S::phase:        if(val !=  store.propo) {
                            return raw( sym,store.propo  = val ); }
                      else  return raw( sym,val );
    case S::modulus:      if(val !=  store.denom) {
      raw( S::LnLsModes,nsMode = (lowSpur == nsMode) ? ((50 > val) ? lowNoise : nsMode) : nsMode);
                            return raw( sym,store.denom = val ); }
                      else  return raw( sym,val );
    case S::rfDivSelect:  if(val !=  store.rfdiv) {
                            return raw( sym,store.rfdiv  = val ); }
                      else  return raw( sym,val );
    case S::rfOutPwr:     if(static_cast<dBm>(val) != store.rfpwr) { 
                            raw( S::rfSoftEnable, ON );
                            return raw( sym,store.rfpwr = static_cast<dBm>(val) ); }
                      else  return raw( sym,val );                                } }
    // Parameter dispatcher
  auto operator()( const Parameters& loci ) -> decltype(*this) {
    set( S::fraction,loci.numer ).set( S::integer,loci.whole ).set( S::modulus,loci.denom );
    set( S::phase,loci.propo ).set( S::rfDivSelect,loci.rfdiv ).set( S::rfOutPwr,loci.rfpwr );
    return *this;  }
  auto operator()() -> const decltype(store) { return store; }
  auto phAdj( const bool& e ) -> decltype(*this) { raw( S::phAdj,e ); return *this; }
    // Wrapper for operator()( sym,val )
  auto set( const S& sym,const u16& val ) -> decltype(*this) { return operator()( sym,val ); }
    // Wrapper for opertor()( loci )
  auto set( const Parameters& loci ) -> decltype(*this) { return operator()( loci ); }
} final; using SO = SpecifiedOvelay; const LayoutSpecification * const SO::layoutSpec{ ADF435x };
    /* ©2024 kd9fww */
class Marker {
  using DBL = double;
  private:
    // Rotating phasor: f(t) = |magnitude| * pow( Euleran, j( omega*t + phi ))
    // Where: Amplitude <- |magnitude|, Frequency <- omega, and Phase <- phi, are all scalars.
    Parameters loci{ INIT };
    u32 pfd; u16 spacing;
    auto amplitude() -> const u8 { return loci.rfpwr; }
    auto amplitude(const dBm& a) -> const decltype(loci) { loci.rfpwr = a; return loci; }
    auto delta() -> const decltype(spacing) { return spacing; }
    auto delta(const DBL& step) -> const decltype(loci) { spacing = step; return loci; }
    auto phi() -> const DBL { return (loci.propo / DBL(loci.denom - 1)); }
    auto phi(DBL normalized) -> const decltype(loci) {  // 'normalized' is a function scope copy.
        normalized = (0 > normalized) ? 0 : normalized;
        normalized = (1 < normalized) ? 1 : normalized;
        auto proportion{ u16(round(normalized * (loci.denom - 1))) };
        loci.propo = (1 > proportion) ? 1 : proportion;
        return loci; }
    auto omega() -> const DBL {
      return pfd * (loci.whole + DBL(loci.numer) / loci.denom) / pow(2,loci.rfdiv); }
    auto omega(DBL freq) -> decltype(loci) {            // 'freq' is a function scope copy.
      freq = (Manifest::MIN_FREQ < freq) ? freq : Manifest::MIN_FREQ;
      freq = (Manifest::MAX_FREQ > freq) ? freq : Manifest::MAX_FREQ;
      loci.rfdiv = u16( floor( log2(Manifest::MAX_VCO / freq) ) );
      auto fractional_N{ freq / pfd * pow(2, loci.rfdiv) };
      loci.whole = u16( floor( fractional_N ) );
      loci.whole = (22 < loci.whole) ? loci.whole : 22;
      loci.denom = u16( ceil( OSC / REF_COUNTER / spacing ) );
      loci.numer = u16( round( (fractional_N - loci.whole) * loci.denom) );
      return loci;  }
  public:
  Marker( const u32& actual_pfd = PFD, const u16& step = STEP_SIZE )
    : pfd{ actual_pfd }, spacing{ step } {}
  auto dump() -> void {
    pr("a:", operator()(AMPL)); pd("p:", operator()(PHAS),4); pd("f:", operator()()); }
  auto operator()(DBL arg, Axis axis = FREQ) -> const decltype(loci) { switch(axis) {
    default:
    case AMPL:  return amplitude(static_cast<dBm>(arg));
    case FREQ:  return omega(arg);
    case PHAS:  return phi(arg);
    case STEP:  return delta(arg); } }
      // Marker value dispatcher. Returns Axis selective value from Parameters
  const auto operator()(Axis axis = FREQ) -> const decltype(omega()) {
    switch (axis) {
      default:
      case AMPL:  return static_cast<double>(amplitude());
      case FREQ:  return omega();
      case PHAS:  return phi();
      case STEP:  return delta(); } } };
/* End Synthesis:: */ }
  /* "How shall I tell you the story?" The King replied, "Start at the beginning. Proceed
  until the end. Then stop." Lewis Carroll. "Alice's Adventures in Wonderland". 1865. */
auto setup() -> void { using namespace Hardware;    // "And, away we go ..." Gleason.
  SPI.begin();
  digitalWrite(static_cast<u8>(PIN::MUX), INPUT_PULLUP);  // Unused.
  pinMode(static_cast<u8>(PIN::PDR), OUTPUT);             // Rf output enable.
  rf( OFF );
  pinMode(static_cast<u8>(PIN::LE_A), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::LE_A), 1);
  pinMode(static_cast<u8>(PIN::LD_A), INPUT); /*
  pinMode(static_cast<u8>(PIN::LE_B), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::LE_B), 1);
  pinMode(static_cast<u8>(PIN::LD_B), INPUT); */ }
     // Jettson[George]: "Jane! JANE! Stop this crazy thing! JANE! !!!".
;auto loop() -> void { using namespace Synthesis;
  Serial.begin(1000000L); delay(1000L);
; SO pll;     { /* Enter another scope. */ SO temp; /*
  Quantiy S::_end calls of set() are required, in any order. */
  //                     S::fraction, S::integer, S::modulus S::rfDivSelect       (1) (2) (3) (34)
  temp( S::phase, 1);                     // Adjust phase AFTER loop lock. Not redundant.      (4)
  temp( S::phAdj, OFF );                                                                    // (5)
  temp( S::prescaler,PRSCL::eight9ths );  // Possiblly redundant                            // (6)
  temp( S::counterReset, OFF );                                                             // (7)
  temp( S::cp3state, OFF );                                                                 // (8)
  temp( S::idle, OFF );                                                                     // (9)
  temp( S::pdPolarity, PDpolarity::positive );                                             // (10)
  temp( S::ldp, LDPnS::ten );                                                              // (11)
  temp( S::ldf, LockDetectFunction::fracN );                                               // (12)
  temp( S::cpIndex, 7 );  // 0 thru 15, 2.5mA = '7', more increases loop bandwidth.           (13)
  temp( S::dblBfr, ON );                                                                   // (14)
  temp( S::rCounter, REF_COUNTER );                                                        // (15)
  temp( S::refToggler, TGLR );                                                             // (16)
  temp( S::refDoubler, DBLR );                                                             // (17)
  temp( S::muxOut, MuxOut::HiZ );         // see 'cheat sheet'                                (18)
  temp( S::LnLsModes, lowNoise );                                                          // (19)
  constexpr auto CLKDIV32 = 150;          // I don't understand this, YET.
  //= round( PFD / MOD * 400e-6 ); // from datasheets'
  // 'Phase Resync' text: tSYNC = CLK_DIV_VALUE × MOD × tPFD
  constexpr auto CLKDIV{ u16(CLKDIV32) };
  static_assert( (0 < CLKDIV) && (4096 > CLKDIV) ); // Non-zero, 12 bit value.
  temp( S::clkDivider, CLKDIV );                                                           // (20)
  temp( S::clkDivMode, ClockingMode::dividerOff );                                         // (21)
  temp( S::csr, ON );                     // Cycle Slip reduction                             (22)
  temp( S::chrgCancel, OFF );                                                              // (23)
  temp( S::abp, ABPnS::nS6fracN );                                                         // (24)
  temp( S::bscMode,(Manifest::MIN_PFD < PFD) ? BSCmd::automatic : BSCmd::programmed);      // (25)
  temp( S::rfOutPwr, minus4 );            // Possiblly redundant                           // (26)
  temp( S::rfSoftEnable, ON );                                                             // (27)
  temp( S::auxOutPwr, minus4 );                                                            // (28)
  temp( S::auxOutEnable, OFF );           // Pin not connected.                               (29)
  temp( S::auxFBselect, !Feedback );                                                       // (30)
  temp( S::muteTillLD, ON );                                                               // (31)
  temp( S::vcoPwrDown, OFF );                                                              // (32)
  auto BscClkDiv = ceil(double(PFD) / Manifest::MIN_PFD);
  // auto BscClk = double(REF) / REF_COUNTER / BscClkDiv;
  temp( S::bndSelClkDv, u8(BscClkDiv) );                                                   // (33)
  temp( S::rfFBselect, !Feedback );       /* EEK! Why the negation?                           (35)
  It works NEGATED. I'm stumped. Perhaps I've been daVinci'd. */
  temp( S::ledMode, LEDmode::lockDetect );                                 // Ding. Winner!   (36)
; pll = temp; } /* Save and exit scope (discarding temp). */
  Marker mk;
  // pll( HW::ctrl[HW::A] ); // Needed if 1 < sizeof(ctrl)
  pll(mk(dBm::plus2,AMPL)).phAdj(OFF).set(mk(0/360.,PHAS));
  HW::rf(ON);  //pll( HW::ctrl[HW::B] ).phAdj(ON).set(mk(180/360.,PHAS)).flush().lock() );
  bool dir{ 0 }, debug{ 0 };  constexpr u32 coarse{ 1000000 };
  u32 bot{ 34375000 }, top{ 100000000 }, f{ top }; auto fine{ coarse / 40 };
  while(1) {
    delay(2222);
    pll(mk( f )).flush().lock();
    if(debug) { pll.dump(); mk.dump(); pr(f); pr(f - mk()); pr('\n'); }
    f += dir ? fine : -fine;
    if(top < f) { dir = 0; } else if(bot > f) { dir = 1; } } } // kd9fww