/*  ©2024 kd9fww. ADF435x stand alone using Arduino Nano hardware SPI (in ~450 lines, ~20k mem).
    https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Where you got this code.
    https://www.analog.com/ADF4351 <- The device for which this code is specifically tailored.
    https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet */
  #include <Adafruit_GFX.h>
  #include <Arduino.h>
  #include <ArxContainer.h>
  #include <BasicEncoder.h>
  #include "OakOLED.h"
  #include <SPI.h>
  #include <Wire.h>
#define DEBUG
  #undef DEBUG
; using u64 = unsigned long long;
  using DBL = double;
  using OLED = OakOLED;
  enum Enable { OFF = 0, ON = 1 };
    #ifdef DEBUG  // Debug shorthand.
  void pr( const char& cc ) { Serial.print(cc); }
  void pr( const u8& uc ) { Serial.print(uc); }
  void pr( const char* const s ) { Serial.print(s); }
  void pr( const DBL& arg, int num = 0 ) { Serial.print(arg, num); pr(' '); }
  void pr( const u16& arg, int num = DEC ) { Serial.print(arg, num); pr(' '); }
  void pr( const u32& arg, int num = DEC ) { Serial.print(arg, num); pr(' '); }
  void pd( const char* const s, const DBL& arg, int num = 0 ) { pr(s), pr(arg,num); }
  void pr( const char* const s, const u16& arg, int num = DEC ) { pr(s); pr(arg,num); }
 /* void pb( const u32& arg, u8 nb = 32 ) { while( nb ) { switch(nb) {  // Print binary ...
      default: break; case 8: case 16: case 24: pr(' '); }            // in byte sized chunks,
    pr( ((1UL << --nb) & arg ) ? '1' : '0' ); } pr(' '); }            // one bit at a time. */
      #endif
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
} namespace HW = Hardware; namespace Synthesis {
 enum   ABPnS { nS6fracN = 0, nS3intN };            // AntiBacklash Pulse
  enum  Axis : size_t { AMPL, FREQ, DF, PHAS, DP };
  enum  BSCmd { programmed = 0, automatic };        // Band Select Clock mode
  enum  ClockingMode { dividerOff = 0, fastLock, phResync };
  enum  dBm : u8 { minus4 = 0, minus1, plus2, plus5 };
  enum  Direction : char { up, dn, left, rght };
  constexpr enum  FDBK { divided = 0, fundamental } Feedback = divided;
  enum  LDPnS { ten = 0, six };                     // Lock Detect Precision
  enum  LEDmode { low = 0, lockDetect = 1, high = 3 };
  auto  log2(const DBL& arg) -> DBL { return log10(arg) / log10(2); };
  enum  LockDetectFunction{ fracN = 0, intN };
  enum  MuxOut { HiZ = 0, DVdd, DGnd, RcountOut, NdivOut, analogLock, digitalLock };
  enum  NoiseSpurMode { lowNoise = 0, lowSpur = 3 };
  constexpr auto  OVERLAYED_REGISTERS{ 6 };
  enum  PRSCL { four5ths = 0, eight9ths };
  enum  PDpolarity { negative = 0, positive };
  auto  power(u8 radix, u8 exponent) -> const u64 { // radix raised to exponent
          u64 rv{1}; for(auto ix{exponent}; ix; --ix) rv *= radix; return rv; }
 constexpr  u64   kHz{ 1000 }, MHz{ 1000*kHz }, GHz{ 1000*MHz }, bottom{ 34*MHz + 375*kHz };
  constexpr u64   MAX_VCO{ 4400000000 };            // 4400 MHz.
  constexpr  u32  MIN_VCO{ MAX_VCO / 2 };           // 2200 MHz.
  constexpr  u32  MIN_PFD{ MIN_VCO / 17600 };       // 125 kHz.
  constexpr  u32  MAX_PFD{ MIN_VCO / 50 };          // ≈45 MHz (Found in datasheet fine print).
  constexpr  u32  MIN_FREQ{ MIN_VCO / 64 };         // 34375 kHz (64: maximum rf divider value).
  constexpr auto  MAX_FREQ{ MAX_VCO };
  constexpr auto  M0{ 5U }, M4{ M0*M0*M0*M0 };      // 5 raised to the fourth.
  constexpr enum  PICK { SML = 0, LRG } size = SML; // SML: Longer lock time.
  constexpr auto  MOD{ size ? M4 * M0 : M4 };       // Pick a modulus (not divisible by {2,3}).
  static_assert((4096>MOD) && (MOD%2) && (MOD%3));  // 12 bits with spur avoidance.
  constexpr auto  OSC{ 25000000U };                 // Nominal osc. freq. Yours may be different.
 constexpr   u16  IOTA{ 1000 };                     // IOTA such that the following are exact.
  constexpr auto  R_COUNT{ u16(OSC / IOTA / MOD) }; // REF / Rcounter = PFD = Modulus * IOTA
  constexpr auto  COMP{ ON };                       // OFF: No OSCillator error COMPensation.
 constexpr  auto  CORRECTION{ -420 };               // Determined by working in reverse, from
  constexpr auto  REF_ERROR{ (COMP) * CORRECTION }; // the value of REF, as measured, next line.
  constexpr auto  REF{ OSC + REF_ERROR };           // Measured reference oscillator frequency.
  constexpr auto  TGLR{ OFF }, DBLR{ TGLR };        // OFF: ONLY if OSC is a 50% duty square wave.
 constexpr  auto  PFD{ DBL(REF) * (1+DBLR) / (1+TGLR) / R_COUNT };
  static_assert(R_COUNT * IOTA == OSC / MOD);       // No remainder.
  static_assert((0<R_COUNT) && (1024>R_COUNT));     // Non-zero, 10 bits.
  static_assert((MAX_PFD >= PFD));// && (PFD * R_COUNT == REF));
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
constexpr struct LayoutSpecification { const u8 RANK, OFFSET, WIDTH; } ADF435x[] {  /*
  Human deduced via inspection.
    OVERLAYED_REGISTERS:  Number of (32 bit) "registers".
    RANK:                 RANK = OVERLAYED_REGISTERS - 1 - Datasheet Register Number.
                          tx() in ascending RANK order, unless not dirty. Thus, datasheet
                          register '0' is always tx()'d last (and will always need to be tx()'d).
                          See flush() below.
    OFFSET:               Zero based position of the field's least significant bit.
    WIDTH:                Correct. The number of bits in a field (and is at least one).
                          •You get a gold star• */
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
struct State { u8 rpwr, rdiv; u16 dnom, whol, numr, prop; };
  constexpr State INIT{ 0, 0, 0, 0, 1, dBm::minus4 };
  /* ©2024 kd9fww */
class SpecifiedOverlay {
  private:
    HW::PIN le{ HW::ctrl[HW::UNIT::A].le }, ld{ HW::ctrl[HW::UNIT::A].ld };
    NoiseSpurMode nsMode = lowNoise;
    static const LayoutSpecification* const layoutSpec;
    State store{ INIT };
    struct Overlay {
      static constexpr size_t NR{ OVERLAYED_REGISTERS };
      using RegArray = std::array<u32, NR>; /*
        With the exception of r5 bits 19 and 20, all 'reserved' bits are to be set to zero. These
        regions become 'invariants' by not providing fields for them in the Specification. */
    u8 durty; SPISettings settings; RegArray reg; };
    using OVL = Overlay;
    OVL ovl{ 0, SPISettings(4000000, MSBFIRST, SPI_MODE0), OVL::RegArray{ 0x180005,4,3,2,1,0} };
    auto raw( const S& symbol,const u16& value ) -> decltype(*this) {
      static constexpr u32 MASK[] = {
        0, 1, 3, 7, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383, 32767, 65535 };
      auto pSpec = &layoutSpec[ static_cast<const u8>( symbol ) ];
      ovl.reg[pSpec->RANK] &= ( ~(        MASK[pSpec->WIDTH]   << pSpec->OFFSET) ); // First, off.
      ovl.reg[pSpec->RANK] |= (  (value & MASK[pSpec->WIDTH] ) << pSpec->OFFSET  ); // Then, on.
      static constexpr u8 WEIGHT[] = { 1, 2, 4, 8, 16, 32 };
      ovl.durty |= WEIGHT[ (ovl.NR - 1) - pSpec->RANK ]; // Encode which ovl.reg was dirty'd.
      return *this; }
  public:
  auto flush() -> decltype(*this) {
    u8 cx{ 0 };
    switch( ovl.durty ) { // Avoid the undirty'd. Well, almost.
      default:  break;                    /* Otherwise: say they're all dirty. */
      case  0:  return *this;;            /* None dirty. */
      case  1:  cx = ovl.NR - 1; break;    /* r0 ••• */
      case  2:  /* fall thru */           /* r1 ••• */
      case  3:  cx = ovl.NR - 2; break;    /* r1 and r0 ••• */
      case 16:  cx = ovl.NR - 4; break;    /* r4 ••• */ }
    ovl.durty = 0;
    SPI.beginTransaction( ovl.settings );
    for(/* empty */; ovl.NR != cx; ++cx) HW::tx(le, &ovl.reg[cx], sizeof(ovl.reg[cx]) );
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
    case S::fraction:     if(val !=  store.numr) {
                            return raw( sym,store.numr  = val ); }
                      else  return raw( sym,val );
    case S::integer:      if(val !=  store.whol) {
      raw( S::prescaler,(75 < val) ? PRSCL::eight9ths : PRSCL::four5ths );
                            return raw( sym,store.whol = val ); }
                      else  return raw( sym,val );
    case S::phase:        if(val !=  store.prop) {
                            return raw( sym,store.prop  = val ); }
                      else  return raw( sym,val );
    case S::modulus:      if(val !=  store.dnom) {
      raw( S::LnLsModes,nsMode = (lowSpur == nsMode) ? ((50 > val) ? lowNoise : nsMode) : nsMode);
                            return raw( sym,store.dnom = val ); }
                      else  return raw( sym,val );
    case S::rfDivSelect:  if(val !=  store.rdiv) {
                            return raw( sym,store.rdiv  = val ); }
                      else  return raw( sym,val );
    case S::rfOutPwr:     if(static_cast<dBm>(val) != store.rpwr) { 
                            raw( S::rfSoftEnable, ON );
                            return raw( sym,store.rpwr = static_cast<dBm>(val) ); }
                      else  return raw( sym,val );                                } }
      // Parameter dispatcher
  auto operator()( const State& loci ) -> decltype(*this) {
    set( S::fraction,loci.numr ).set( S::integer,loci.whol ).set( S::modulus,loci.dnom );
    set( S::phase,loci.prop ).set( S::rfDivSelect,loci.rdiv ).set( S::rfOutPwr,loci.rpwr );
    return *this;  }
  auto operator()() -> const decltype(store) { return store; }
  auto phAdj( const bool& e ) -> decltype(*this) { raw( S::phAdj,e ); return *this; }
    // Wrapper for operator()( sym,val )
  auto set( const S& sym,const u16& val ) -> decltype(*this) { return operator()( sym,val ); }
    // Wrapper for opertor()( loci )
  auto set( const State& loci ) -> decltype(*this) { return operator()( loci ); }
} final; const LayoutSpecification * const SpecifiedOverlay::layoutSpec{ ADF435x };
    /* ©2024 kd9fww */
class Resolver {
  // Rotating phasor: f(t) = |magnitude| * pow( Euleran, j( omega*t + phi ))
  // Where: Amplitude <- |magnitude|, Frequency <- omega, and Phase <- phi, are all scalars.
  private:
  State loci{ INIT };
  DBL pfd; u16 spacing;
  auto amplitude() -> const u8 { return loci.rpwr; }
  auto amplitude(const dBm& a) -> const decltype(loci) { loci.rpwr = a; return loci; }
  auto phi() -> const DBL { return (loci.prop / DBL(loci.dnom - 1)); }
  auto phi(DBL normalized) -> const decltype(loci) {
      normalized = constrain((0 > normalized) ? -normalized : normalized, 0, 1);
      auto proportion{ u16(round(normalized * (loci.dnom - 1))) };
      loci.prop = (1 > proportion) ? 1 : proportion;
      return loci; }
  auto omega() -> const u64 {
    return u64(pfd) * (loci.whol + DBL(loci.numr) / loci.dnom) / pow(2,loci.rdiv); }
  auto omega(const u64& bn) -> const decltype(loci) {
    auto freq{ constrain(bn, MIN_FREQ, MAX_FREQ) };
    loci.rdiv = u16( floor( log2(MAX_VCO/freq) ) );
    auto fractional_N{ (freq / pfd) * pow(2, loci.rdiv) };
    loci.whol = u16( trunc( fractional_N ) );
    //loci.whol = (22 < loci.whol) ? loci.whol : 22;
    loci.dnom = u16( ceil( OSC / R_COUNT / spacing ) );
    loci.numr = u16( round( (fractional_N - loci.whol) * loci.dnom) );
    return loci; }
  public:
  Resolver( const DBL& actual_pfd = PFD, const u16& step = IOTA )
    : pfd{ actual_pfd }, spacing{ step } {}
  auto operator()(const u64& bn, Axis axis = FREQ) -> const decltype(loci) { switch(axis) {
    case AMPL:  return amplitude(static_cast<dBm>(bn));
    default:
    case FREQ:  return omega(bn);
    case PHAS:  return phi(DBL(bn)); } }
      // Resolver value dispatcher. Returns Axis selective value from State
  const auto operator()(Axis axis = FREQ) -> const u64 {
    switch (axis) {
      case AMPL:  return static_cast<u64>(amplitude());
      default:
      case FREQ:  return omega();
      case PHAS:  return static_cast<u64>(phi()); } } };
  template <size_t Digits>
class Indexer {
  private:
    size_t index;
  public:
  Indexer(const size_t& ix = 0)                      // Constrain() is a macro. So, beware.
    : index{ constrain(ix, 0, Digits-1) } {}
  virtual ~Indexer() {}
  auto operator()() -> decltype(index) { return index; }
  auto operator()(u8 ix) -> decltype(index) { return index = constrain(ix, 0, Digits-1); }
  auto operator++() -> decltype(*this) { if(Digits-1 < ++index) index = 0; return *this; }
  auto operator++(int) -> decltype(*this) { return operator++(); }
  auto operator--() -> decltype(*this) { if(0 == index--) index = Digits-1; return *this; }
  auto operator--(int) -> decltype(*this) { return operator--(); } };
  template <size_t Digits, size_t Radix = 10>
class Numeral {
  private:  //  https://en.m.wikipedia.org/w/index.php?title=Positional_notation
    std::deque<u8,Digits> numrl;
  public:
  Indexer<Digits> cursor;
  Numeral(u64 bn = bottom) { operator()(bn); }
  virtual ~Numeral() {}
  auto operator[](const size_t& position) -> const u8 {
    return numrl[ constrain(position, 0, Digits-1) ]; } 
  auto operator()() -> const u64 {
    u64 sum{0};
    for(u8 idx{0}; idx!=numrl.size(); idx++) sum += operator[](idx) * power(Radix, idx);
    return sum; }
  auto operator()(const Direction& d) -> void { switch(d) {
    default:  break;
    case up:  { u64 sum{ operator()() }; sum += power(Radix, cursor());
                operator()( constrain(sum, MIN_FREQ, MAX_FREQ)); } break;//
    case dn:  { u64 sum{ operator()() }; sum -= power(Radix, cursor());
                operator()( constrain(sum, MIN_FREQ, MAX_FREQ)); } break;//
    case left: ++cursor; break;
    case rght: --cursor; break; } }
  auto operator()(u64 bn) -> void {
    numrl.clear();
    for(u8 index{0}; index!=Digits; index++) {
      numrl.push_front(bn / power(Radix, Digits-1-index));
        bn %= power(Radix, Digits-1-index); } }
  auto operator+(const u64& bn) -> decltype(*this) { operator()(operator()() + bn); return *this; }
  auto operator-(const u64& bn) -> decltype(*this) { operator()(operator()() - bn); return *this; }
    #ifdef DEBUG
  auto pr() -> void {
    for(size_t ix{}; size() != ix; ix++) ::pr(operator[](size()-1-ix)); ::pr(' '); }
    #endif
  auto size() -> const size_t { return numrl.size(); }
  auto disp( OLED& oled, int x, int y) -> void {
    oled.setCursor(x,y);
    for(size_t ix{}; size() != ix; ix++) oled.print( operator[](size() - 1 - ix) ); } };
struct Panel {
  Numeral<10> f{bottom}, df{25*kHz};
  Numeral<4>  pnumr{1}, dp{1};
  Numeral<1,4> a{0};
    #ifdef DEBUG
  auto pr(Axis axis = FREQ) -> void { switch(axis) {
    case AMPL:  a.pr();     break;
    default:
    case FREQ:  f.pr();     break;
    case DF:    df.pr();    break;
    case PHAS:  pnumr.pr(); break;
    case DP:    dp.pr();    break; } }
      #endif
  auto operator()(Axis axis = FREQ) -> const u64 { switch(axis) {
    case AMPL:  return u64(a());
    default:
    case FREQ:  return f();
    case DF:    return df();
    case PHAS:  return pnumr();
    case DP:    return dp();  } }
  auto operator()(const u64& bn, Axis axis = FREQ) -> void { switch(axis) {
    case AMPL:  a(dBm(bn));  break;
    default:
    case FREQ:  f(bn);       break;
    case DF:    df(bn);      break;
    case PHAS:  pnumr(bn);   break;
    case DP:    dp(bn);      break; } } 
};/* End Synthesis:: */ }
BasicEncoder encoder(2, 3, HIGH);
void pciSetup(byte pin) {
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR |= bit(digitalPinToPCICRbit(pin));                    // clear outstanding interrupt
  PCICR |= bit(digitalPinToPCICRbit(pin)); }
void setup_encoders(int a, int b) {
  uint8_t old_sreg = SREG;
  cli();
  pciSetup(a);
  pciSetup(b);
  //encoder.reset();
  SREG = old_sreg; }
ISR(PCINT2_vect) {
  encoder.service(); }
    /* "How shall I tell you the story?" The King replied, "Start at the beginning. Proceed
    until the end. Then stop." Lewis Carroll. "Alice's Adventures in Wonderland". 1865. */
auto setup() -> void { using namespace Hardware;    // "And, away we go." Gleason.
  pinMode(static_cast<u8>(PIN::PDR), OUTPUT);       // Rf output enable.
  rf( OFF );
  pinMode(static_cast<u8>(PIN::LE_A), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::LE_A), 1);
  pinMode(static_cast<u8>(PIN::LD_A), INPUT);
  // pinMode(static_cast<u8>(PIN::LE_B), OUTPUT);
  // digitalWrite(static_cast<u8>(PIN::LE_B), 1);
  // pinMode(static_cast<u8>(PIN::LD_B), INPUT);
  digitalWrite(static_cast<u8>(PIN::MUX), INPUT_PULLUP);
  SPI.begin();
  setup_encoders(2,3);
  encoder.set_reverse(); }
    // Jettson[George]: "Jane! JANE! Stop this crazy thing! JANE! !!!".
auto loop() -> void { using namespace Synthesis;
; SpecifiedOverlay pll;
    #ifdef DEBUG
  Serial.begin(1000000L); delay(1000L);
    #endif // Quantiy S::_end calls of set() are required, in any order.
  //                     S::fraction, S::integer, S::modulus S::rfDivSelect      (1) (2) (3) (36)
  pll( S::phase, 1);                     // Adjust phase AFTER loop lock. Not redundant.      (4)
  pll( S::phAdj, OFF );                                                                    // (5)
  pll( S::prescaler,PRSCL::eight9ths );  // Possiblly redundant                            // (6)
  pll( S::counterReset, OFF );                                                             // (7)
  pll( S::cp3state, OFF );                                                                 // (8)
  pll( S::idle, OFF );                                                                     // (9)
  pll( S::pdPolarity, PDpolarity::positive );                                             // (10)
  pll( S::ldp, LDPnS::ten );                                                              // (11)
  pll( S::ldf, LockDetectFunction::fracN );                                               // (12)
  pll( S::cpIndex, 7 );  // 0 thru 15, 2.5mA = '7', more increases loop bandwidth.           (13)
  pll( S::dblBfr, ON );                                                                   // (14)
  pll( S::rCounter, R_COUNT );                                                            // (15)
  pll( S::refToggler, TGLR );                                                             // (16)
  pll( S::refDoubler, DBLR );                                                             // (17)
  pll( S::muxOut, MuxOut::HiZ );          // see 'cheat sheet'                               (18)
  pll( S::LnLsModes, lowNoise );                                                          // (19)
  constexpr auto CLKDIV32{ 150 };          // I don't understand this, YET.
  //= round( PFD / MOD * 400e-6 ); // from datasheets'
  // 'Phase Resync' text: tSYNC = CLK_DIV_VALUE × MOD × tPFD
  constexpr auto CLKDIV{ u16(CLKDIV32) };
  static_assert( (0 < CLKDIV) && (4096 > CLKDIV) ); // Non-zero, 12 bit value.
  pll( S::clkDivider, CLKDIV );                                                           // (20)
  pll( S::clkDivMode, ClockingMode::dividerOff );                                         // (21)
  pll( S::csr, ON );                      // Cycle Slip reduction                            (22)
  pll( S::chrgCancel, OFF );                                                              // (23)
  pll( S::abp, ABPnS::nS6fracN );                                                         // (24)
  pll( S::bscMode,(MIN_PFD < PFD) ? BSCmd::programmed : BSCmd::automatic );               // (25)
  pll( S::rfOutPwr, minus4 );             // Possiblly redundant                          // (26)
  pll( S::rfSoftEnable, ON );                                                             // (27)
  pll( S::auxOutPwr, minus4 );                                                            // (28)
  pll( S::auxOutEnable, OFF );            // Pin not connected.                              (29)
  pll( S::muteTillLD, ON );                                                               // (30)
  pll( S::vcoPwrDown, OFF );                                                              // (31)
  pll( S::bndSelClkDv, u8(ceil(DBL(PFD) / MIN_PFD)) );                                    // (32)
  pll( S::rfFBselect, !Feedback );  // EEK! Why the negation? Perhaps I've been daVinci'd.   (33)
  pll( S::auxFBselect, !Feedback ); // See EEK!, above.                                      (34)
  pll( S::ledMode, LEDmode::lockDetect );                               // Ding. Winner!     (35)
; Panel panel;
  Resolver resolver;
  OLED oled;  oled.begin();  oled.setTextSize(2);
  panel(plus2,AMPL);
  pll(resolver(panel(AMPL),AMPL));
  pll.phAdj(OFF).set(resolver(0/360.,PHAS));
  panel(pll().numr,PHAS);
  // State trajectory versus frequency along a line: loci(f) = resolver(slope * f + bottom)
  auto top{ 100*MHz };
  // panel( 4*GHz + 400*MHz, FREQ ); // 4400 MHz requires at least thirty THREE bits.
  panel( 12500,DF );
  HW::rf(ON);
 for(bool once{ 0 }; ON; ) {
    pll( resolver(panel()) ).flush().lock();
      oled.clearDisplay();   panel.df.disp(oled, 0, 0);
      oled.setTextSize(2);   panel.f.disp(oled, 0, 12);
      oled.setTextSize(1);   oled.setCursor(0, 30);
      oled.print("rpwr:");   oled.print(pll().rpwr); oled.print(" rdiv:"); oled.print(pll().rdiv);
      oled.print("\ndnom:"); oled.print(pll().dnom); oled.print(" prop:"); oled.print(pll().prop);
      oled.print("\nwhol:"); oled.print(pll().whol); oled.print(" numr:"); oled.print(pll().numr);
    oled.display();
    #ifdef DEBUG
      if (encoder.get_change()) Serial.print(encoder.get_count()); pr(' ');
      panel.pr(); panel.pr(DF);
      panel.pr(DP); panel.pr(PHAS);
      panel.pr(AMPL); /*pr(' ');
      pr("rpwr:",pll().rpwr); pr("rdiv:",pll().rdiv); pr("prop:",pll().prop);
      pr("dnom:",pll().dnom); pr("whol:",pll().whol); pr("numr:",pll().numr); */
      pr('\n');
    #endif
    panel( random(0,5250) * panel(DF) + bottom ); // y = mx + b
    do delay(1666); while(once); } } // kd9fww