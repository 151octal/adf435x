/*  ©2024 kd9fww. ADF435x stand alone using Arduino Nano hardware SPI (in ~450 lines, ~25k mem).
    https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Source code.
    https://www.analog.com/ADF4351 <- Datasheet of the device for which it is designed.
    https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet */
  #include "Adafruit_EEPROM_I2C.h"
  #include <Adafruit_GFX.h>
  #include "Adafruit_seesaw.h"
  #include <Arduino.h>
  #include <ArxContainer.h>
  #include "SSD1306Ascii.h"
  #include "SSD1306AsciiWire.h"
  #include <SPI.h>
  #include <TimerOne.h>
  #include <Wire.h>
#define DEBUG
  #undef DEBUG
  auto setup() -> void {}
; using i64 = long long;
  using DBL = double;
  using EMEM = Adafruit_EEPROM_I2C;
  using OLED = SSD1306AsciiWire;
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
  enum PIN : u8 { MUX =  4, PDR =  6, LD_A = 7, LE_A = 10 };
  enum UNIT { A, /* B, */ _end };
  constexpr struct CTRL { PIN le, ld; } ctrl[] = { [A] = { LE_A, LD_A } };/*
    , [B] = { LE_B, LD_B } };*/
  static_assert(UNIT::_end == sizeof(ctrl) / sizeof(ctrl[0]));
  const auto hardWait = [](const PIN& pin) { while( !digitalRead( static_cast<u8>(pin) )); };
  const auto rf(bool enable) -> void { digitalWrite( static_cast<u8>(PIN::PDR), enable ); };
  const auto rf() -> const bool { return digitalRead( static_cast<u8>(PIN::PDR) ); }
  const auto tx = [](const PIN& le, void *pByte, int nByte) {
    auto p = static_cast<u8*>(pByte) + nByte;       // Most significant BYTE first.
    digitalWrite( static_cast<u8>(le), 0 );         // Predicate condition for data transfer.
    while( nByte-- ) SPI.transfer( *(--p) );        // Return value is ignored.
    digitalWrite( static_cast<u8>(le), 1 ); };      /* Data is latched on the rising edge. */
  volatile bool blank{ 0 };                           // oled timeout result
  void Hide(void) { blank = 1; }
} namespace HW = Hardware; namespace Synthesis {
 enum   ABPnS { nS6fracN = 0, nS3intN };            // AntiBacklash Pulse
  enum class Axis : size_t { AMPL = 1, FREQ, PHAS };
  Axis& operator++(Axis& axis) { switch(axis) {
    case Axis::AMPL: return axis = Axis::FREQ;
    case Axis::FREQ: return axis = Axis::PHAS;
    case Axis::PHAS: return axis = Axis::AMPL; } }
  Axis& operator--(Axis& axis) { switch(axis) {
    case Axis::AMPL: return axis = Axis::PHAS;
    case Axis::FREQ: return axis = Axis::AMPL;
    case Axis::PHAS: return axis = Axis::FREQ; } }
  enum  BSCmd { programmed = 0, automatic };        // Band Select Clock mode
  enum  ClockingMode { dividerOff = 0, fastLock, phResync };
  enum  dBm : u8 { minus4 = 0, minus1, plus2, plus5 };
  enum  Dir : u8 { sel = 2, up  = sel << 1, sav = sel+up,
                            left = up << 1, prev = sel+left,
                            dn = left << 1, rcl = sel+dn,
                            rght = dn << 1, next = sel+rght };
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
  auto  power(u8 radix, u8 exponent) -> const i64 { // radix raised to exponent
          i64 rv{1}; for(auto ix{exponent}; ix; --ix) rv *= radix; return rv; }
 constexpr  i64   kHz{ 1000 }, MHz{ 1000*kHz }, GHz{ 1000*MHz }, bottom{ 34*MHz + 375*kHz };
  constexpr i64   MAX_VCO{ 4400000000 };            // 4400 MHz.
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
 constexpr  auto  CORRECTION{ -150 };               // Determined by working in reverse, from
  constexpr auto  REF_ERROR{ (COMP) * CORRECTION }; // the value of REF, as measured, next line.
  constexpr auto  REF{ OSC + REF_ERROR };           // Measured reference oscillator frequency.
  constexpr auto  TGLR{ OFF }, DBLR{ TGLR };        // OFF: ONLY if OSC is a 50% duty square wave.
 constexpr  auto  PFD{ DBL(REF) * (1+DBLR) / (1+TGLR) / R_COUNT };
  static_assert(R_COUNT * IOTA == OSC / MOD);       // No remainder.
  static_assert((0<R_COUNT) && (1024>R_COUNT));     // Non-zero, 10 bits.
  static_assert((MAX_PFD >= PFD));// && (PFD * R_COUNT == REF));
enum Identifier : u8 {  // Human readable register 'field' identifiers.
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
  };  using I = Identifier;
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
  [I::fraction] = {5, 3, 12},     [I::integer] = {5, 15, 16},     [I::modulus] = {4, 3, 12},
  [I::phase] = {4, 15, 12},       [I::prescaler] = {4, 27, 1},    [I::phAdj] = {4, 28, 1},
  [I::counterReset] = {3, 3, 1},  [I::cp3state] = {3, 4, 1},      [I::idle] = {3, 5, 1},
  [I::pdPolarity] = {3, 6, 1},    [I::ldp] = {3, 7, 1},           [I::ldf] = {3, 8, 1},
  [I::cpIndex] = {3, 9, 4},       [I::dblBfr] = {3, 13, 1},       [I::rCounter] = {3, 14, 10},
  [I::refToggler] = {3, 24, 1},   [I::refDoubler] = {3, 25, 1},   [I::muxOut] = {3, 26, 3},
  [I::LnLsModes] = {3, 29, 2},    [I::clkDivider] = {2, 3, 12},   [I::clkDivMode] = {2, 15, 2},
  [I::csr] = {2, 18, 1},          [I::chrgCancel] = {2, 21, 1},   [I::abp] = {2, 22, 1},
  [I::bscMode] = {2, 23, 1},      [I::rfOutPwr] = {1, 3, 2},      [I::rfSoftEnable] = {1, 5, 1},
  [I::auxOutPwr] = {1, 6, 2},     [I::auxOutEnable] = {1, 8, 1},  [I::auxFBselect] = {1, 9, 1},
  [I::muteTillLD] = {1, 10, 1},   [I::vcoPwrDown] = {1, 11, 1},   [I::bndSelClkDv] = {1, 12, 8},
  [I::rfDivSelect] = {1, 20, 3},  [I::rfFBselect] = {1, 23, 1},   [I::ledMode] = {0, 22, 2} };
  static_assert(Identifier::_end == (sizeof(ADF435x) / sizeof(ADF435x[0])));
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
        regions become invariants by not providing identifiers for them in the Specification. */
    u8 durty; SPISettings settings; RegArray reg; };
    using OVL = Overlay;
    OVL ovl{ 0, SPISettings(4000000, MSBFIRST, SPI_MODE0), OVL::RegArray{ 0x180005,4,3,2,1,0} };
    auto raw( const I& symbol,const u16& value ) -> decltype(*this) {
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
  auto locked() -> int { return digitalRead( static_cast<u8>(ld) ); }
  auto operator()( const HW::PIN _le, HW::PIN _ld ) -> decltype(*this) {
    le = _le; ld =_ld; return *this; }
  auto operator()( const HW::CTRL& io ) -> decltype(operator()(io.le, io.ld)) {
    return operator()(io.le, io.ld); }
    // Parameter Storage Intercept
  auto operator()( const I& sym,const u16& val ) -> decltype(*this) { switch(sym) {
    default:                return raw( sym,val );  // Beware of { case: fall thru }
    case I::fraction:     if(val !=  store.numr) {
                            return raw( sym,store.numr  = val ); }
                      else  return raw( sym,val );
    case I::integer:      if(val !=  store.whol) {
      raw( I::prescaler,(75 < val) ? PRSCL::eight9ths : PRSCL::four5ths );
                            return raw( sym,store.whol = val ); }
                      else  return raw( sym,val );
    case I::phase:        if(val !=  store.prop) {
                            return raw( sym,store.prop  = val ); }
                      else  return raw( sym,val );
    case I::modulus:      if(val !=  store.dnom) {
      raw( I::LnLsModes,nsMode = (lowSpur == nsMode) ? ((50 > val) ? lowNoise : nsMode) : nsMode);
                            return raw( sym,store.dnom = val ); }
                      else  return raw( sym,val );
    case I::rfDivSelect:  if(val !=  store.rdiv) {
                            return raw( sym,store.rdiv  = val ); }
                      else  return raw( sym,val );
    case I::rfOutPwr:     if(static_cast<dBm>(val) != store.rpwr) { 
                            raw( I::rfSoftEnable, ON );
                            return raw( sym,store.rpwr = static_cast<dBm>(val) ); }
                      else  return raw( sym,val );                                } }
      // Parameter dispatcher
  auto operator()( const State& loci ) -> decltype(*this) {
    set( I::fraction,loci.numr ).set( I::integer,loci.whol ).set( I::modulus,loci.dnom );
    set( I::phase,loci.prop ).set( I::rfDivSelect,loci.rdiv ).set( I::rfOutPwr,loci.rpwr );
    return *this;  }
  auto operator()() -> const decltype(store) { return store; }
  auto phAdj( const bool& e ) -> decltype(*this) { raw( I::phAdj,e ); return *this; }
    // Wrapper for operator()( sym,val )
  auto set( const I& sym,const u16& val ) -> decltype(*this) { return operator()( sym,val ); }
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
  auto omega() -> const i64 {
    return i64(pfd) * (loci.whol + DBL(loci.numr) / loci.dnom) / pow(2,loci.rdiv); }
  auto omega(const i64& bn) -> const decltype(loci) {
    auto freq{ constrain(bn, MIN_FREQ, MAX_FREQ) };
    loci.rdiv = u16( floor( log2(MAX_VCO/freq) ) );
    auto fractional_N{ (freq / pfd) * pow(2, loci.rdiv) };
    loci.whol = u16( trunc( fractional_N ) );
    //loci.whol = (22 < loci.whol) ? loci.whol : 22;
    loci.dnom = u16( ceil( OSC / R_COUNT / spacing ) );
    loci.numr = u16( round( (fractional_N - loci.whol) * loci.dnom) );
    return loci; }
  auto pnum() -> const i64 { return loci.prop; }
  auto pnum(const i64& bn) -> const decltype(loci) {
    loci.prop = constrain(bn, 1, loci.dnom-1 ); return loci; }
  public:
  Resolver( const DBL& actual_pfd = PFD, const u16& step = IOTA )
    : pfd{ actual_pfd }, spacing{ step } {}
  auto operator()(const i64& bn, Axis axis = Axis::FREQ) -> const decltype(loci) { switch(axis) {
    case Axis::AMPL:  return amplitude(static_cast<dBm>(bn));
    default:
    case Axis::FREQ:  return omega(bn);
    case Axis::PHAS:  return pnum(bn); } }//phi(DBL(bn)); } }
      // Resolver value dispatcher. Returns Axis selective value from State
  const auto operator()(Axis axis = Axis::FREQ) -> const i64 {
    switch (axis) {
      case Axis::AMPL:  return static_cast<i64>(amplitude());
      default:
      case Axis::FREQ:  return omega();
      case Axis::PHAS:  return pnum(); } } };
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
  Numeral(i64 bn) { operator()(bn); }
  virtual ~Numeral() {}
  auto operator[](const size_t& position) -> const u8 {
    return numrl[ constrain(position, 0, Digits-1) ]; } 
  auto operator()() -> const i64 {
    i64 sum{0};
    for(u8 idx{0}; idx!=numrl.size(); idx++) sum += operator[](idx) * power(Radix, idx);
    return sum; }
  auto operator()(const Dir& d) -> void { switch(d) {
    default:  break;
    case up:  { i64 sum{ operator()() }; sum += power(Radix, cursor());
                operator()( sum ); } break;
    case dn:  { i64 sum{ operator()() }; sum -= power(Radix, cursor());
                operator()( sum ); } break;
    case left: ++cursor; break;
    case rght: --cursor; break; } }
  auto operator()(i64 bn) -> void {
    numrl.clear();
    for(u8 index{0}; index!=Digits; index++) {
      numrl.push_front(bn / power(Radix, Digits-1-index));
        bn %= power(Radix, Digits-1-index); } }
  auto operator+(const i64& bn) -> decltype(*this) { operator()(operator()() + bn); return *this; }
  auto operator-(const i64& bn) -> decltype(*this) { operator()(operator()() - bn); return *this; }
    #ifdef DEBUG
  auto pr() -> void {
    for(size_t ix{}; size() != ix; ix++) ::pr(operator[](size()-1-ix)); ::pr(' '); }
    #endif
  auto size() -> const size_t { return numrl.size(); }
  auto disp( OLED& oled ) -> void {
    for(size_t ix{}; size() != ix; ix++) oled.print(operator[](size()-1-ix));    oled.println();
    for(size_t ix{}; size()-1-cursor() != ix; ix++) oled.print(' ');    oled.println('^'); } };
/* End Synthesis:: */ }
auto loop() -> void {                               // "And, away we go ..." Gleason.
  using namespace Hardware;
  pinMode(static_cast<u8>(PIN::PDR), OUTPUT);       // Rf output enable.
  rf( OFF );
  pinMode(static_cast<u8>(PIN::LE_A), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::LE_A), 1);
  pinMode(static_cast<u8>(PIN::LD_A), INPUT);
  // pinMode(static_cast<u8>(PIN::LE_B), OUTPUT);
  // digitalWrite(static_cast<u8>(PIN::LE_B), 1);
  // pinMode(static_cast<u8>(PIN::LD_B), INPUT);
  pinMode(static_cast<u8>(PIN::MUX), INPUT_PULLUP);
  Wire.begin();  Wire.setClock(400000L);
  SPI.begin();
  Timer1.initialize(7654321UL);  Timer1.attachInterrupt(HW::Hide);
  using namespace Synthesis;
; SpecifiedOverlay pll;                             // ... with it all on the stack. Me.
    #ifdef DEBUG
  Serial.begin(1000000L);// delay(1000L);
    #endif // Quantiy I::_end calls of set() are required, in any order.
  //                     I::fraction, I::integer, I::modulus I::rfDivSelect      (1) (2) (3) (36)
  pll( I::phase, 1);                     // Adjust phase AFTER loop lock. Not redundant.      (4)
  pll( I::phAdj, OFF );                                                                    // (5)
  pll( I::prescaler,PRSCL::eight9ths );  // Possiblly redundant                            // (6)
  pll( I::counterReset, OFF );                                                             // (7)
  pll( I::cp3state, OFF );                                                                 // (8)
  pll( I::idle, OFF );                                                                     // (9)
  pll( I::pdPolarity, PDpolarity::positive );                                             // (10)
  pll( I::ldp, LDPnS::ten );                                                              // (11)
  pll( I::ldf, LockDetectFunction::fracN );                                               // (12)
  pll( I::cpIndex, 7 );  // 0 thru 15, 2.5mA = '7', more increases loop bandwidth.           (13)
  pll( I::dblBfr, ON );                                                                   // (14)
  pll( I::rCounter, R_COUNT );                                                            // (15)
  pll( I::refToggler, TGLR );                                                             // (16)
  pll( I::refDoubler, DBLR );                                                             // (17)
  pll( I::muxOut, MuxOut::HiZ );          // see 'cheat sheet'                               (18)
  pll( I::LnLsModes, lowNoise );                                                          // (19)
  constexpr auto CLKDIV32{ 150 };          // I don't understand this, YET.
  //= round( PFD / MOD * 400e-6 ); // from datasheets'
  // 'Phase Resync' text: tSYNC = CLK_DIV_VALUE × MOD × tPFD
  constexpr auto CLKDIV{ u16(CLKDIV32) };
  static_assert( (0 < CLKDIV) && (4096 > CLKDIV) ); // Non-zero, 12 bit value.
  pll( I::clkDivider, CLKDIV );                                                           // (20)
  pll( I::clkDivMode, ClockingMode::dividerOff );                                         // (21)
  pll( I::csr, ON );                      // Cycle Slip reduction                            (22)
  pll( I::chrgCancel, OFF );                                                              // (23)
  pll( I::abp, ABPnS::nS6fracN );                                                         // (24)
  pll( I::bscMode,(MIN_PFD < PFD) ? BSCmd::programmed : BSCmd::automatic );               // (25)
  pll( I::rfOutPwr, minus4 );             // Possiblly redundant                          // (26)
  pll( I::rfSoftEnable, ON );                                                             // (27)
  pll( I::auxOutPwr, minus4 );                                                            // (28)
  pll( I::auxOutEnable, OFF );            // Pin not connected.                              (29)
  pll( I::muteTillLD, ON );                                                               // (30)
  pll( I::vcoPwrDown, OFF );                                                              // (31)
  pll( I::bndSelClkDv, u8(ceil(DBL(PFD) / MIN_PFD)) );                                    // (32)
  pll( I::rfFBselect, !Feedback );  // EEK! Why the negation? Perhaps I've been daVinci'd.   (33)
  pll( I::auxFBselect, !Feedback ); // See EEK!, above.                                      (34)
  pll( I::ledMode, LEDmode::lockDetect );                               // Ding. Winner!     (35)
; Resolver resolver;
  pll.phAdj(OFF);
  Numeral<1> pwr{plus5};
  Numeral<10> frequency{bottom};  for(auto x{5}; x; --x) frequency(left);
  Numeral<4> angle{1};
  Axis axis{ Axis::FREQ };
  OLED oled;  oled.begin(&Adafruit128x64, 0x3d);  oled.setContrast(0);
  EMEM ee; while(!ee.begin());
  i64 bn{}; bool transmit;
  ee.readObject(0, bn); pwr(bn);
  ee.readObject(sizeof(i64), bn); frequency(bn);
  ee.readObject(2*sizeof(i64), bn); angle(bn);
  ee.readObject(2*sizeof(i64)+sizeof(bool), transmit);
  if(transmit) HW::rf(ON);
  Adafruit_seesaw ass;  while(!ass.begin());  ass.pinModeBulk(0x3F, INPUT_PULLUP);
    //auto knob{ ass.getEncoderPosition() };
; for( bool update{1}; ON; ) {
    if( update ) {
      pwr(constrain(pwr(),minus4,plus5)); pll(resolver(pwr(), Axis::AMPL));
      frequency(constrain(frequency(),MIN_FREQ,MAX_FREQ)); pll(resolver(frequency(), Axis::FREQ));
      angle(constrain(angle(),1,pll().dnom-1)); pll(resolver(angle(), Axis::PHAS)).flush().lock();
      Timer1.start();
      oled.clear();
      oled.setFont(font5x7); oled.setLetterSpacing(1);
      if(transmit) oled.print("* "); else oled.print("  ");
      switch(axis) {
        case Axis::AMPL:  oled.println("Power"); break;
        case Axis::FREQ:  oled.println("Frequency"); break;
        case Axis::PHAS:  oled.println("Phase"); break; }
      oled.setFont(X11fixed7x14); oled.setLetterSpacing(3);
      switch(axis) {
        case Axis::AMPL:  pwr.disp(oled); break;
        case Axis::FREQ:  frequency.disp(oled); break;
        case Axis::PHAS:  angle.disp(oled); break; }
      #ifdef DEBUG
        oled.setFont(font5x7);  oled.setLetterSpacing(1);
        oled.print("rpwr: ");   oled.print(pll().rpwr); oled.print(" rdiv: "); oled.print(pll().rdiv);
        oled.print("\ndnom: "); oled.print(pll().dnom); oled.print(" prop: "); oled.print(pll().prop);
        oled.print("\nwhol: "); oled.print(pll().whol); oled.print(" numr: "); oled.print(pll().numr);
        //frequency.pr();
        pr("rpwr:",pll().rpwr); pr("rdiv:",pll().rdiv); pr("prop:",pll().prop);
        pr("dnom:",pll().dnom); pr("whol:",pll().whol); pr("numr:",pll().numr);
        pr('\n');
      #endif
      update = HW::blank = 0; }
    if(HW::blank) { Timer1.stop(); oled.clear(); HW::blank = 0; }
    auto buttons{ 0x3F ^ ass.digitalReadBulk(0x3F) };
    switch(buttons) { default: break;
      case sel:   update = 1; break;
      case up:    switch(axis) {
                    case Axis::AMPL:  pwr(up); break;
                    case Axis::FREQ:  frequency(up); break;
                    case Axis::PHAS:  angle(up); break; } update = 1; break;
      case sav:   bn = pwr();  ee.writeObject(0, bn);
                  bn = frequency(); ee.writeObject(sizeof(i64), bn);
                  bn = angle(); ee.writeObject(2*sizeof(i64), bn);
                  transmit = HW::rf(); HW::rf(OFF);
                  ee.writeObject(2*sizeof(i64)+sizeof(bool), transmit);
                  update = 1; break;
      case left:  switch(axis) {
                    case Axis::AMPL:  pwr(left); break;
                    case Axis::FREQ:  frequency(left); break;
                    case Axis::PHAS:  angle(left); break; } update = 1; break;
      case prev:  --axis; update = 1; break;
      case dn:    switch(axis) {
                    case Axis::AMPL:  if(pwr()) pwr(dn); break;
                    case Axis::FREQ:  frequency(dn); break;
                    case Axis::PHAS:  if(1 < angle()) angle(dn); break; } update = 1; break;
      case rcl:   ee.readObject(0, bn); pwr(bn);
                  ee.readObject(sizeof(i64), bn); frequency(bn);
                  ee.readObject(2*sizeof(i64), bn); angle(bn);
                  HW::rf(ON); transmit = HW::rf();
                  break;
      case rght:  switch(axis) {
                    case Axis::AMPL:  pwr(rght); break;
                    case Axis::FREQ:  frequency(rght); break;
                    case Axis::PHAS:  angle(rght); break; } update = 1; break;
      case next:  ++axis; update = 1; break; }
    delay(33);  } } // kd9fww