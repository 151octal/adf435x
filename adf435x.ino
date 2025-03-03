/*  ©rwHelgeson[kd9fww] 2024, 2025.  ADF435x using ATMEGA328 hardware SPI.
    https://github.com/151octal/adf435x/blob/main/LICENSE
    https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Source code.
    https://www.analog.com/ADF4351 <- Datasheet of the device for which it is designed.
    https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet */
  #include "Adafruit_EEPROM_I2C.h"
  #include <Adafruit_GFX.h>
  #include "Adafruit_seesaw.h"
  #include <Arduino.h>
  #include <ArxContainer.h>
  #include <avr/sleep.h>
  #include "SSD1306Ascii.h"
  #include "SSD1306AsciiWire.h"
  #include <SPI.h>
  #include <TimerOne.h>
  #include <Wire.h>
  #define DEBUG
  //  #undef DEBUG
  using i64 = long long;
  using AFSS = Adafruit_seesaw;
  using DBL = double;
  using OLED = SSD1306AsciiWire;  // Because the Adafruit library is too big. (both)
  using XMEM = Adafruit_EEPROM_I2C;
  enum Enable { OFF = 0, ON = 1 };
  auto pr( const char& cc ) -> size_t { return Serial.print(cc); }  // Shorthand.
  auto pr( const u8& uc ) -> size_t { return Serial.print(uc); }
      #ifdef DEBUG
  auto pr( const char* const s ) -> size_t { return Serial.print(s); }
  auto pr( const u16& arg, int num = DEC ) -> size_t {
    auto n{ Serial.print(arg, num) }; n += pr(' '); return n; }
  auto pr( const char* const s, const u16& arg, int num = DEC ) -> size_t {
    auto n{ pr(s) }; n += pr(arg, num); return n; }
      #endif
namespace Hardware {
  // See https://github.com/151octal/adf435x/blob/main/README.md for circuitry notes.
  enum PIN : u8 { USR = 2, MUX = 4, PDR = 6, LD_A = 7, LE_A = 10 };
  enum UNIT { A, /* B, */ _end };
  const auto ckMem = [](void* vp, size_t n) {
    auto p{ static_cast<u8*>(vp) };
    u16 sum{0}; while(n--) { sum += *(p++); sum %= 255; } return sum; };
  constexpr struct CTRL { PIN le, ld; } ctrl[] = { [A] = { LE_A, LD_A } };/*
    , [B] = { LE_B, LD_B } };*/
  static_assert(UNIT::_end == sizeof(ctrl) / sizeof(ctrl[0]));
  auto hardWait{ [](const PIN& pin){ while( !digitalRead( static_cast<u8>(pin) )); } };
  auto Nudge{ [](){ sleep_disable(); } };
  auto rf(bool enable) -> void { digitalWrite( static_cast<u8>(PIN::PDR), enable ); };
  auto rf() -> const bool { return digitalRead( static_cast<u8>(PIN::PDR) ); }
  auto snooz() -> void {  // lambda hostile macros. Another reason to promulgate them not.
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    attachInterrupt(digitalPinToInterrupt(USR), Nudge, LOW);
    sleep_mode();
    detachInterrupt(digitalPinToInterrupt(USR));  }
  volatile bool timOut{ 0 };
  auto Trigger{ [](){ timOut = 1; } };  // Timer1 ISR target
  auto tx = [](const PIN& le, void *pByte, int nByte){
    auto p{ static_cast<u8*>(pByte) + nByte };      // Most significant BYTE first.
    digitalWrite( static_cast<u8>(le), 0 );         // Predicate condition for data transfer.
    while( nByte-- ) SPI.transfer( *(--p) );        // Return value is ignored.
    digitalWrite( static_cast<u8>(le), 1 ); };      // Data is latched on the rising edge.
} namespace HW = Hardware; namespace Synthesis {
 constexpr  i64   kHz{ 1000 }, MHz{ 1000*kHz }, GHz{ 1000*MHz };
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
  constexpr   u16  IOTA{ 1000 };                    // Choose IOTA such that R_COUNT is exact.
  constexpr auto  R_COUNT{ u16(OSC / IOTA / MOD) }; // REF / Rcounter = PFD = Modulus * IOTA
  constexpr auto  TGLR{ ON }, DBLR{ TGLR };        // OFF: ONLY if OSC is a 50% duty square wave.
 constexpr  auto  PFD{ DBL(OSC) * (1+DBLR) / (1+TGLR) / R_COUNT };
  static_assert(R_COUNT * IOTA == OSC / MOD);       // No remainder.
  static_assert((0<R_COUNT) && (1024>R_COUNT));     // Non-zero, 10 bits.
  static_assert((MAX_PFD >= PFD) && (MIN_PFD <= PFD));
 enum   ABPnS { nS6fracN = 0, nS3intN };            // AntiBacklash Pulse
  enum class Axis : size_t { HOLD, FREQ, AMPL, PHAS, REF };
  Axis& operator++(Axis& axs) { switch(axs) {
    case Axis::HOLD: axs = Axis::FREQ;  break;
    case Axis::FREQ: axs = Axis::PHAS;  break;
    case Axis::PHAS: axs = Axis::AMPL;  break;
    case Axis::AMPL: axs = Axis::REF;   break;
    case Axis::REF:  axs = Axis::HOLD;  break; } return axs; }
  enum  Action : u8 { sft = 2, inc = sft<<1, lft = inc<<1, dec = lft<<1, rgt = dec<<1 };
  constexpr auto btnMask{ sft + inc + lft + dec + rgt };
  const auto btns{ [](AFSS& ss,const u32& msk = btnMask){ return msk^ss.digitalReadBulk(msk); } };
  enum  BSCmd { programmed = 0, automatic };        // Band Select Clock mode
  enum  ClockingMode { dividerOff = 0, fastLock, phResync };
  enum  dBm : u8 { minus4 = 0, minus1, plus2, plus5 };
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
  const auto power{ [](const u8 r, u8 e){ i64 rv{1}; while(e--) rv *= r; return rv; } };
enum Identifier : u8 {
    // Human readable register 'field' identifiers.
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
  //  https://gcc.gnu.org/onlinedocs/gcc/Designated-Inits.html
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
  constexpr State INIT{ .rpwr = minus4, 0, .dnom = 1, .whol = 0, .numr = 0, .prop = 1 };
  /* kd9fww */
class SpecifiedOverlay {
  private:
    HW::PIN le{ HW::ctrl[HW::UNIT::A].le }, ld{ HW::ctrl[HW::UNIT::A].ld };
    NoiseSpurMode nsMode = lowNoise;  // sloppy afterthought
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
      case  1:  cx = ovl.NR - 1; break;   /* r0 ••• */
      case  2:  /* fall thru */           /* r1 ••• */
      case  3:  cx = ovl.NR - 2; break;   /* r1 and r0 ••• */
      case 16:  cx = ovl.NR - 4; break;   /* r4 ••• */ }
    ovl.durty = 0;
    SPI.beginTransaction( ovl.settings );
    for(/* empty */; ovl.NR != cx; ++cx) HW::tx(le, &ovl.reg[cx], sizeof(ovl.reg[cx]) );
    SPI.endTransaction();
    return *this; }
  auto lock() -> void { HW::hardWait(ld); } // Wait on active ld pin, until lock is indicated.
  auto locked() -> const int { return digitalRead( static_cast<u8>(ld) ); }
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
  /* kd9fww */
class Resolver {
  private:  // Rotating phasor: f(t) = |magnitude| * pow( Euleran, j( omega*t + phi ))
    State loci{ INIT };
    DBL pvtPFD; u16 spacing;
    auto amplitude() -> const u8 { return loci.rpwr; }
    auto amplitude(const dBm& a) -> const decltype(loci) { loci.rpwr = a; return loci; }
      #ifdef MULTIPLE_PLL
        auto phi() -> const DBL { return (loci.prop / DBL(loci.dnom - 1)); }
        auto phi(DBL normalized) -> const decltype(loci) {
            normalized = constrain((0 > normalized) ? -normalized : normalized, 0, 1);
            auto proportion{ u16(round(normalized * (loci.dnom - 1))) };
            loci.prop = (1 > proportion) ? 1 : proportion;
            return loci; }
      #endif
    auto omega() -> const i64 {
      return i64(pvtPFD) * (loci.whol + DBL(loci.numr) / loci.dnom) / pow(2,loci.rdiv); }
    auto omega(const i64& bn) -> const decltype(loci) {
      auto freq{ constrain(bn, MIN_FREQ, MAX_FREQ) };
      loci.rdiv = u16( floor( log2(MAX_VCO/freq) ) );
      auto fractional_N{ (freq / pvtPFD) * pow(2, loci.rdiv) };
      loci.whol = u16( trunc( fractional_N ) );
      //loci.whol = (22 < loci.whol) ? loci.whol : 22;
      loci.dnom = u16( ceil( OSC / R_COUNT / spacing ) );
      loci.numr = u16( round( (fractional_N - loci.whol) * loci.dnom) );
      return loci; }
    auto pnum() -> const i64 { return loci.prop; }
    auto pnum(const i64& bn) -> const decltype(loci) {  // Phase NUMerator: phase = prop / denom
      loci.prop = constrain(bn, 1, loci.dnom-1 ); return loci; }
  public:
  Resolver( const DBL& pfd, const u16& step ) : pvtPFD{ pfd }, spacing{ step } {}
  auto operator()(const i64& bn, Axis axs = Axis::FREQ) -> const decltype(loci) { switch(axs) {
    default:
    case Axis::FREQ:  return omega(bn);
    case Axis::AMPL:  return amplitude(static_cast<dBm>(bn));
    case Axis::PHAS:  return pnum(bn); } }//phi(DBL(bn)); } }
      // Resolver value dispatcher. Returns Axis selective value from State
  auto operator()(Axis axs = Axis::FREQ) -> const i64 {
    switch (axs) {
      default:
      case Axis::FREQ:  return omega();
      case Axis::AMPL:  return static_cast<i64>(amplitude());
      case Axis::PHAS:  return pnum(); } }
  auto pfd() -> const DBL { return pvtPFD; }  // Phase Frequency Detector frequency
  auto pfd(const i64& ref) -> void { pvtPFD = DBL(ref) * (1+DBLR) / (1+TGLR) / R_COUNT; } };
    /* kd9fww */
  template <size_t Digits>
class Cursor {
  private:
    size_t pstn;
  public:
  Cursor(const size_t& ix = 0) : pstn{ constrain(ix, 0, Digits-1) } {}
  virtual ~Cursor() {}
  auto operator()() -> decltype(pstn) { return pstn; }
  auto operator()(u8 ix) -> decltype(pstn) { return pstn = constrain(ix, 0, Digits-1); }
  auto operator++() -> decltype(*this) { if(Digits-1 < ++pstn) pstn = 0; return *this; }
  auto operator--() -> decltype(*this) { if(0 == pstn--) pstn = Digits-1; return *this; } };
    template <size_t Digits, size_t Radix = 10>
class Nmrl {
  private:  //  https://en.m.wikipedia.org/w/index.php?title=Positional_notation
    std::deque<u8,Digits> numrl;
    Cursor<Digits> cursor;
  public:
  Nmrl(i64 bn = 0) { operator()(bn); }
  virtual ~Nmrl() {}
  auto disp( OLED& oled, char term = 0, const bool carat = true ) -> void {
    const auto current{ cursor() };
    auto keep{ oled.invertMode() };
    oled.setFont(X11fixed7x14); oled.setLetterSpacing(4);
    for(size_t x{}; size() != x; x++) {
      if(carat) { if(size()-1-current == x)   oled.setInvertMode(1);
      /**/        /**/                  else  oled.setInvertMode(0); }
      oled.print(operator[](size()-1-x));  }
    oled.setInvertMode(keep); oled.print(term);
    oled.setFont(Adafruit5x7);  oled.setLetterSpacing(1); }
 auto operator[](const size_t& pstn) -> const u8 { return numrl[constrain(pstn, 0, Digits-1)]; }
  auto operator()() -> const i64 {
    i64 sum{0};
    for(u8 idx{0}; idx!=numrl.size(); idx++) sum += operator[](idx) * power(Radix, idx);
    return sum; }
  auto operator()(const Action& action) -> void { switch(action) {
    default:    break;
    case inc:  { i64 sum{ operator()() }; sum += power(Radix, cursor());
                operator()( sum ); } break;
    case dec:  { i64 sum{ operator()() }; sum -= power(Radix, cursor());
                operator()( sum ); } break;
    case lft: ++cursor; break;
    case rgt: --cursor; break; } }
  auto operator()(i64 bn) -> void {
    numrl.clear();
    for(u8 index{0}; index!=Digits; index++) {
      numrl.push_front(bn / power(Radix, Digits-1-index));
        bn %= power(Radix, Digits-1-index); } }
          #ifdef DEBUG
  auto pr() -> void {
    for(size_t ix{}; size() != ix; ix++) ::pr(operator[](size()-1-ix)); ::pr(' '); }
          #endif
  auto size() -> const size_t { return numrl.size(); } };
  /* End Synthesis:: */ }
  void setup() __attribute__ ((noreturn));
  //https://gcc.gnu.org/onlinedocs/gcc/Common-Function-Attributes.html#index-noreturn-function-attribute
  //  No loop(){} haHahaha!
auto setup() -> void {
  using namespace Hardware;
  pinMode(static_cast<u8>(PIN::PDR), OUTPUT);       // Rf output enable
  rf( OFF );
  pinMode(static_cast<u8>(PIN::LE_A), OUTPUT);      // latch enable
  digitalWrite(static_cast<u8>(PIN::LE_A), 1);
  pinMode(static_cast<u8>(PIN::USR), INPUT_PULLUP); // SeeSaw interrupt
  pinMode(static_cast<u8>(PIN::LD_A), INPUT);       // lock detect
    // pinMode(static_cast<u8>(PIN::LE_B), OUTPUT);
    // digitalWrite(static_cast<u8>(PIN::LE_B), 1);
    // pinMode(static_cast<u8>(PIN::LD_B), INPUT);
  pinMode(static_cast<u8>(PIN::MUX), INPUT_PULLUP); // ignored
  Wire.begin();  Wire.setClock(400000L);
  SPI.begin();
  Timer1.initialize(10000000UL);  Timer1.attachInterrupt(Trigger);  // Inactivity timer
    #ifdef DEBUG
  Serial.begin(115200L);
    #endif
  OLED oled;  oled.begin(&Adafruit128x64, 0x3d);  oled.setContrast(0x40);
  using namespace Synthesis;
; SpecifiedOverlay pll;
  // Prior to flush(), quantiy I::_end calls of set() are required, in any order.
  //  handled later are: I::fraction, I::integer, I::modulus I::rfDivSelect      (1) (2) (3) (36)
  pll( I::phase, 1);                     // Adjust phase AFTER loop lock. Not redundant.      (4)
    //pll( I::phAdj, OFF );              // redundant                                      // (5)
  pll( I::prescaler,PRSCL::eight9ths );  // Possiblly redundant                            // (6)
  pll( I::counterReset, OFF );                                                             // (7)
  pll( I::cp3state, OFF );                                                                 // (8)
  pll( I::idle, OFF );                                                                     // (9)
  pll( I::pdPolarity, PDpolarity::positive );                                             // (10)
  pll( I::ldp, LDPnS::ten );                                                              // (11)
  pll( I::ldf, LockDetectFunction::fracN );                                               // (12)
  pll( I::cpIndex, 7 );  // 0 thru 15, 2.5mA = '7', cal increases loop bandwidth.           (13)
  pll( I::dblBfr, ON );                                                                   // (14)
  pll( I::rCounter, R_COUNT );                                                            // (15)
  pll( I::refToggler, TGLR );                                                             // (16)
  pll( I::refDoubler, DBLR );                                                             // (17)
  pll( I::muxOut, MuxOut::HiZ );          // see 'cheat sheet'                               (18)
  pll( I::LnLsModes, lowNoise );                                                          // (19)
  constexpr auto CLKDIV32{ 150 };         // I don't understand this, YET.
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
    //pll( I::rfOutPwr, minus4 );         // redundant                                    // (26)
  pll( I::rfSoftEnable, ON );                                                             // (27)
  pll( I::auxOutPwr, minus4 );                                                            // (28)
  pll( I::auxOutEnable, OFF );            // Pin not connected.                              (29)
  pll( I::muteTillLD, ON );                                                               // (30)
  pll( I::vcoPwrDown, OFF );                                                              // (31)
  pll( I::bndSelClkDv, u8(ceil(DBL(PFD) / MIN_PFD)) );                                    // (32)
  pll( I::rfFBselect, !Feedback );  // EEK! Why the negation? Perhaps I've been daVinci'd.   (33)
  pll( I::auxFBselect, !Feedback ); // See EEK!, above.                                      (34)
  pll( I::ledMode, LEDmode::lockDetect );                               // Ding. Winner!     (35)
  pll.phAdj(OFF);
; Resolver resolver(PFD, IOTA);
  struct Panel { Nmrl<8> Ref; Nmrl<1> Pwr; Nmrl<10> Frq; Nmrl<3> Lag; bool xmt, cal; Axis axs; };
  struct { Panel pnl; u16 check; } Checked; Panel& pnl{ Checked.pnl };
  XMEM xm; bool hasXM{ xm.begin() }; if( hasXM ) xm.readObject(0, Checked);
  if( ckMem(&pnl, sizeof(pnl)) != Checked.check ) { // default values
    pnl.Ref(OSC); pnl.Pwr(minus4); pnl.Frq(34*MHz + 375*kHz); pnl.Lag(1);
    pnl.xmt = ON, pnl.cal = ON; pnl.axs = Axis::REF; };
    rf(pnl.xmt);
  AFSS ss; bool hasSS{0}; if(ss.begin()) hasSS = (5740 == (0xFFFF & (ss.getVersion() >> 16)));
  if(hasSS) { ss.pinModeBulk(btnMask, INPUT_PULLUP); ss.setGPIOInterrupts(btnMask, 1); }
  for( bool toDevice{ON}, toHuman{ON}; ON; ) {
/**/if( toDevice ) { toDevice = 0;
      pnl.Ref(constrain(pnl.Ref(), 9*MHz, MAX_PFD));
      pnl.Pwr(constrain(pnl.Pwr(), minus4, plus5));
      pll(resolver(pnl.Pwr(), Axis::AMPL));
      resolver.pfd(pnl.Ref());
      pnl.Frq(constrain(pnl.Frq(), MIN_FREQ, MAX_FREQ));
      pll(resolver(pnl.Frq(), Axis::FREQ));
      pnl.Lag(constrain(pnl.Lag(), 1, pll().dnom-1));
      pll(resolver(pnl.Lag(), Axis::PHAS));
      pll.flush(); }
/**/if( toHuman && !timOut ) {
      oled.clear(); Timer1.start();
      if(pnl.cal) { oled.setFont(Adafruit5x7);  oled.setLetterSpacing(1); }
      /**/  else  { oled.setFont(X11fixed7x14); oled.setLetterSpacing(4); }
      if(pll.locked()) oled.print('+'); else oled.print('-');
      if(rf()) oled.print('+'); else oled.print('-');
        // I'm having an inherititance<N> mental block. N is getting in my way.
      switch(pnl.axs) {
        default:
        case Axis::HOLD:                                                                break;
        case Axis::FREQ:  oled.println("Frequency");  pnl.Frq.disp(oled,'\n');          break;
        case Axis::AMPL:  oled.println("Pwr");        pnl.Pwr.disp(oled,'\n');          break;
        case Axis::PHAS:  oled.println("Phase");      pnl.Lag.disp(oled,'\n');          break;
        case Axis::REF:   oled.println("RefOsc");     pnl.Ref.disp(oled,'\n',pnl.cal);  break; }
      if(pnl.cal && (Axis::HOLD != pnl.axs)) {
        oled.print("rpwr: ");   oled.print(pll().rpwr);
        oled.print(" rdiv: ");  oled.println(pll().rdiv);
        oled.print("dnom: ");   oled.print(pll().dnom);
        oled.print(" prop: ");  oled.println(pll().prop);
        oled.print("whol: ");   oled.print(pll().whol);
        oled.print(" numr: ");  oled.println(pll().numr);
        oled.print(" pfd: ");   oled.print(resolver.pfd());
            #ifdef DEBUG
        pr("rpwr:",pll().rpwr); pr("rdiv:",pll().rdiv); pr("prop:",pll().prop);
        pr("dnom:",pll().dnom); pr("whol:",pll().whol); pr("numr:",pll().numr);
        pr("pfd:"); Serial.print(resolver.pfd()); pr(' ');
        switch(pnl.axs) {
          default:
          case Axis::HOLD:
          case Axis::FREQ:  pnl.Frq.pr(); break;
          case Axis::AMPL:  pnl.Pwr.pr(); break;
          case Axis::PHAS:  pnl.Lag.pr(); break;
          case Axis::REF:   pnl.Ref.pr(); break; } pr('\n');
            #endif
        }
      toHuman = timOut = 0; }
/**/if( timOut ) { timOut = 0; Timer1.stop(); oled.clear(); snooz(); /* Awake here, just now. */ }
/**/if( hasSS && !digitalRead(USR) ) { // Service the human.
      if( auto action{ btns(ss) } ) { toHuman = 1; if(sft != action) switch(action) {
          default:                                                              break;
          case    inc:  toDevice = 1;
                    switch(pnl.axs) {
                      default:          // fall thru
                      case Axis::HOLD:                  toDevice = 0; break;
                      case Axis::FREQ:                  pnl.Frq(inc); break;
                      case Axis::AMPL:                  pnl.Pwr(inc); break;
                      case Axis::PHAS:                  pnl.Lag(inc); break;
                      case Axis::REF:   if(pnl.cal)     pnl.Ref(inc);
                                        else            toDevice = 0; break; }  break;
          case    dec:  toDevice = 1;
                    switch(pnl.axs) {
                      default:          // fall thru
                      case Axis::HOLD:                  toDevice = 0; break;
                      case Axis::FREQ:                  pnl.Frq(dec); break;
                      case Axis::AMPL:  if(pnl.Pwr())   pnl.Pwr(dec); break;
                      case Axis::PHAS:  if(1<pnl.Lag()) pnl.Lag(dec); break;
                      case Axis::REF:   if(pnl.cal)     pnl.Ref(dec);
                                        else            toDevice = 0; break; }  break;
          case    lft:  switch(pnl.axs) {
                      default:          // fall thru
                      case Axis::HOLD:                                break;
                      case Axis::FREQ:                  pnl.Frq(lft); break;
                      case Axis::AMPL:                  pnl.Pwr(lft); break;
                      case Axis::PHAS:                  pnl.Lag(lft); break;
                      case Axis::REF:   if(pnl.cal)     pnl.Ref(lft); break; }  break;
          case    rgt:  switch(pnl.axs) {
                      default:          // fall thru
                      case Axis::HOLD:                                break;
                      case Axis::FREQ:                  pnl.Frq(rgt); break;
                      case Axis::AMPL:                  pnl.Pwr(rgt); break;
                      case Axis::PHAS:                  pnl.Lag(rgt); break;
                      case Axis::REF:   if(pnl.cal)     pnl.Ref(rgt); break; }  break;
          case sft+inc: if(hasXM) { Checked.check = ckMem(&pnl, sizeof(pnl));
                                    xm.writeObject(0,Checked); }                break;
          case sft+rgt: pnl.cal = !pnl.cal;                                     break;
          case sft+lft: ++pnl.axs;                                              break;
          case sft+dec: rf( pnl.xmt = !rf() );                                  break; } } } } }