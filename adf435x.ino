/*
  https://github.com/151octal/adf435x/blob/main/LICENSE
  ©rwHelgeson[kd9fww] 2024, 2025.  ADF435x using ATMEGA328 hardware SPI.
    https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Source. A program (not a library).
    https://github.com/151octal/adf435x/blob/main/README.md <- Circuitry notes.
    https://www.analog.com/ADF4351 <- Datasheet of the device for which it is designed.
    https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet */
    #include "Adafruit_EEPROM_I2C.h"
    #include <Adafruit_GFX.h>
    #include "Adafruit_seesaw.h"
    #include <Arduino.h>
    #include <ArxContainer.h>
    #include <avr/sleep.h>
    #include <avr/wdt.h>
    #include "SSD1306Ascii.h"
    #include "SSD1306AsciiWire.h"
    #include <SPI.h>
    #include <TimerOne.h>
    #include <Wire.h>
  using i64 = long long;
  using u64 = unsigned long long;
  using AFSS = Adafruit_seesaw;
  using DBL = double;
  using OLED = SSD1306AsciiWire;  // Because the Adafruit library is too big. (both)
  using XMEM = Adafruit_EEPROM_I2C;
  enum Enable : u8 { OFF = 0, ON = 1 };
  constexpr auto& S{ Serial };  // sHORTHAND.
  constexpr auto& T1{ Timer1 }; // ditto
  auto pr( const char& cc = ' ' ) -> const size_t { return S.print(cc); }
  auto pr( const   u8& uc ) -> const size_t { return S.print(uc); }
  auto pr( const char* const s ) -> const size_t { return S.print(s); }
  auto pr( const u16& arg, char term = 0, int num = DEC ) -> const size_t {
    return S.print(arg, num) + pr(term); }
  auto pr( const char* const s, const u16& arg, char term = ' ', int num = DEC ) -> const size_t {
    return pr(s) + pr(arg, term, num); }
namespace Hardware {
  enum PIN : u8 { USR = 2, MUX = 4, PDR = 6, LD = 7, LE = 10, VS = A0, Aosc = A1, RLY = A3 };
  auto Tk{ [](const int i10){ return (100e-6+i10*1.1/1024/26.7e3)/226.5e-6*562; } }; // Kirchhoff
  enum Scale { Kelvin, Celsius, Fahrenheit };
  auto pT{ [](const DBL& tk, int scale, int n = 1, char t = ' '){ switch (scale) {
    default:  case Kelvin: return S.print(tk,n)+S.print(t);
              case Celsius: return S.print(tk-273,n)+S.print(t);
              case Fahrenheit: return S.print((tk-273)*9/5+32,n)+S.print(t);  } } };
  const auto ckMem = [](void* vp, size_t n) {
    auto p{ static_cast<u8*>(vp) };
    u16 sum{0}; while(n--) { sum += *(p++); sum %= 255; } return sum; };
  auto hardWait{ [](const PIN& pin){ while( !digitalRead( pin )); } };
  auto rf(bool enable) -> void { digitalWrite( PDR, enable ); };
  auto rf() -> const bool { return digitalRead( PDR ); }
  auto send = [](const PIN& le, void *pByte, int nByte){
    auto p{ static_cast<u8*>(pByte) + nByte };      // Most significant BYTE first.
    digitalWrite( le, 0 );                          // Predicate condition for data transfer.
    while( nByte-- ) SPI.transfer( *(--p) );        // Return value is ignored.
    digitalWrite( le, 1 ); };                       // Data is latched on the rising edge.
  auto sNudge{ [](){ sleep_disable(); detachInterrupt(digitalPinToInterrupt(USR)); } };
  auto snuz() -> void {  // lambda hostile macros contained herein.
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // From: Nick Gammon. https://www.gammon.com.au/power
    sleep_enable();
    noInterrupts(); // Begin critical section. (Needed?)
    attachInterrupt(digitalPinToInterrupt(USR), sNudge, LOW);
    EIFR = bit(INTF0);
    interrupts();   // End critical section.
    sleep_mode();
    detachInterrupt(digitalPinToInterrupt(USR));  }
  volatile bool vT1{ 0 };
  auto Trig1{ [](){ vT1 = 1; } };  // Timer1 ISR target
  volatile bool vAcq{ 1 };
  ISR(WDT_vect) { vAcq = 1; }
  auto wdtInit() -> void {  // More (ugly) macros.
    MCUSR = 0;  // From: Nick Gammon. https://www.gammon.com.au/power
    WDTCSR = bit(WDCE) | bit(WDE);
    WDTCSR = (bit(WDIE)) | (bit(WDP3)); // Four seconds?              Rewrite me for readablility.
    wdt_reset();  }
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
 constexpr  u16  IOTA{ 625 };                     // Choose IOTA such that R_COUNT is exact.
  constexpr auto  R_COUNT{ u16(OSC / IOTA / MOD) }; // REF / Rcounter = PFD = Modulus * IOTA
  constexpr auto  TGLR{ ON }, DBLR{ TGLR };        // OFF: ONLY if OSC is a 50% duty square wave.
 constexpr  auto  PFD{ DBL(OSC) * (1+DBLR) / (1+TGLR) / R_COUNT };
  static_assert(R_COUNT * IOTA == OSC / MOD);       // No remainder.
  static_assert((0<R_COUNT) && (1024>R_COUNT));     // Non-zero, 10 bits.
  static_assert((MAX_PFD >= PFD) && (MIN_PFD <= PFD));
 enum class Axis : u8 { FREQ = 1, AMPL, PHAS, REF };
    Axis& operator++(Axis& axs) { switch(axs) {
      case Axis::FREQ: axs = Axis::PHAS;  break;
      case Axis::PHAS: axs = Axis::AMPL;  break;
      case Axis::AMPL: axs = Axis::REF;   break;
      case Axis::REF:  axs = Axis::FREQ;  break; } return axs; }
    Axis& operator--(Axis& axs) { switch(axs) {
      case Axis::REF:  axs = Axis::AMPL;  break;
      case Axis::AMPL: axs = Axis::PHAS;  break;
      case Axis::PHAS: axs = Axis::FREQ;  break;
      case Axis::FREQ: axs = Axis::REF;   break; } return axs; }
  enum   ABPnS : u8 { nS6fracN = 0, nS3intN };            // AntiBacklash Pulse
  enum  Action : u8 { sft = 2, inc = sft<<1, lft = inc<<1, dec = lft<<1, rgt = dec<<1 };
  constexpr auto btnMask{ sft + inc + lft + dec + rgt };
  const auto btns{ [](AFSS& ss,const u8& msk = btnMask){
    return u8(msk^ss.digitalReadBulk(msk)); } };
  enum  BSCmd : u8 { programmed = 0, automatic };        // Band Select Clock mode
  enum  ClockingMode : u8 { dividerOff = 0, fastLock, phResync };
  enum  dBm : u8 { minus4 = 0, minus1, plus2, plus5 };
  constexpr enum FDBK : u8 { divided = 0, fundamental } Feedback = divided;
  enum  LDPnS : u8 { ten = 0, six };                     // Lock Detect Precision
  enum  LEDmode : u8 { low = 0, lockDetect = 1, high = 3 };
  auto  log2(const DBL& arg) -> DBL { return log10(arg) / log10(2); };
  enum  LockDetectFunction : u8 { fracN = 0, intN };
  enum  MuxOut : u8 { HiZ = 0, DVdd, DGnd, RcountOut, NdivOut, analogLock, digitalLock };
  enum  NoiseSpurMode : u8 { lowNoise = 0, lowSpur = 3 };
  constexpr auto  OVERLAYED_REGISTERS{ 6 };
  const auto pfdf = [](DBL r){ return r * (1+DBLR) / (1+TGLR) / R_COUNT; };
  enum  PRSCL : u8 { four5ths = 0, eight9ths };
  enum  PDpolarity : u8 { negative = 0, positive };
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
 constexpr struct Layout { const u8 RANK, OFFSET, WIDTH; } ADF435x[] {  /*
    //  https://gcc.gnu.org/onlinedocs/gcc/Designated-Inits.html
    Human deduced via inspection.
      OVERLAYED_REGISTERS:  Number of (32 bit) "registers".
      RANK:                 RANK = OVERLAYED_REGISTERS - 1 - Datasheet Register Number.
                            send() in ascending RANK order, unless not dirty. Thus, datasheet
                            register '0' is always send()'d last (and will always need to be sent).
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
    static_assert(Identifier::_end == (sizeof(ADF435x) / sizeof(Layout)));
  struct Parameters { u8 rpwr, rdiv; u16 dnom, whol, numr, prop; };
    constexpr Parameters INIT{ .rpwr = minus4, 0, .dnom = 1, .whol = 0, .numr = 0, .prop = 1 };
class Overlay {
  private:
    HW::PIN le{ HW::PIN::LE }, ld{ HW::PIN::LD };  // could be dynamic (multi-device)
    NoiseSpurMode nsMode = lowNoise;  // sloppy afterthought
    static const Layout* const layoutSpec;
    Parameters store{ INIT };
    struct Storage {
      static constexpr size_t NR{ OVERLAYED_REGISTERS };
      using RegArray = std::array<u32, NR>; /*
        With the exception of r5 bits 19 and 20, all 'reserved' bits are to be set to zero. These
        regions become invariants by not providing identifiers for them in the Specification. */
    u8 durty; SPISettings settings; RegArray reg; };
    using STO = Storage;
    STO ovl{ 0, SPISettings(4000000, MSBFIRST, SPI_MODE0), STO::RegArray{ 0x180005,4,3,2,1,0} };
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
    for(/* empty */; ovl.NR != cx; ++cx) HW::send(le, &ovl.reg[cx], sizeof(ovl.reg[cx]) );
    SPI.endTransaction();
    return *this; }
  auto lock() -> void { HW::hardWait(ld); } // Wait on active ld pin, until lock is indicated.
  auto locked() -> const int { return digitalRead( static_cast<u8>(ld) ); }
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
  auto operator()( const Parameters& loci ) -> decltype(*this) {
    set( I::fraction,loci.numr ).set( I::integer,loci.whol ).set( I::modulus,loci.dnom );
    set( I::phase,loci.prop ).set( I::rfDivSelect,loci.rdiv ).set( I::rfOutPwr,loci.rpwr );
    return *this;  }
  auto operator()() -> const decltype(store) { return store; }
  auto phAdj( const bool& e ) -> decltype(*this) { raw( I::phAdj,e ); return *this; }
    // Wrapper for operator()( sym,val )
  auto set( const I& sym,const u16& val ) -> decltype(*this) { return operator()( sym,val ); }
    // Wrapper for opertor()( loci )
  auto set( const Parameters& loci ) -> decltype(*this) { return operator()( loci ); }
} final; const Layout * const Overlay::layoutSpec{ ADF435x };
 class Resolver {
    private:  // Rotating phasor: f(t) = |magnitude| * pow( Euleran, j( omega*t + phi ))
      Parameters loci{ INIT };
      DBL _pfd; u16 spacing;
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
        return i64(_pfd * (loci.whol + DBL(loci.numr) / loci.dnom) / pow(2,loci.rdiv)); }
      auto omega(const i64& bn) -> const decltype(loci) {
        auto freq{ constrain(bn, MIN_FREQ, MAX_FREQ) };
        loci.rdiv = u16( floor( log2(MAX_VCO/freq) ) );
        auto fractional_N{ (freq / _pfd) * pow(2, loci.rdiv) };
        loci.whol = u16( trunc( fractional_N ) );
        //loci.whol = (22 < loci.whol) ? loci.whol : 22;
        loci.dnom = u16( ceil( OSC / R_COUNT / spacing ) );
        loci.numr = u16( round( (fractional_N - loci.whol) * loci.dnom) );
        return loci; }
      auto pnum() -> const i64 { return loci.prop; }
      auto pnum(const i64& bn) -> const decltype(loci) {  // Phase NUMerator: phase = prop / denom
        loci.prop = constrain(bn, 1, loci.dnom-1 ); return loci; }
    public:
    Resolver( const DBL& pfd, const u16& step ) : _pfd{ pfd }, spacing{ step } {}
    auto operator()(const i64& bn, Axis axs = Axis::FREQ) -> const decltype(loci) { switch(axs) {
      default:
      case Axis::FREQ:  return omega(bn);
      case Axis::AMPL:  return amplitude(static_cast<dBm>(bn));
      case Axis::PHAS:  return pnum(bn); } }//phi(DBL(bn)); } }
        // Resolver value dispatcher. Returns Axis selective value from Parameters
    auto operator()(Axis axs = Axis::FREQ) -> const i64 {
      switch (axs) {
        default:
        case Axis::FREQ:  return omega();
        case Axis::AMPL:  return static_cast<i64>(amplitude());
        case Axis::PHAS:  return pnum(); } }
    auto pfd() -> const DBL { return _pfd; }  // Phase Frequency Detector frequency
    auto pfd(const DBL& f) -> void { _pfd = f; } };
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
    auto disp( OLED& oled, char term = '\n', const bool carat = true ) -> void {
      const auto pstn{ cursor() };
      const auto ivmd{ oled.invertMode() };
      for(size_t x{}; size() != x; x++) {
        if(carat) { if(size()-1-pstn == x)  oled.setInvertMode(1);
        /**/        /**/              else  oled.setInvertMode(0); }
        oled.print(operator[](size()-1-x)); }
      oled.setInvertMode(ivmd); oled.print(term); }
    auto operator[](const size_t& pstn) -> const u8 { return numrl[constrain(pstn, 0, Digits-1)]; }
    auto operator()() -> const i64 {
      i64 sum{0};
      for(u8 idx{0}; idx!=numrl.size(); idx++) sum += operator[](idx) * power(Radix, idx);
      return sum; }
    auto operator()(const Action& act) -> void { switch(act) {
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
    auto pr(char term = ' ') -> void {
      for(size_t ix{}; size() != ix; ix++) ::pr(operator[](size()-1-ix)); ::pr(term); }
    auto size() -> const size_t { return numrl.size(); } };
/* End Synthesis:: */ }
  void setup() __attribute__ ((noreturn));
  //https://gcc.gnu.org/onlinedocs/gcc/Common-Function-Attributes.html#index-noreturn-function-attribute
  //  No loop(){} haHahaha!
  //  "Where shall I begin, please, your Majesty?" he asked. "Begin at the beginning," the King
  //  said gravely, "ang go on till you come to the end: then stop." 'Alice's Advetures in Wonderland'
  //  Lewis Carroll. London: Macmillan and Co. 1865. 
auto setup() -> void {
  using namespace Hardware;
  pinMode(PDR, OUTPUT);       // Rf output enable
  rf( OFF );
  pinMode(LE, OUTPUT);        // (spi) latch enable
  digitalWrite(LE, 1);
  pinMode(USR, INPUT_PULLUP); // SeeSaw interrupt
  pinMode(LD, INPUT);         // lock detect
  pinMode(MUX, INPUT_PULLUP); // ignored
  pinMode(VS, OUTPUT);        // Sensor supply
  digitalWrite(VS, OFF);
  pinMode(Aosc, INPUT);       // Oscillaror Temperature Sensor
  pinMode(RLY, OUTPUT);       // Relay exciter, active low
  digitalWrite(RLY, 1);
  analogReference(INTERNAL); auto raw{analogRead(A1)}, old{ raw };  // prime the adc
  Wire.begin();  Wire.setClock(400000L);  SPI.begin();
  S.begin(115200L);
  OLED oled; oled.begin(&Adafruit128x64, 0x3d); oled.setContrast(0x20);
  oled.setFont(X11fixed7x14); oled.setLetterSpacing(4);
  using namespace Synthesis;
; Overlay pll;
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
  pll( I::cpIndex, 7 );   // 0 thru 15, 2.5mA = '7', cal increases loop bandwidth.           (13)
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
  pll( I::muteTillLD, OFF );              // ON: contrary to wdt update scheme            // (30)
  pll( I::vcoPwrDown, OFF );                                                              // (31)
  pll( I::bndSelClkDv, u8(ceil(DBL(PFD) / MIN_PFD)) );                                    // (32)
  pll( I::rfFBselect, !Feedback );  // EEK! Why the negation? Perhaps I've been daVinci'd.   (33)
  pll( I::auxFBselect, !Feedback ); // See EEK!, above.                                      (34)
  pll( I::ledMode, LEDmode::lockDetect );                               // Ding. Winner!     (35)
  pll.phAdj(OFF);
    // Hardware not required but written for Adafruit SeeSaw P/N: 5740 only.
; AFSS ss; bool hasSS{0}; if(ss.begin()) hasSS = (5740 == (0xFFFF & (ss.getVersion() >> 16)));
  long knob; if(hasSS) { knob = ss.getEncoderPosition(); ss.enableEncoderInterrupt();
  /**/ss.pinModeBulk(btnMask,INPUT_PULLUP); ss.setGPIOInterrupts(btnMask,ON);
  /**/T1.initialize(10000000UL); T1.attachInterrupt(Trig1); }
  struct Persist {  Nmrl<8> Ref; Nmrl<1> Pwr; Nmrl<10> Frq; Nmrl<3> Lag;
  /**/              int raw; long osc; Axis axs; bool xmt, hld, lup; };
  struct { Persist P; u16 sum; } Mem; Persist& P{ Mem.P };
  XMEM X; bool hasX{ X.begin() }; if( hasX ) X.readObject(0, Mem);
  if( ckMem(&P, sizeof(P)) != Mem.sum ) { // default values
    P.Ref(OSC);/* P.Pwr(minus4); P.Frq(34*MHz + 375*kHz);*/
    P.axs = Axis::REF; P.xmt = OFF; P.hld = OFF; P.lup = ON; /*}*/
    P.Lag(1); P.Pwr(minus4); P.Frq(100*MHz);
    P.raw = 571; P.osc = 24999798; };
  auto xmit{ [&pll](bool b) { rf(b); pll(I::idle,!b).set(I::cp3state,!b).flush();
                              digitalWrite(RLY, !b); } };
  xmit(P.xmt); Resolver rslv(pfdf(P.Ref()), IOTA); wdtInit(); long refHat{ OSC };
; for( bool toDev{ON}, toHuman{ON}; ON; ) {
/**/if( vAcq ) { /* HW::wdtInit() sets WD Timeout interval. */
      constexpr auto RC{ 26.7e3 /*ohm*/ * 39e-9 /*farad*/ }; // roughly one milliSecond
      constexpr auto settleTime{ u32(ceil(7 * 1e3 * RC)) };  // 10bits; ln(999) * RC ≈ 7RC
      digitalWrite(VS, ON); delay(settleTime); /* Power the sensor and wait for it to settle. */
      raw = analogRead(A1);
      digitalWrite(VS, OFF); noInterrupts(); vAcq = 0; interrupts();
      if(old != raw && !P.lup) { old = raw; toHuman = 1; }
      constexpr auto TempCoef{ -12.6 }; // Units: Hertz per ºK
      refHat = P.osc + (Tk(raw) - Tk(P.raw)) * TempCoef;  // Ref estimate
      if(P.lup) { if( auto dR{ int(P.Ref() - refHat) } ) { constexpr u8 nu{ 4 };
  /**/  if((dR < 0 ? -dR : dR) > nu) { P.Ref( refHat ); toHuman = toDev = 1; pr(); } } } }
/**/if( toDev ) {
      P.Pwr(constrain(P.Pwr(), minus4, plus5));
      pll(rslv(P.Pwr(), Axis::AMPL));
      P.Ref(constrain(P.Ref(), 9*MHz, MAX_PFD));
      rslv.pfd(pfdf(P.Ref()));  // Results of Resolver:: are dependent on Ref value.
      P.Frq(constrain(P.Frq(), MIN_FREQ, MAX_FREQ));
      pll(rslv(P.Frq(), Axis::FREQ));
      P.Lag(constrain(P.Lag(), 1, pll().dnom-1));
      pll(rslv(P.Lag(), Axis::PHAS));
      pll.flush(); toDev = 0; }
/**/if( toHuman ) {
      oled.clear(); T1.start();
      oled.print(rf() ? " RUN " : "IDLE ");
      oled.print(pll.locked() ? "LOCK " : "UNLK ");
      oled.println(P.lup ? "AUT" : "MAN");
      switch(P.axs) {
        default:
        case Axis::FREQ:  P.Frq.disp(oled,'\n',!P.hld); break;
        case Axis::AMPL:  P.Pwr.disp(oled,'\n',!P.hld); break;
        case Axis::PHAS:  P.Lag.disp(oled,'\n',!P.hld); break;
        case Axis::REF:   P.Ref.disp(oled,'\n',!P.hld); break; }
      oled.print(raw); oled.print(' ');
      oled.println(Tk(raw)-273,1);
      S.print(raw); pr();/**/ pT(Tk(raw),Celsius,1);
      if(!P.lup) { pr('w'); S.print(refHat); pr(); }
      pr('r'); P.Ref.pr();
      S.print(rf() ? 'R' : '\0'); S.print(pll.locked() ? 'L' : '\0');
      S.print(P.lup ? 'A' : '\0'); S.println(P.hld ? '\0' : '!');
      noInterrupts(); toHuman = vT1 = 0; interrupts(); }
/**/if( hasSS ) { if( !digitalRead(USR) ) { // SeeSaw interrupt
        if( auto diff{ss.getEncoderPosition() - knob}) {
          if(0 < diff) ++P.axs; else --P.axs;
          knob = ss.getEncoderPosition(); toHuman = 1; }
        else { if( auto act{btns(ss)} ) {
        /**/toHuman = 1;
        /**/if(sft == act) { while(btns(ss) == sft); }
        /**/else { switch(act) {
                case sft+inc: if(hasX) { if(!P.lup && !P.hld) { P.lup = ON; P.hld = OFF;/**/
                                P.raw = raw; P.osc = P.Ref(); pr('K'); }
                                Mem.sum = ckMem(&P, sizeof(P)); X.writeObject(0,Mem); }
                                pr(" STO\n"); break;
                case sft+lft: P.lup = !P.lup; break;
                case sft+dec: toDev = 1; xmit( P.xmt = !rf() ); break;/**/
                case sft+rgt: P.hld = !P.hld; break;
                case    inc:  if(!P.hld) {
                          toDev = 1;
                          switch(P.axs) {
                            case Axis::FREQ:  P.Frq(inc); break;
                            case Axis::AMPL:  P.Pwr(inc); break;
                            case Axis::PHAS:  P.Lag(inc); break;
                            case Axis::REF:   P.Ref(inc); break; } }  break;
                case    dec:  if(!P.hld) {
                          toDev = 1;
                          switch(P.axs) {
                            case Axis::FREQ:  P.Frq(dec); break;
                            case Axis::AMPL:  if(P.Pwr()) P.Pwr(dec); break;
                            case Axis::PHAS:  P.Lag(dec); break;
                            case Axis::REF:   P.Ref(dec); break; } }  break;
                case    lft:  if(!P.hld) {
                          toDev = 1;
                          switch(P.axs) {
                            case Axis::FREQ:  P.Frq(lft); break;
                            case Axis::AMPL:  P.Pwr(lft); break;
                            case Axis::PHAS:  P.Lag(lft); break;
                            case Axis::REF:   P.Ref(lft); break; } }  break;
                case    rgt:  if(!P.hld) {
                          toDev = 1;
                          switch(P.axs) {
                            case Axis::FREQ:  P.Frq(rgt); break;
                            case Axis::AMPL:  P.Pwr(rgt); break;
                            case Axis::PHAS:  P.Lag(rgt); break;
                            case Axis::REF:   P.Ref(rgt); break; } }  break;
                default:  break; }
        /**/  while(btns(ss)); } pr('*'); } } } }
/**/noInterrupts(); auto wink{ (P.lup && vT1) && !vAcq && !toDev && !toHuman }; interrupts();
/**/if( wink ) {  T1.stop(); oled.clear(); noInterrupts(); vT1 = 0; interrupts(); snuz(); } } }