/* ©2024 kd9fww. ADF435x stand alone using Arduino Nano hardware SPI (in ~300 lines, ~9K mem).
  https://github.com/151octal/adf435x/blob/main/adf435x.ino <- Where you got this code.
  https://github.com/151octal/adf435x/blob/main/README.md <- Circuitry notes.
  https://www.analog.com/ADF4351 <- The device for which this code is specifically tailored.
  https://ez.analog.com/rf/w/documents/14697/adf4350-and-adf4351-common-questions-cheat-sheet */
#include <ArxContainer.h>
#include <SPI.h>
  enum Enable { OFF = 0, ON = 1 };
  // Shorthand
  void pr(                       const char& cc ) { Serial.print(cc); }
  void pr(   const double& arg,     int num = 0 ) { Serial.print(arg, num); pr(' '); }
  void pr(     const  u16& arg,   int num = DEC ) { Serial.print(arg, num); pr(' '); }
  void pr(     const  u32& arg,   int num = DEC ) { Serial.print(arg, num); pr(' '); }
  void pr( const char* const s, const  u16& arg, int num = DEC ) { Serial.print(s); pr(arg,num); }
  void pb(     const  u32& arg,      u8 nb = 32 ) { while( nb ) { switch(nb) {  // Print binary...
      default: break;  case 8: case 16: case 24: pr(' '); break; }  // in byte sized chunks,
    pr( ((1UL << --nb) & arg) ? '1' : '0' ); } pr(' '); }           // one bit at a time.
; namespace System {
  enum PIN : u8 { encJ = 2, encK = 3, MUX = 4, PDR = 6, LD_A = 7, LE_A = 10 };
  enum UNIT { A, /* B, */ _end };
  constexpr struct CTRL { PIN le, ld; } ctrl[] = { [A] = { LE_A, LD_A } };/*
    , [B] = { LE_B, LD_B } };*/
  static_assert(UNIT::_end == sizeof(ctrl) / sizeof(ctrl[0]));
  const auto hardWait = [](const PIN& pin) { while( !digitalRead( static_cast<u8>(pin) )); };
  auto  log2(double arg) -> double { return log10(arg) / log10(2); };
  const auto rf = [](bool enable) { digitalWrite( static_cast<u8>(PIN::PDR), enable ); };
  const auto tx = [](const PIN& le, void *pByte, int nByte) {
    auto p = static_cast<u8*>(pByte) + nByte;       // Most significant BYTE first.
    digitalWrite( static_cast<u8>(le), 0 );         // Predicate condition for data transfer.
    while( nByte-- ) SPI.transfer( *(--p) );        // Return value is ignored.
    digitalWrite( static_cast<u8>(le), 1 ); };      /* Data is latched on the rising edge. */
} namespace Synthesis {
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
} namespace State { struct Parameters { u16 divis, whole, denom, numer, propo; u8 rfpwr; };
  constexpr Parameters INIT{ 0, 0, 1, 0, 1, Synthesis::minus4 };
} namespace Synthesis {
  constexpr auto N_REGISTERS{ 6 };
  /* ©2024 kd9fww */
class SpecifiedOvelay {
  private:
    System::PIN le{ System::ctrl[System::UNIT::A].le }, ld{ System::ctrl[System::UNIT::A].ld };
    NoiseSpurMode nsMode = lowNoise;
    static const LayoutSpecification* const layoutSpec;
    State::Parameters store{ State::INIT };
    struct Overlay {
      static constexpr size_t N{ N_REGISTERS };
      using RegArray = std::array<u32, N>; /*
        With the exception of r5 bits 19 and 20, all "reserved" bits are to be set to zero. These
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
    for(/* empty */; ovl.N != cx; ++cx) System::tx(le, &ovl.reg[cx], sizeof(ovl.reg[cx]) );
    SPI.endTransaction();
    return *this; }
  auto lock() -> void { System::hardWait(ld); } // Wait on active ld pin, until lock is indicated.
  auto operator()( const System::PIN _le, System::PIN _ld ) -> decltype(*this) {
    le = _le; ld =_ld; return *this; }
  auto operator()( const System::CTRL& io ) -> decltype(operator()(io.le, io.ld)) {
    return operator()(io.le, io.ld); }
  auto operator()( const S& sym,const u16& val ) -> decltype(raw( sym,val )) { switch(sym) {
      default:                                      return raw( sym,val );
      case S::fraction:     if(val !=  store.numer) return raw( sym,store.numer  = val );
      case S::integer:      if(val !=  store.whole) {
                      raw( S::prescaler,(75 < val) ? PRSCL::eight9ths : PRSCL::four5ths );
                                                    return raw( sym,store.whole  = val ); }
      case S::phase:        if(val !=  store.propo) return raw( sym,store.propo  = val );
      case S::modulus:      if(val !=  store.denom) {
                        raw( S::LnLsModes,nsMode =
                        (lowSpur == nsMode) ? ((50 > val) ? lowNoise : nsMode) : nsMode);
                                                    return raw( sym,store.denom  = val ); }
      case S::rfDivSelect:  if(val !=  store.divis) return raw( sym,store.divis  = val );
      case S::rfOutPwr:     if(static_cast<dBm>(val) != store.rfpwr){
                                    raw( S::rfSoftEnable, ON );
                                    return raw( sym,store.rfpwr = static_cast<dBm>(val) ); } } }
  auto operator()( const State::Parameters& loci ) -> decltype(*this) {
    set( S::fraction,loci.numer ).set( S::integer,loci.whole ).set( S::modulus,loci.denom );
    set( S::phase,loci.propo ).set( S::rfDivSelect,loci.divis ).set( S::rfOutPwr,loci.rfpwr );
    return *this;  }
  auto operator()() -> const State::Parameters {
    return { store.divis, store.whole, store.denom, store.numer, store.propo, store.rfpwr }; }
  auto phAdj( const bool& e ) -> decltype(*this) { raw( S::phAdj,e ); return *this; }
  auto set( const S& sym,const u16& val ) -> decltype(*this) { return operator()( sym,val ); }
  auto set( const State::Parameters& loci ) -> decltype(*this) { return operator()( loci ); }
} /* End SpecifiedOvelay */ final;  using SO = SpecifiedOvelay;
const LayoutSpecification * const SO::layoutSpec{ ADF435x }; 
/* End Synthesis:: */ } namespace Manifest  {       // Manifest data ...
  constexpr auto  MIN_PFD{125e3}, MAX_PFD{045e6};   // (in units of Hertz)
  constexpr auto  MIN_VCO{2.2e9}, MAX_VCO{4.4e9};   // ... from the datasheet
  constexpr auto  MIN_FREQ{ MIN_VCO / 64 }, MAX_FREQ{ MAX_VCO };
/* End Manifest:: */  } namespace Synthesis {       // Details. Because, comments don't get tested
  constexpr  auto COMP{ ON };                       // OFF: No OSCillator error COMPensation.
   constexpr auto CONSTRAINT{ 1e1 };                // 'digit(s) lost' Assertion failure avoidance
  constexpr  auto CORRECTION{ -15 * CONSTRAINT };   // Determined by working in reverse, from the
   constexpr auto REF_ERROR{ (COMP) * CORRECTION }; // value of REF, as measured, below.
  constexpr auto  OSC{ 25e6 };                      // Nominal osc. freq. Yours may be different.
   constexpr auto REF{ OSC + REF_ERROR };           // Measured osc. freq. YOURS WILL BE DIFFERENT
   static_assert( 0 == (REF - OSC) - REF_ERROR, "Least significant digit(s) lost." );
  constexpr auto  RESOLUTION{ 0.5e3 };             // REF / Rcounter = PFD = Modulus * Resolution
  constexpr auto  R_TGLR{ OFF }, R_DBLR{ R_TGLR };  // OFF: ONLY if OSC IS a 50% duty square wave.
   constexpr auto BASE{ 5 }, EXP_2{ BASE * BASE }, EXP_4{ EXP_2 * EXP_2 }, EXP_5{ EXP_4 * BASE };
   // Choose a Prpduct, not divisible by {2,3}.
  constexpr enum  PICK { SML = 0, LRG } size = LRG; // SML: Longer lock time.
    constexpr auto MODULUS{ size ? EXP_5 : EXP_4 }; // The same entity as that of the datasheet.
    static_assert(4096 > MODULUS);                  // 12 bits.
    static_assert((MODULUS % 2) || (MODULUS % 3), "Spur avoidance. It is worth the effort.");
    constexpr auto  REF_COUNTER{ u16(OSC / RESOLUTION / MODULUS) };
    static_assert( (0 < REF_COUNTER) && (1024 > REF_COUNTER) );     // Non-zero, 10 bits.
    // PFD: The (scaled) Phase Frequency detector's reference input frequency.
    // Important: REF_ERROR is propagated by it's inclusion in the calculation of PFD
    constexpr auto  PFD = REF * (1 + R_DBLR) / (1 + R_TGLR) / REF_COUNTER;
    static_assert( (Manifest::MIN_PFD <= PFD) && (Manifest::MAX_PFD >= PFD) );
  enum Axis { FREQ, PHAS, AMPL, STEP };
    /* ©2024 kd9fww */
class Marker {
  using DBL = double;
  private:
    // Rotating phasor: f(t) = |magnitude| * pow( Euleran, j( omega*t + phi ))
    // Where: Amplitude <- |magnitude|, Frequency <- omega, and Phase <- phi, are all scalars.
    State::Parameters loci{ State::INIT };
    DBL pfd, spacing;
    auto amplitude() -> const u8 { return loci.rfpwr; }
    auto amplitude(const dBm& a) -> const decltype(loci) { loci.rfpwr = a; return loci; }
    auto delta() -> const decltype(spacing) { return spacing; }
    auto delta(const DBL& step) -> const decltype(loci) { spacing = step; return loci; }
    auto phi() -> const DBL { return (loci.propo / DBL(loci.denom - 1)); }
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
      loci.divis = u16( floor( System::log2(Manifest::MAX_VCO / freq) ) );
      auto fractional_N{ freq / pfd * pow(2, loci.divis) };
      loci.whole = u16( floor( fractional_N ) );
      loci.whole = (22 < loci.whole) ? loci.whole : 22;
      loci.denom = u16( round( pfd / spacing ) );
      loci.numer = u16( round( (fractional_N - loci.whole) * loci.denom) );
      return loci;  }
  public:
  Marker( const DBL& actual_pfd = PFD, const DBL& step = RESOLUTION )
    : pfd{ actual_pfd }, spacing{ step } {}
  auto dump() -> void {
    using namespace System;
    pr(operator()()); pr(operator()(AMPL)); pr(operator()(PHAS),4); pr('\n'); }
  auto operator()(DBL arg, Axis axis = FREQ) -> const decltype(loci) { switch(axis) {
    default:
    case AMPL:  return amplitude(static_cast<dBm>(arg));
    case FREQ:  return omega(arg);
    case PHAS:  return phi(arg);
    case STEP:  return delta(arg); } }
      // Marker value dispatcher. Returns Axis selective value from State::Parameters
  const auto operator()(Axis axis = FREQ) -> const decltype(omega()) {
    switch (axis) {
      default:
      case AMPL:  return static_cast<double>(amplitude());
      case FREQ:  return omega();
      case PHAS:  return phi();
      case STEP:  return delta(); } 
}/* end Marker:: */ }; /* End Synthesis:: */ }
  #ifdef  degrees
  #define please_dont_pollute_the_global_namespace "What a bummer, eh?" Ian Anderson.
  #endif  // "Good grief, Charlie Brown." C.M. Schulz.
  /* "How shall I tell you the story?" The King replied, "Start at the beginning. Proceed
  until the end. Then stop." Lewis Carroll. "Alice's Adventures in Wonderland". 1865. */
auto setup() -> void {  using namespace System;     // "And away we go ..." Gleason.
  SPI.begin();
  /* digitalWrite(static_cast<u8>(PIN::MUX), INPUT_PULLUP); */
  pinMode(static_cast<u8>(PIN::PDR), OUTPUT); // Rf output enable.
  rf( OFF );
  pinMode(static_cast<u8>(PIN::LE_A), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::LE_A), 1);
  pinMode(static_cast<u8>(PIN::LD_A), INPUT); /*
  pinMode(static_cast<u8>(PIN::LE_B), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::LE_B), 1);
  pinMode(static_cast<u8>(PIN::LD_B), INPUT); */ }
     // Jettson[George]: "Jane! JANE! Stop this crazy thing! JANE! !!!".
auto loop() -> void {  using namespace Synthesis;
  Serial.begin(1000000L); delay(1000L);
; SO pll;    { /* Enter another scope. */ SO temp; /*
  Quantiy S::_end calls of set() are required, in any order. Four set() calls are made for each
  mk.freq(double). So, S::_end - 4, remaining. */
  //                                         S::fraction, S::integer, S::modulus       (1) (2) (3)
  temp( S::phase, 1);                     // Adjust phase AFTER loop lock. Not redundant.      (4)
  temp( S::phAdj, OFF );                                                                 // (5)
  temp( S::prescaler,PRSCL::eight9ths );  // Possiblly redundant                            // (6)
  temp( S::counterReset, OFF );                                                             // (7)
  temp( S::cp3state, OFF );                                                                 // (8)
  temp( S::idle, OFF );                                                                     // (9)
  enum PDpolarity { negative = 0, positive };
  temp( S::pdPolarity, PDpolarity::positive );                                             // (10)
  enum LDPnS { ten = 0, six };            // Lock Detect Precision nanoSeconds
  temp( S::ldp, LDPnS::ten );                                                              // (11)
  enum LockDetectFunction{ fracN = 0, intN };
  temp( S::ldf, LockDetectFunction::fracN );                                               // (12)
  temp( S::cpIndex, 7 );  // 0 thru 15, 2.5mA = '7', more increases loop bandwidth.           (13)
  temp( S::dblBfr, ON );                                                                   // (14)
  temp( S::rCounter, Synthesis::REF_COUNTER );                                             // (15)
  temp( S::refToggler, Synthesis::R_TGLR );                                                // (16)
  temp( S::refDoubler, Synthesis::R_DBLR );                                                // (17)
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
  temp( S::csr, ON );                  // Cycle Slip reduction                                (22)
  temp( S::chrgCancel, OFF );                                                              // (23)
  enum ABPnS { nS6fracN = 0, nS3intN };   // AntiBacklash Pulse nanoSeconds
  temp( S::abp, ABPnS::nS6fracN );                                                         // (24)
  enum BndSelClkMd { automatic = 0, programmed };
  temp( S::bscMode,
  (Manifest::MIN_PFD < Synthesis::PFD) ? BndSelClkMd::automatic : BndSelClkMd::programmed);// (25)
  temp( S::rfOutPwr, minus4 );            // Possiblly redundant                           // (26)
  temp( S::rfSoftEnable, ON );                                                             // (27)
  temp( S::auxOutPwr, minus4 );                                                            // (28)
  temp( S::auxOutEnable, OFF );        // Pin not connected. So, I can't test it.             (29)
  constexpr enum FDBK { divided = 0, fundamental } Feedback = divided;
  temp( S::auxFBselect, !Feedback );                                                       // (30)
  temp( S::muteTillLD, ON );                                             // put me back on // (31)
  temp( S::vcoPwrDown, OFF );                                                              // (32)
  auto BscClkDiv = ceil((Synthesis::PFD / Manifest::MIN_PFD));
  //  static_assert( (0 < BscClkDiv) && (256 > BscClkDiv) ); // Non-zero, 8 bit value.
  temp( S::bndSelClkDv, u8(BscClkDiv) );                                                   // (33)
  // S::rfDivSelect                                                                           (34)
  temp( S::rfFBselect, !Feedback );   /* EEK! Why the negation?                               (35)
  It works NEGATED. I'm stumped. Perhaps I've been daVinci'd. */
  enum LEDmode { low = 0, lockDetect = 1, high = 3 };
  temp( S::ledMode, LEDmode::lockDetect );                                 // Ding. Winner!   (36)
; pll = temp; } /* Save and exit scope (discarding temp). */
  Marker mk;
  using namespace System;
  pr('\n');
  pll( ctrl[A] ).set( mk(Synthesis::dBm::minus1,AMPL) ).phAdj(OFF).set(mk(0/360.,PHAS));
  rf(ON);  //pll( ctrl[B] ).phAdj(ON).set(mk(180/360.,PHAS)).flush().lock() );
  bool dir{ true }, debug{ 1 };
  auto df{ 1e3 }, bot{ 68.748e6 }, top{ 68.752e6 }, f{ bot };
while(1) {
    delay(2222);
    f += dir ? df : -df;
    if(top < f) { dir = false; } else
    if(bot > f) { dir =  true; }
    pll(mk( f )).flush().lock();    //pll( mk(60/360.,PHAS) ).phAdj(ON).flush();
    if(debug) { auto loci = pll();
      pr("rfpwr:", loci.rfpwr); pr("divis:", loci.divis); pr("denom:", loci.denom);
      pr("propo:", loci.propo); pr("whole:", loci.whole); pr("numer:", loci.numer);
      pr(f); mk.dump(); }} /* End loop() */ } // kd9fww