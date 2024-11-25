// kd9fww. (c) 2024. ADF435x stand alone using Arduino Nano hardware SPI.
  // https://github.com/151octal/adf435x/blob/main/pll.ino    hoo! hoo!
#include <SPI.h>
#include <ArxContainer.h>
  /*  bi-directional level shifter module [P/N: TXS0108E hereafter referred to as: shft'r]
  Assembly gist: The shft'r is post † soldered to the nano such that the pins b1..b6 directly
  connect to the nano's pcb pins h9..h14. Wire-wrap the rest. Limit to 5cm and common mode choke
  the aggregate of (qty:9) wires connecting the ADF435x module. Supply the ADF435x from the nano's
  on board 5v regulator output. Use the ADF435x onboard 3v regulator output to supply the 3v3
  needed by the shft'r. Implement NO DC ground loop(s) ††.
    https://en.wikipedia.org/w/index.php?title=Ground_loop_(electricity)
  pin mapping legend: { -w-wire-w-, =p=post=p= }
  scheme: nano { num:(nano name) } -=- lvl shft'r { b.label | a.label(pll name) } -=- pll { num }
  where:
    h.x: nano pcb 'header' pin number   notes:  i) pin h.1 has a square (solder) pad
    b.x: level shft 5V logic pins              ii) shifter's supplies: v.b > v.a  and  v.a >= 0
    a.x: level shft 3V logic pins                    --> (nominals) 5.1 = v.b & 3.3 = v.a <--
                    nano                shft'r              pll module
                    xx pins             18 pins             9 wires
  system datum: single point ground-w-GND-+-w--------------w-7    |  ADF435x module pin header   |
  h.4:(GND)-w-----------------------w-GND | oe-w---------w-a.v    |     component side view      |
  h.16:D13(SCK)-w-------------------w-b.8 | a.8(clk)-w-----w-4    |------------------------------|
  h.15:D12(MISO) <- open circuit: ignore data FROM the pll        | [[ gnd ]] | 0 | 9 | 3v3      |
  h.7:D4(T0)-w----------------------w-b.7 | a.7(mux)-w-----w-3    | [[  ce ]] | 8 | 7 | GND      |
  h.14:D11(MOSI)=p==================p=b.6 | a.6(dat)-w-----w-5    |        le | 6 | 5 | dat      |
  h.13:D10(SS*)=p===================p=b.5 | a.5(le)--w-----w-6    |       clk | 4 | 3 | mux      |
  h.12:D9=p=========================p=b.4 | a.4  available.0      |        ld | 2 | 1 | pdr      |
  h.11:D8=p=========================p=b.3 | a.3  available.1      |------------------------------|
  h.10:D7=p=========================p=b.2 | a.2(ld)--w-----w-2    | leave '0' open to provide a  |
  h.9:D6=p==========================p=b.1 | a.1(pdr)-w-------1    | [[sleeve]] jumper to '8'     |
                                                                  | my unit has pull-up R's      |
  (from nano.reg5)  h.27:(5V)-w-----w-b.v | v.a(3v3)-w-----w-9  (NOT h.17 !)
                                      b.v-+-w--------------w-5v (to supply of pll.reg3.3)
  system.pwr.return-h.29:(GND: nano.reg5 return [pcb ground plane])
  system.pwr.supply-h.30:(VIN: nano.reg5 supply)
  † posts: equal length, stiff, solderable, conductors that fit in the holes - don't use bus wire.
  †† Faraday enclosures bonded to earth: one high value X7R between GND and EARTH IS better than
  a DC short. *//* Yes. The nano LED is in contention with the default SPI clock line and is not
  easily open circuited. Look, let it be and 'fugget about it', OK? SPI will work regardless.   */
  // commented out, but wired:     D4                                      D11       D13
enum class PIN : u8 {    /* MUX = 4, */ PDR = 6, LD = 7, LE = 10 /* DAT = 11, CLK = 13 */ };
auto wait4lock = []() { while( !digitalRead( static_cast<u8>(PIN::LD) )); }; // block til lock
static auto weight = [](int index) {
  static constexpr u8 WeightTable[] = { 1, 2, 4, 8, 16, 32 };
  return WeightTable[index]; };
auto tx(void *pByte, int nByte) -> void {   // SPI stuff here
  auto p = static_cast<u8*>(pByte) + nByte; // most significant BYTE first
  digitalWrite( static_cast<u8>(PIN::LE), 0 );
    // ascending Specification order (see below) <- block handling of multiple u32s for speed
  while( nByte-- ) SPI.transfer( *(--p) );  // return value ignored
    // documentation for data FROM the ADF435x is unavailable (to me) <- not supported herein.
  digitalWrite( static_cast<u8>(PIN::LE), 1 ); };
enum Symbol : char {
  // in datasheet order. Enumerant names do NOT mirror datasheet's names exactly
  r0 = 0,
      fraction,     integer,          // r0 has 3 symbols
  r1, modulus,      phase,
      prescaler,    phase_adjust,     // 5
  r2, counterReset, cp3state,
      idle,         pdPolarity,
      ldp,          ldf,
      cpIndex,      dblBfr,
      rCounter,     refDivider,
      refDoubler,   muxOut,
      LnLsModes,                      // 14
  r3, clkDivider,   clkDivMode,
      csr,          chrgCancel,
      abp,          bscMode,          // 7
  r4, rfOutPwr,     rfOutEnable,
      auxOutPwr,    auxOutEnable,
      auxOutSelect, muteTillLD,
      vcoPwrDown,   bandSelectClkDiv,
      rfDivSelect,  feedbackSelect,   // 11
  r5, res5,         led_mode,         // 3
  end                                 // sanity check computation aid only
  };  static constexpr size_t nSymbol{ Symbol::end };
using S = Symbol;  // ADF435x register layout <- Specification.
static constexpr struct { const u8 rank, offset, width; } Specification[] = {
  /* deduced via datasheet inspection (it's the only really clear part of the datasheet proper)
  // N:       number of (32 bit width) registers: 6
  // rank:    datasheetRegisterNumber = N - 1 - rank
  //          flush in ascending rank order, unless not dirty. hence, datasheet register '0' is
  //          always flushed last (and always needs flushing to cause effect on the pll).
  // offset:  position of field's least significant BIT within a 32 bit width register.
  // width:   yup. *//*
  {   r0   }, { integer }, { fraction },                                                        */
  {5,  0, 3}, {5,  3, 12}, {5, 15, 16}, /*                          // r0 has 3 'fields'
  {   r1   }, {  phase  }, { modulus },  et cetera                                              */
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
  {   r5   }, { resv'd  }, { LEDmode }                                                          */
  {0,  0, 3}, {0, 19,  2}, {0, 22,  2}                              // r5: 3
  };  // human tallied number of fields: 43                         // ad taedium. not ad nauseam.
  static_assert(nSymbol == (sizeof(Specification) / sizeof(Specification[0])));
  // KD9FWW (c) 2024
struct SpecifiableStorageAndSPIOutput {
  // not everything worth doing is worth over doing. Says ME. ArxContainer is just enough.
  struct Frame {
    static constexpr auto N{ 6 };       // no, this N won't collide with your N.
    using Buffer = std::array<u32, N>;
    // allocation and initialization ("reserved" & "contol" bit 'fields')
      // with the exception of r5 bits 19 and 20, all "reserved" bits must be set to zero. 
      // NB: this is my old school 'constructor'. don't use the set() method to overwrite these
      // note: Ein verboten fer a reason! behave yourself or no more Symbol::r# for you.
    u8 dirty; Buffer bfr; } frame = { 0, Frame::Buffer{ 0x180005, 4, 3, 2, 1, 0 } };
      // usage: object.set(symA,valA).set(symB,valB) ... ad infinitum
  auto set( S symbol, u16 value ) -> decltype(*this) {
    static constexpr u32 Mask[] = {
              0,    1,    3,    7,   15,   31,    63,   127,
      255,  511, 1023, 2047, 4095, 8191, 16383, 32767, 65535 };
    auto STp = &Specification[ static_cast<const char>( symbol ) ]; // twiddle dee, twiddle duh
    frame.bfr[ STp->rank ] &= ( ~(        Mask[ STp->width ]   << STp->offset) ); // all off, 1st
    frame.bfr[ STp->rank ] |= (  (value & Mask[ STp->width ] ) << STp->offset  ); // then, on.
    frame.dirty |= weight( (frame.N - 1) - STp->rank ); // encode which u32 we durty'd
    return *this; } // When it is appropriate to do so, flush() 'it' to the pll module.
      // The SSSO House Rules. All 2 (two) of them.
      // Rule 1). The (bit) train shall only depart with a trailing caboose. 'Tis the caboose
      // which triggers the motion of said train. (translation: flush() calls tx() which does SPI).
      // Rule 2). See: Rule 1).
  auto flush() -> void {  // NO, the type is auto. (It is NOT automatically triggered.)
    // to enable a flush to NOT be the last in a chain (but why?),
    //  use: auto flush() -> decltype(*this) { ... and return same instead.
    if( 0 == frame.dirty ) return;  // sophomoric. needs refinement; use aggregate feature of tx()
      // instead of multple, repetitive calls to tx() with a single u32
    for(auto ix = ( 1 < frame.dirty ) ? 0 : frame.N - 1; frame.N != ix; ++ix)
      tx( &frame.bfr[ix], sizeof(frame.bfr[ix]) );
    frame.dirty = 0; }  // it's not an issue, if only register 0 is dirty
};  using SSSO = SpecifiableStorageAndSPIOutput;
SSSO global;     // it's as it's name implies
  // And so are these, but they're not calculated on the nano. Avail one's self of the IDE's
  // (compiler) floating point. That is, compute ad435x register content as compile time constants.
  // Resources not needed at run time, don't consume [nano] memory. A noop sketch requires
  // {444, 9}[PGM,DATA] bytes. Addition of my code, as written herein, consumes: (IT'S WEE SMALL)
  /* {2404,258}[PGM,DATA] = {2848, 267} - {444, 9} bytes. <-- from the IDE: pasted as follows ...
  Sketch uses 2848 bytes (9%) of program storage space. Maximum is 30720 bytes.
  Global variables use 267 bytes (13%) of dynamic memory, leaving 1781 bytes for local ... */
  constexpr auto   fOSC{ 025e6 }, // manifest constants
                 minVCO{ 2.2e9 },
                 maxVCO{ 4.4e9 },
                 minPFD{ 125e3 },
                 maxPFD{ 050e6 };
constexpr enum SelectorSet { EVAL, CM23, WIDE, CM33, OOK, CM70, M2, M6 } Selector = WIDE;
  /*  ... and how shall I tell you?  And the King replied: "Start at the beginning. Proceed until
      the end. Then stop." Lewis Carroll. "Alice's Adventures in Wonderland". 1865.             */
constexpr struct { double fSet; int rfDtabIndex, offset; } control[] = { // tuning data
// start here. add your tuning member and 'Selector' it or modify one of these
    [EVAL] = { 2.5e9, 0, 0 }, // { frequency, divisor index, offset: see Fraction below }
    [CM23] = { 1.27e9, 1, 0 },
    [WIDE] = { 910.000e6, 2, 5 },
    [CM33] = { 902e6, 2, 0 },
    [OOK] = { 433.9200e6, 3, 2 }, // to get 430MHz, divide the vco by 8
    [CM70] = { 420E6, 3, 0 },
    [M2] = { 144e6, 4, 0 },   // untested
    [M6] = { 50e6, 6, 0 } };  // this output is fVCO divided by 64. see midX's decl below
  constexpr auto fStep{ (SelectorSet::EVAL == Selector) ? 100e3 : 12.5e3 / 2 };
  enum Enab { OFF = 0, ON };
  constexpr u8 rfDivisorTable[] = { 1, 2, 4, 8, 16, 32, 64 };
  constexpr auto RfDivisorTableIndex{ control[ Selector ].rfDtabIndex };  // todo: calc rfDivisor
      // that is, remove the need for the midde data member the control table entries
  constexpr auto rfDivisor = rfDivisorTable[ RfDivisorTableIndex ];       // use result of todo here
  //  FYI. fSet ranges
    constexpr auto mid0{ (maxVCO - minVCO) / 2 + minVCO };  // 3300 ± 1100 = { 4400, 2200 }
    constexpr auto mid1{ mid0 / rfDivisorTable[1] };  // 1650 ± 550 = { 2200, 1100 }
    constexpr auto mid2{ mid0 / rfDivisorTable[2] };  // 825 ± 275 = { 1100, 550 }
    constexpr auto mid3{ mid0 / rfDivisorTable[3] };  // 412.5 ± 137.5 = { 550, 275 }
    constexpr auto mid4{ mid0 / rfDivisorTable[4] };  // 206.25 ± 68.75 = { 275, 137.5 }
    constexpr auto mid5{ mid0 / rfDivisorTable[5] };  // 103.125 ± 34.375 = { 137.5, 68.75 }
    constexpr auto mid6{ mid0 / rfDivisorTable[6] };  // 51.5625 ± 17.1875 = { 68.75, 34.375 }
  constexpr auto fVCO = control[ Selector ].fSet * rfDivisor;
    static_assert((maxVCO >= fVCO) & (minVCO <= fVCO));
  constexpr auto oscDoubler{ Enab::OFF };
  constexpr auto oscToggler{ Enab::OFF };
  constexpr auto Rcounter{ 1 };
    // PFD: Phase Frequency Detector
  constexpr auto fPFD = fOSC * (1 + oscDoubler) / (1 + oscToggler) / Rcounter;
    static_assert((minPFD <= fPFD) & (maxPFD >= fPFD));
  constexpr auto BscClkDiv = u8( round(fPFD / 125e3) );
  constexpr enum FeedBack { divided = 0, fundamental } Fdbk = FeedBack::divided;
  constexpr auto fracN = ( (Fdbk) ? 1 : rfDivisor ) * control[ Selector ].fSet / fPFD;
  constexpr auto Whole = u16( fracN );
    static_assert( 22 < Whole );
  enum Prsc { four5ths = 0, eight9ths };
  constexpr auto preScale = (75 < Whole) ? Prsc::eight9ths : Prsc::four5ths;
    static_assert( ((preScale) ? 75 : 23) <= Whole);
  constexpr auto Modulus = round(fPFD / fStep);
    static_assert((1 < Modulus) & (4096 > Modulus));
      // todo: spur avoidance mechanism; use next lowest Modulus value NOT factorable by { 2, 3 }
  constexpr auto ModulusShort = u16( Modulus );
  constexpr auto frac = (fracN - Whole);  // there must be a better 'offset' method
    // fOSC is statisically always in error. use the ± offset member to 'trim' the output freq
  constexpr auto Fraction = u16( round( frac * ModulusShort ) ) + control[ Selector ].offset;
    // table lookup? gasp! no. fix it in one place: at fPFD <- todo
    static_assert(ModulusShort > Fraction);
    // yeah, it's a cute const too but stop staring
constexpr auto freq =
  (Whole + double(Fraction) / ModulusShort) * fPFD / ((Fdbk) ? 1 : rfDivisor);
    constexpr auto error = control[ Selector ].fSet - freq;
    constexpr auto absErr = (0 <= error) ? error : -error;
      static_assert( fStep > (absErr / 2) );
    // "And away we go!" -Gleason.
auto setup() -> void  {
  SPI.begin();
  pinMode(static_cast<u8>(PIN::PDR), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::PDR), (SelectorSet::OOK == Selector) ? 0 : 1 ); // 'key' off
    // for OnOffKeying (OOK) start with 'key' off.
  pinMode(static_cast<u8>(PIN::LE), OUTPUT);
  digitalWrite(static_cast<u8>(PIN::LE), 1);  // adf435x is rising edge: 1 then 0, followed by 1
  pinMode(static_cast<u8>(PIN::LD), INPUT);   // lock detect
  /* digitalWrite(static_cast<u8>(PIN::MUX), INPUT_PULLUP); */  { // begin another local scope
SSSO scratch;  // setup a (stack allocated) Specification configured SSSO
  // qty:37 unique calls of set() are required, in any order. Note: set() does NOT write directly
  // to the pll; flush() does that (automatically, in the correct order).
  scratch.set( S::fraction, Fraction );
  scratch.set( S::integer, Whole );
  scratch.set( S::modulus, ModulusShort );
  scratch.set( S::phase, 1);  // adjust phase AFTER loop lock
  scratch.set( S::phase_adjust, Enab::OFF );
  scratch.set( S::prescaler, preScale );
  scratch.set( S::counterReset, Enab::OFF );
  scratch.set( S::cp3state, Enab::OFF );
  scratch.set( S::idle, Enab::OFF );
  enum PDpolarity { negative = 0, positive };
  scratch.set( S::pdPolarity, PDpolarity::positive );
  enum LDPnS { ten = 0, six };
  scratch.set( S::ldp, LDPnS::ten );
  enum LDF { fracN = 0, intN };
  scratch.set( S::ldf, LDF::fracN );
  scratch.set( S::cpIndex, 7 );
  scratch.set( S::dblBfr, Enab::ON );   /*  Why can I see no reason not to double buffer?
    it's probably because I am lacking knowledge of some aspect of the pll. hmm. grumble.
    "There is no substitute for fundamental understanding." E. Neville Pickering. Bradley U. */
  scratch.set( S::rCounter, Rcounter );
  scratch.set( S::refDivider, oscToggler );
  scratch.set( S::refDoubler, oscDoubler );
  /* I didn't need the pll MUX out (for debug?). I have a hunch that pll internal states
    might be observable here <- output from the pll isn't available (to me). I wired it simply
    because the mfr wired it on their $200+ eval product. To use theirs, would require major
    modification to 'patch-in' a nano. They use some other µC with an MS Windows™ host (ONLY) 
    over USB <- I refuse to accept this abominable, rip-off, BUG RIDDEN, liability.
    It works for them - get the customers to find, fix, and PAY for it. Free stuff only for me. */
  constexpr enum MuxOut { HiZ = 0, DVdd, DGnd, rCntOut, nDivOut, aLock, dLock } Mux = HiZ;
  scratch.set( S::muxOut, Mux ); // Mux is a named enumerant of type MuxOut. NOT PIN::MUX.
  constexpr enum NoiseSpurMode { lowNoise = 0, lowSpur = 3 } nsMode = lowNoise;
    static_assert(( NoiseSpurMode::lowSpur == nsMode) ? (49 < ModulusShort ? 1 : 0) : 1 );
  scratch.set( S::LnLsModes, nsMode );
  constexpr auto ckDvdr = round( fOSC * 400e-6 / ModulusShort );
    static_assert( 4096 > ckDvdr );
  scratch.set( S::clkDivider, ckDvdr );
  constexpr enum ClockingMode { dividerOff = 0, fastLock, phResync } clockingMode = phResync;
  scratch.set( S::clkDivMode, clockingMode );
  scratch.set( S::csr, Enab::OFF );
  scratch.set( S::chrgCancel, Enab::OFF );
  enum ABPnS { nS6fracN = 0, nS3intN };
  scratch.set( S::abp, ABPnS::nS6fracN );
  scratch.set( S::bscMode, 0 );
  constexpr enum dBm { minus4, minus1, plus2, plus5 } rfPower = plus5;
  scratch.set( S::rfOutPwr, rfPower );
  scratch.set( S::rfOutEnable, Enab::ON );
  scratch.set( S::auxOutPwr, dBm::minus4 );
  scratch.set( S::auxOutEnable, Enab::OFF );
  scratch.set( S::auxOutSelect, FeedBack::divided );
  scratch.set( S::muteTillLD, Enab::ON );
  scratch.set( S::vcoPwrDown, Enab::OFF );
  scratch.set( S::bandSelectClkDiv, BscClkDiv );
  scratch.set( S::rfDivSelect, RfDivisorTableIndex );
  scratch.set( S::feedbackSelect, !Fdbk );  // Argh! why ( !Fdbk ) ??? I'm stumped. Tell me.
  constexpr enum LedMode { low = 0, lockDetect = 1, high = 3 } Led = lockDetect;
  scratch.set( S::led_mode, Led);      
global = scratch; } // save. end local scope.
  // variable and enumerant names declared therein hence, become unavailable. 'Mux' is one such
  // example where something is declared for the sake of clarity. Where an enumerant is
  // made availble, again for clarity, it is instantiated immediately prior to it's use.
global.flush(); /* "ship it!",
  heard in rm 6137 Northrop DSD. Rolling Meadows. circa 1980.
  along with "what's the answer?". same time, same station (mouth, that is).
  leghorn[Foghorn]: "I say, I, I, I say, It's a joke son, S'posed t' laugh."                    */
wait4lock();  // that pretty blue led (on the pll) indicates phase lock. now, set phase at 180º.
global.set( S::phase_adjust,Enab::ON ).set( S::phase,(ModulusShort >> 1) ).flush();
} // end of setup()
  /* jettson[George]: "Jane! JANE! Stop this crazy thing! JANE! !!!" */
auto loop() -> void {  }  //  73's, KD9FWW.
