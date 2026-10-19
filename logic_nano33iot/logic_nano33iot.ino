/*
 * logic_nano33iot.ino — 5-channel logic analyzer on an Arduino Nano 33 IoT
 *
 * Purpose: look at up to four DShot300 motor lines and one ESC telemetry
 * (UART) wire with sub-bit resolution, without a scope. Development aid, not a
 * calibrated instrument. GUI: logic_gui.py in the repository root.
 *
 * Board: Arduino Nano 33 IoT (SAMD21G18, Cortex-M0+, 48 MHz, 3.3 V logic,
 *        NOT 5 V tolerant — check the signal levels before you connect anything).
 *
 * How it samples: the five inputs all sit in byte 2 of PORTA (PA16..PA23), so
 * one 8-bit read of the PORT IN register captures every channel at once. A tight
 * unrolled loop copies that byte into RAM as fast as the core can (about
 * 10 MSa/s, ~100 ns per sample). TC4 (32-bit, 48 MHz) timestamps the start and
 * end of the loop, so the exact ns/sample is measured, not assumed, and sent
 * with every capture. D13 shares the byte but carries the on-board LED, which
 * would load a DShot line — it is masked out.
 *
 * Channels (Arduino pin -> PORTA bit, derived at runtime from the pin table):
 *   CH0 D8   (PA18)   suggested: motor 1 DShot
 *   CH1 D9   (PA20)   suggested: motor 2 DShot
 *   CH2 D10  (PA21)   suggested: motor 3 DShot
 *   CH3 D11  (PA16)   suggested: motor 4 DShot
 *   CH4 D12  (PA19)   suggested: ESC telemetry (pad T, KISS/UART 115200)
 *   GND -> common ground with the flight controller / ESC
 *
 * Self-test: 'G' toggles a hardware 300 kHz / 50 % PWM on D2 (TCC0, no CPU).
 * Jumper D2 -> D8 and capture: period must read 3.333 us, high 1.667 us. That
 * checks the sampling clock against a crystal-derived DShot300 bit period.
 *
 * Serial protocol (USB CDC, 115200 nominal), one command per line:
 *   a            arm + capture with the current settings, stream the result
 *   t<ch><edge>  trigger: ch 0..4, edge r|f|a|n|F|I  (rising, falling, any, none,
 *                F = frame start: rising edge after >= ~100 us of low,
 *                I = inverted frame start: falling edge after >= ~100 us of high,
 *                    for bidirectional DShot)  e.g. t0F
 *   l<n>         capture length in samples (16..BUF_MAX, rounded to 16)
 *   d<n>         rate divider: 0 = fastest; n adds n NOPs per sample (slower, longer window)
 *   w<ms>        trigger timeout in ms (default 2000)
 *   G            toggle self-test PWM on D2
 *   p            toggle the internal pull-up on D12 / CH4 (default ON: the ESC's pad T idles low)
 *   s            pin activity scan D2..D12 (wiring diagnosis)
 *   i            print settings + channel map
 *   h            help
 *
 * Capture output:
 *   #CAP v=1 n=<samples> ns=<ns per sample> div=<d> trig=<ch><edge> trigok=<0|1> mask=<hex> bits=<b0,b1,b2,b3,b4> names=D8,D9,D10,D11,D12
 *   <value>,<run length>       (run-length encoded, value = masked PORTA byte 2)
 *   ...
 *   #END
 */

#include <Arduino.h>
#include "wiring_private.h"

#define N_CH      5
#define BUF_MAX   20000u          /* bytes; the Nano has 32 KB RAM, the core needs the rest */

static const uint8_t chPin[N_CH] = { 8, 9, 10, 11, 12 };
static uint8_t  chBit[N_CH];       /* bit inside PORTA byte 2, derived from g_APinDescription */
static uint8_t  chMask = 0;

static uint8_t  buf[BUF_MAX] __attribute__((aligned(4)));

static uint32_t capLen     = BUF_MAX;
static uint32_t rateDiv    = 0;
static uint8_t  trigCh     = 0;
static char     trigEdge   = 'r';
static uint32_t trigTimeoutMs = 2000;
static bool     selfTestPwm = false;
static bool     tlmPullup   = true;    /* D12 (CH4, ESC pad T) idle-high via the internal pull-up */

/* ---------------- TC4: 32-bit free-running counter at 48 MHz ---------------- */
static void timerInit()
{
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5;
  while (GCLK->STATUS.bit.SYNCBUSY) { }
  PM->APBCMASK.reg |= PM_APBCMASK_TC4 | PM_APBCMASK_TC5;

  TC4->COUNT32.CTRLA.bit.ENABLE = 0;  while (TC4->COUNT32.STATUS.bit.SYNCBUSY) { }
  TC4->COUNT32.CTRLA.bit.SWRST  = 1;  while (TC4->COUNT32.CTRLA.bit.SWRST) { }
  TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_WAVEGEN_NFRQ;
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY) { }
  TC4->COUNT32.CTRLA.bit.ENABLE = 1;  while (TC4->COUNT32.STATUS.bit.SYNCBUSY) { }
}

static inline uint32_t timerNow()
{
  TC4->COUNT32.READREQ.reg = TC_READREQ_RREQ | TC_READREQ_ADDR(0x10);   /* COUNT register */
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY) { }
  return TC4->COUNT32.COUNT.reg;
}

/* ---------------- self-test PWM on D2 (TCC0/WO[4], PB10) ---------------- */
static void selfTestInit()
{
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1;
  while (GCLK->STATUS.bit.SYNCBUSY) { }
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0;

  TCC0->CTRLA.bit.ENABLE = 0;  while (TCC0->SYNCBUSY.bit.ENABLE) { }
  TCC0->CTRLA.bit.SWRST  = 1;  while (TCC0->SYNCBUSY.bit.SWRST || TCC0->CTRLA.bit.SWRST) { }
  TCC0->CTRLA.reg  = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_PRESCSYNC_GCLK;
  TCC0->WAVE.reg   = TCC_WAVE_WAVEGEN_NPWM;      while (TCC0->SYNCBUSY.bit.WAVE) { }
  TCC0->PER.reg    = 160 - 1;                    while (TCC0->SYNCBUSY.bit.PER)  { }   /* 3.333 us */
  TCC0->CC[0].reg  = 80;                         while (TCC0->SYNCBUSY.bit.CC0)  { }   /* 1.667 us high */
  TCC0->CTRLA.bit.ENABLE = 1;  while (TCC0->SYNCBUSY.bit.ENABLE) { }
}

static void selfTestSet(bool on)
{
  selfTestPwm = on;
  if (on) {
    pinPeripheral(2, PIO_TIMER_ALT);     /* PB10 -> TCC0/WO[4] */
  } else {
    pinMode(2, INPUT);                   /* release D2 */
  }
}

/* ---------------- capture loops ---------------- */
/* Fastest: 16x unrolled ldrb/strb. n must be a multiple of 16. */
static void __attribute__((noinline, optimize("O2")))
captureFast(volatile uint8_t *in, uint8_t *dst, uint32_t n)
{
  uint8_t *end = dst + n;
  do {
    dst[0]  = *in; dst[1]  = *in; dst[2]  = *in; dst[3]  = *in;
    dst[4]  = *in; dst[5]  = *in; dst[6]  = *in; dst[7]  = *in;
    dst[8]  = *in; dst[9]  = *in; dst[10] = *in; dst[11] = *in;
    dst[12] = *in; dst[13] = *in; dst[14] = *in; dst[15] = *in;
    dst += 16;
  } while (dst < end);
}

/* Slower: one sample then `div` NOPs. Exact rate comes from the TC4 timestamps. */
static void __attribute__((noinline, optimize("O2")))
captureDiv(volatile uint8_t *in, uint8_t *dst, uint32_t n, uint32_t div)
{
  uint8_t *end = dst + n;
  do {
    *dst++ = *in;
    for (uint32_t d = div; d != 0; d--) { __asm__ __volatile__("nop"); }
  } while (dst < end);
}

/* Spin until the selected edge shows up on the trigger channel. Interrupts are
 * already off. Returns false on timeout. */
static bool waitTrigger(volatile uint8_t *in)
{
  if (trigEdge == 'n') return true;
  const uint8_t bit = chBit[trigCh];
  const uint32_t t0 = timerNow();
  const uint32_t limit = trigTimeoutMs * 48000UL;
  uint8_t prev = (*in >> bit) & 1u;
  uint32_t lowRun = 0;                 /* consecutive polls with the line low (~0.2 us each) */
  uint32_t highRun = 0;                /* same, line high (inverted frame start) */
  const uint32_t frameGap = 500;       /* ~100 us: longer than any gap inside a DShot300 frame */
  for (;;) {
    for (uint32_t k = 0; k < 4096; k++) {
      const uint8_t cur = (*in >> bit) & 1u;
      if (cur != prev) {
        if (trigEdge == 'a' || (trigEdge == 'r' && cur) || (trigEdge == 'f' && !cur)) return true;
        if (trigEdge == 'F' && cur && lowRun >= frameGap) return true;
        if (trigEdge == 'I' && !cur && highRun >= frameGap) return true;
        prev = cur;
      }
      lowRun  = cur ? 0 : lowRun + 1;
      highRun = cur ? highRun + 1 : 0;
    }
    if ((uint32_t)(timerNow() - t0) > limit) return false;
  }
}

static void capture()
{
  volatile uint8_t *in = ((volatile uint8_t *)&PORT_IOBUS->Group[PORTA].IN.reg) + 2;
  const uint32_t n = capLen;

  __disable_irq();
  const bool trigOk = waitTrigger(in);
  const uint32_t t0 = timerNow();
  if (rateDiv == 0) captureFast(in, buf, n);
  else              captureDiv(in, buf, n, rateDiv);
  const uint32_t t1 = timerNow();
  __enable_irq();

  const float nsPerSample = (float)(t1 - t0) * (1000.0f / 48.0f) / (float)n;

  Serial.print(F("#CAP v=1 n=")); Serial.print(n);
  Serial.print(F(" ns=")); Serial.print(nsPerSample, 3);
  Serial.print(F(" div=")); Serial.print(rateDiv);
  Serial.print(F(" trig=")); Serial.print(trigCh); Serial.print(trigEdge);
  Serial.print(F(" trigok=")); Serial.print(trigOk ? 1 : 0);
  Serial.print(F(" mask=0x")); Serial.print(chMask, HEX);
  Serial.print(F(" bits="));
  for (uint8_t c = 0; c < N_CH; c++) { Serial.print(chBit[c]); if (c < N_CH - 1) Serial.print(','); }
  Serial.println(F(" names=D8,D9,D10,D11,D12"));

  /* run-length encode the masked samples */
  uint8_t  v   = buf[0] & chMask;
  uint32_t len = 1;
  for (uint32_t i = 1; i < n; i++) {
    const uint8_t s = buf[i] & chMask;
    if (s == v) { len++; }
    else {
      Serial.print(v); Serial.print(','); Serial.println(len);
      v = s; len = 1;
    }
  }
  Serial.print(v); Serial.print(','); Serial.println(len);
  Serial.println(F("#END"));
}


/* Pin activity scan: 2000 digitalRead()s on D2..D12, prints how many read high.
 * A PWM line shows an intermediate count, a floating pin a random one, a stuck
 * or unconnected-but-pulled pin 0 or 2000. Wiring diagnosis only. */
static void pinScan()
{
  Serial.print(F("#SCAN"));
  for (uint8_t pin = 2; pin <= 12; pin++) {
    if (pin == 2 && selfTestPwm) { Serial.print(F(" D2=pwm-out")); continue; }
    const bool wasCh = (pin >= 8);
    if (!wasCh) pinMode(pin, INPUT);
    uint16_t hi = 0;
    for (uint16_t k = 0; k < 2000; k++) { if (digitalRead(pin)) hi++; }
    Serial.print(F(" D")); Serial.print(pin); Serial.print('='); Serial.print(hi);
  }
  Serial.println(F(" (of 2000 high)"));
}

/* ---------------- console ---------------- */
static void printInfo()
{
  Serial.print(F("#INFO len=")); Serial.print(capLen);
  Serial.print(F(" div=")); Serial.print(rateDiv);
  Serial.print(F(" trig=")); Serial.print(trigCh); Serial.print(trigEdge);
  Serial.print(F(" timeout_ms=")); Serial.print(trigTimeoutMs);
  Serial.print(F(" selftest=")); Serial.print(selfTestPwm ? 1 : 0);
  Serial.print(F(" tlm_pullup=")); Serial.print(tlmPullup ? 1 : 0);
  Serial.print(F(" mask=0x")); Serial.print(chMask, HEX);
  Serial.print(F(" map="));
  for (uint8_t c = 0; c < N_CH; c++) {
    Serial.print(F("CH")); Serial.print(c); Serial.print(F(":D")); Serial.print(chPin[c]);
    Serial.print(F("=PA")); Serial.print(16 + chBit[c]);
    if (c < N_CH - 1) Serial.print(' ');
  }
  Serial.println();
}

static void help()
{
  Serial.println(F("#HELP a=capture  t<ch><r|f|a|n|F|I>=trigger (F=frame start, I=inverted frame start)  l<n>=length  d<n>=divider  w<ms>=timeout  G=selftest PWM D2  p=CH4 pull-up  s=pin scan  i=info"));
}

static void handleLine(char *line)
{
  switch (line[0]) {
    case 'a': capture(); break;
    case 't': {
      const uint8_t ch = (uint8_t)(line[1] - '0');
      const char e = line[2];
      if (ch < N_CH && (e == 'r' || e == 'f' || e == 'a' || e == 'n' || e == 'F' || e == 'I')) { trigCh = ch; trigEdge = e; }
      printInfo();
      break;
    }
    case 'l': {
      uint32_t n = (uint32_t)atol(line + 1);
      n = (n / 16u) * 16u;
      if (n < 16u) n = 16u;
      if (n > BUF_MAX) n = BUF_MAX;
      capLen = n;
      printInfo();
      break;
    }
    case 'd': rateDiv = (uint32_t)atol(line + 1); printInfo(); break;
    case 'w': trigTimeoutMs = (uint32_t)atol(line + 1); printInfo(); break;
    case 'G': selfTestSet(!selfTestPwm); printInfo(); break;
    case 'p': tlmPullup = !tlmPullup; pinMode(chPin[4], tlmPullup ? INPUT_PULLUP : INPUT); printInfo(); break;
    case 'i': printInfo(); break;
    case 's': pinScan(); break;
    case 'h': help(); break;
    default: break;
  }
}

void setup()
{
  /* derive the PORTA byte-2 bit of every channel from the core's pin table */
  chMask = 0;
  for (uint8_t c = 0; c < N_CH; c++) {
    const PinDescription &pd = g_APinDescription[chPin[c]];
    chBit[c] = (uint8_t)(pd.ulPin - 16u);          /* PA16..PA23 -> bit 0..7 */
    chMask |= (uint8_t)(1u << chBit[c]);
    pinMode(chPin[c], INPUT);                       /* INEN, no pull: the DShot lines carry their own pull-up */
  }
  pinMode(chPin[4], tlmPullup ? INPUT_PULLUP : INPUT); /* pad T idles low when the ESC is not driving it */
  /* continuous input sampling for PA16..PA23 so IN is always current */
  PORT->Group[PORTA].CTRL.reg |= ((uint32_t)chMask) << 16;

  timerInit();
  selfTestInit();
  selfTestSet(false);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }
  help();
  printInfo();
}

void loop()
{
  static char line[16];
  static uint8_t pos = 0;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (pos > 0) { line[pos] = 0; handleLine(line); pos = 0; }
    } else if (pos < sizeof(line) - 1) {
      line[pos++] = c;
    }
  }
}
