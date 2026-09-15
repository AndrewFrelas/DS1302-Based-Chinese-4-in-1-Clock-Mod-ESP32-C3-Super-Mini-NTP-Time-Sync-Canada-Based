// NtpClockMod - Andrew Frelas
// NTP-accurate time on the Temu 4-in-1 clock (YM-SZ010-L board) by answering the stock
// MCU's DS1302 reads. Single-file Arduino sketch. Reference: project doc ntp-clock-mod.md
//
// Target : ESP32-C3, Arduino IDE, Espressif "esp32" core 3.3.x, no external libraries
// Board  : "ESP32C3 Dev Module" (or the entry for your C3 board)
//          Tools > USB CDC On Boot: Enabled   (serial log over the C3's native USB)

// ============================================================================ CONFIGURATION

#define FW_VERSION              "01.03.08"

// ---- Wi-Fi (hardcoded; everything else is settable in the web UI and stored in NVS)
#define WIFI_SSID               "your-ssid"
#define WIFI_PASSWORD           "your-password"
#define DEVICE_HOSTNAME         "ntpclock"          // web UI: http://ntpclock.local/

// Reconnect policy: the device retries for as long as it is powered, forever.
#define WIFI_RETRY_MIN_MS       5000UL              // first retry delay after a drop
#define WIFI_RETRY_MAX_MS       60000UL             // retry delay ceiling
#define WIFI_RADIO_RESET_TRIES  12                  // failed tries before the radio is restarted

// Transmit power. Every converging report on the C3 SuperMini running hot points at the radio,
// not the core: a no-WiFi build stays cool, cutting TX power fixed it outright for one user,
// and a thermal camera showed 60 C -> 40 C from allowing modem sleep. Underclocking the CPU
// was tried once and changed nothing.
//
// esp_wifi_set_max_tx_power takes 0.25 dBm units, range [8, 84] = 2 to 20 dBm, and QUANTISES:
// {set, actual} = {8,8} {20,20} {28,28} {34,34} {44,44} {52,52} {56,56} {60,60} {66,66}
// {72,72} {80,80}, i.e. 2, 5, 7, 8, 11, 13, 14, 15, 16, 18, 20 dBm. Anything between rounds
// DOWN to the next step. The Arduino enum lies twice: WIFI_POWER_21dBm actually yields 20, and
// WIFI_POWER_MINUS_1dBm (-4) is outside the legal range and is refused. Neither is offered.
//
// Default 11 dBm: a 9 dB cut from the 20 dBm default, about an eighth of the transmit power.
// TX power cannot affect DS1302 bus timing, so this is the risk-free half of the heat work.
// Must be re-applied after every STA start - a radio reset puts it back to the default.
#define WIFI_TX_DBM_DEFAULT     11

// Modem sleep. This is the bigger lever by the one measurement anyone has taken, but it is
// also the one that could touch bus timing, so it stays off by default. WIFI_PS_MIN_MODEM
// parks the radio between DTIM beacons and leaves the CPU running, so the IRAM ISR should be
// unaffected - watch late_sclk_edge, over_budget and timeouts in Bus diagnostics to confirm
// rather than assuming. 0 = none, 1 = min modem, 2 = max modem.
#define WIFI_PS_MODE_DEFAULT    0

// ---- Optional web UI login. Empty user = no authentication (LAN only).
#define WEB_AUTH_USER           ""
#define WEB_AUTH_PASS           ""

// ---- Time defaults (first boot only; all four are editable in the web UI)
// One NTP server. No fallback servers are configured and DHCP-supplied NTP servers are
// rejected. Public pool servers: the NTP Pool asks for no more than 4-5 queries per hour,
// so use a local NTP server (router, NAS, pfSense) if you want minute-level polling.
#define DEFAULT_NTP_SERVER      "time.chu.nrc.ca"
#define DEFAULT_TZ              "EST5EDT,M3.2.0,M11.1.0"   // POSIX rule, America/Toronto
#define DEFAULT_SYNC_INTERVAL_S 60                  // NTP poll interval, seconds
#define DEFAULT_DST_MODE        0                   // 0 = follow zone rule, 1 = standard only, 2 = daylight only
#define DEFAULT_HOUR_FORMAT     0                   // 0 = follow the MCU, 1 = 24 hour, 2 = 12 hour
#define DEFAULT_SPOOF_ENABLED   false               // off until the passive bus check passes
#define DEFAULT_BOOT_COUNTDOWN  true                // show 00:00, 03:00, 02:00, 01:00 on first arm

// Poll interval limits. 15 s is the floor enforced by SNTP (RFC 4330).
#define NTP_MIN_INTERVAL_S      15UL
#define NTP_MAX_INTERVAL_S      86400UL

// How long the ESP32 keeps answering from its own clock after the last successful sync.
// Past this the bus goes high-Z and the real DS1302 answers. 0 = stop as soon as Wi-Fi drops.
#define NTP_HOLDOVER_HOURS      24

// Boot countdown step length, milliseconds (stretched to the MCU poll interval if slower).
#define BOOT_COUNTDOWN_STEP_MS  1000UL

// ---- DS1302 tap (doc section 5)
#define PIN_CE                  3   // DS1302 pin 5 - input only, rising-edge interrupt
#define PIN_SCLK                4   // DS1302 pin 7 - input only
#define PIN_IO                  5   // DS1302 pin 6 - high-Z except the data phase of a spoofed read

// ---- Bus engine
#define IO_DRIVE_STRENGTH       3   // PIN_IO drive capability 0..3 (3 = strongest: 40 mA source)
#define BUS_INTR_LEVEL          3   // CE ISR interrupt level 1..3 (3 = lowest entry latency)

// Consecutive samples of CE low required to believe the transfer has ended. The poll loop
// runs every few hundred nanoseconds, so this is well under a microsecond - far shorter than
// a genuine end of transfer, which holds CE low until the next poll cycle hundreds of
// milliseconds later. It therefore cannot miss a real one.
//
// Why it exists: a SINGLE noisy low sample used to end a read in flight. The ISR then
// released the line and every remaining bit clocked back as 0. Zeros are valid BCD, so the
// MCU latched a plausible wrong time rather than obvious garbage. Hours 23 (0x23,
// 0b00100011) cut after five bits reads 0b00000011 = 0x03, which is the "display randomly
// shows 03" report; the same cut at 20-22h gives 00, 01, 02, which is the "stuck on 00:00"
// report. Below 20:00 the high bits are already zero so the fault is invisible - which is why
// it only ever got noticed late in the evening.
//
// This board is a wireless charger and the tap is a flying wire, so CE glitches are expected;
// the ISR entry path already flagged TF_CE_GLITCH for them. Mid-transfer had no such guard.
// Set 1 to restore the old single-sample behaviour.
#define BUS_CE_LOW_CONFIRM      3

// ---- DS1302 physically removed: full emulation
// 1 = the DS1302 is OUT and the ESP32 is the only thing that can answer the MCU. It then
// answers EVERY read, including before the first NTP sync, because an unanswered read
// leaves the MCU staring at a floating line. Clock registers come from NTP time (a fixed
// sane BCD image until the first sync), the control register reads back 80h, and RAM /
// trickle-charger reads are answered with zeros rather than left floating.
// 0 = the chip is fitted; behave as before and leave unspoofable reads to the real chip.
#define RTC_CHIP_REMOVED        1

// I/O drive style. 0 = push-pull (drives both levels at 3.3 V).
// 1 = open-drain: pull low only, let an external pull-up make the high. Use this with a
// 10k pull-up from the I/O line to 5 V if the MCU will not accept a 3.3 V logic high.
#define IO_OPEN_DRAIN           0

// First data bit of a read. 1 = present bit 0 as soon as the command byte completes (on the
// 8th rising edge) instead of waiting for the following falling edge. Some MCUs sample the
// first bit during the clock that ends the command, and with the datasheet timing they
// latch the command's own last bit and read every byte shifted one place left.
// Symptom of needing this: values come back as (real << 1) | 1.
#define IO_FIRST_BIT_EARLY      1

// ---- Outdoor temperature output (replaces the removed NTC bead)
// Topology measured on this board: 5V --[NTC]-- node --[~1k]-- GND, node at 2.0 V @ 21 C.
// With the bead removed the 1k holds the node at 0 V, so nothing can push 5 V back into this
// pin and no series resistor is needed. Drive the pad that reads 0 V with the bead out -
// NOT the pad that reads 5 V, which is the supply rail.
// Lower duty = lower node voltage = COLDER on the display.
#define TEMP_OUT_ENABLE         1       // 0 = never touch the pin
#define TEMP_OUT_PIN            7       // dead quiet in the GPIO scan; CE/SCLK/IO stay on 3/4/5

// PWM frequency ceiling. LEDC needs 2^bits timer ticks per period, so the ceiling is
// source_clock / 2^bits. The source is NOT the 80 MHz APB the ESP-IDF docs describe: the
// Arduino core sets LEDC_DEFAULT_CLK to LEDC_USE_XTAL_CLK on every chip with
// SOC_LEDC_SUPPORT_XTAL_CLOCK, which the C3 has, so it is the 40 MHz crystal. Real ceilings
// are 39.06 kHz at 10 bits, 78.125 kHz at 9, 156.25 kHz at 8. Above the ceiling
// ledc_timer_config() fails, ledcAttach() returns false, and the pin is never driven at all.
//
// 9 bits at 76 kHz: 0.49 C of ripple with 220 ohm and 2.2 uF, and 511 steps is still
// 0.13 C per step across the whole 0-40 C span. TEMP_RAW_MAX follows TEMP_PWM_BITS, but the
// calibration table is in raw units and must be re-measured if either changes.
#define TEMP_PWM_FREQ_HZ        76000
#define TEMP_PWM_BITS           9
#define TEMP_RAW_MAX            ((1 << TEMP_PWM_BITS) - 1)
#define TEMP_SWEEP_STEP_MS      15000UL   // dwell per step in the raw ripple sweep

// Tried in order until one attaches, so a bad frequency constant can never leave the output
// silently dead again. The achieved frequency is reported on serial and in the web UI.
#define TEMP_PWM_FALLBACKS      60000, 50000, 40000, 25000, 10000

// The RC that turns the PWM into a DC level, and the board's own lower divider leg. 220 ohm
// is the largest value that still reaches 40 C (full duty must clear 2.935 V) while keeping
// ripple at 0.5 C with 2.2 uF at 78 kHz. 470 ohm is quieter but caps the display at 37 C;
// 100 ohm reaches 40 C but leaves 1.05 C of ripple. These only affect the reported node
// voltage - change them to match what is actually fitted.
#define TEMP_SERIES_OHMS        220L
#define TEMP_NODE_LOWER_OHMS    2700L
#define TEMP_NODE_MV_MAX        (3300L * TEMP_NODE_LOWER_OHMS \
                                 / (TEMP_SERIES_OHMS + TEMP_NODE_LOWER_OHMS))
#define TEMP_HOLD_MS            180000UL  // "show 20 C" hold length, then it reverts
#define TEMP_MARCH_STEP_MS      20000UL   // default dwell; 12 s let a settling value be misread
#define TEMP_DWELL_MIN_S        5UL       // /api/temp dwell= bounds, in seconds
#define TEMP_DWELL_MAX_S        120UL
#define TEMP_HOLD_MIN_C         -50.0f    // accepted range for a requested temperature
#define TEMP_HOLD_MAX_C         60.0f

// Sub-zero encoding. The node cannot go below 0 V and 0 V reads 0 C, so the display has no
// way to show a negative number. Below zero the magnitude is sent instead: -8 C is driven as
// 8 C. With TEMP_NEG_BLINK the output alternates between that magnitude and 0 C, so a steady
// 8 means +8 and an 8 that drops to 0 and back means -8. Set to 0 for plain magnitude.
// The MCU's own ADC sampling rate is unknown: if the blink reads as an average instead of
// two distinct numbers, raise both periods until it steps cleanly.
#define TEMP_NEG_BLINK          1
#define TEMP_NEG_HIGH_MS        4000UL    // time showing the magnitude
#define TEMP_NEG_LOW_MS         2000UL    // time showing 0 C

// ---- Outdoor temperature feed (Environment and Climate Change Canada, SWOB-ML)
// Static per-station URL, no API key. CWWB = Burlington Pier (WMO 71437), the station ECCC's
// own Oakville page sources its current conditions from. Hourly, published about a minute past.
// Parsing: the literal "air_temp" WITH both quotes is unique in the document - the bare string
// also appears in avg_air_temp_pst1hr, max_air_temp_pst24hrs and six more. Same for "date_tm".
// Plain HTTP is UNTESTED against this host (could not be reached from the build environment).
// Leave WX_USE_HTTPS at 1 unless http:// is confirmed to return 200, not a redirect.
// TLS here is unauthenticated (no cert store on the device). The payload is public weather
// data and the worst case of a tampered reply is a wrong number on a clock face.
#define WX_ENABLE               1
#define WX_USE_HTTPS            1
#define WX_HOST                 "dd.weather.gc.ca"
#define WX_PATH_PREFIX          "/today/observations/swob-ml/latest/"
#define WX_PATH_SUFFIX          "-AUTO-swob.xml"
#define WX_STATION_DEFAULT      "CWWB"
#define WX_FETCH_INTERVAL_S     600UL     // observation is hourly; this just recovers misses
#define WX_RETRY_S              120UL     // after a failed attempt
#define WX_MAX_AGE_S            7200UL    // reject a reading whose date_tm is older than this
#define WX_TIMEOUT_MS           12000
#define WX_MAX_BYTES            32768UL   // hard cap on bytes scanned from one response
#define WX_MIN_LARGEST_BLOCK    50000UL   // skip the fetch below this; TLS needs contiguous heap

// Longest time the ISR busy-waits on one transfer. The length is projected from the
// command-byte clock rate; transfers projected past this are not answered or observed.
// Must stay far below the 300 ms interrupt watchdog.
#define BUS_TXN_BUDGET_US       40000UL

// Abort a transfer if CE stays high with no SCLK rising edge for this long.
#define BUS_EDGE_TIMEOUT_US     20000UL

// Bit-slip guard. SCLK's GPIO edge-status latch is cleared when each transfer ends; if it
// is already set when the CE ISR starts, SCLK rose before the ISR could watch it, so that
// transfer is observed but never answered. Set 0 only if SCLK is shared with another
// device that clocks it while CE is low (the passive check reports this).
#define BUS_SLIP_GUARD          1

// CE rises closer together than this count as one polling cycle.
#define POLL_GROUP_GAP_US       50000UL

// ============================================================================ END CONFIGURATION

#include <Arduino.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sys/time.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_sntp.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/gpio_reg.h"

// ================================================================ PURE LOGIC BEGIN
// Hardware-independent code. test/host_test.cpp extracts this region and runs it on a PC
// against a simulated MCU and DS1302.

#define DS_INLINE inline __attribute__((always_inline))

namespace ds1302 {

// Register indexes in the clock image (= burst order).
enum : uint8_t { REG_SEC = 0, REG_MIN, REG_HOUR, REG_DATE, REG_MONTH, REG_DAY, REG_YEAR, REG_CTRL };

// ---- Command byte (shifted LSB first): bit7 = 1, bit6 RAM/clock, bits5..1 address, bit0 read
DS_INLINE bool    cmdValid(uint8_t c)  { return (c & 0x80) != 0; }
DS_INLINE bool    cmdIsRead(uint8_t c) { return (c & 0x01) != 0; }
DS_INLINE bool    cmdIsRam(uint8_t c)  { return (c & 0x40) != 0; }
DS_INLINE uint8_t cmdAddr(uint8_t c)   { return (uint8_t)((c >> 1) & 0x1F); }
DS_INLINE bool    cmdIsBurst(uint8_t c){ return cmdAddr(c) == 0x1F; }

// Data bytes that follow a command: 1, or 8 for clock burst, 31 for RAM burst.
DS_INLINE uint8_t cmdDataBytes(uint8_t c) {
  if (!cmdIsBurst(c)) return 1;
  return cmdIsRam(c) ? 31 : 8;
}

// Reads the ESP32 answers: 81h..8Dh (seconds..year) and BFh (clock burst).
// 8Fh control, 91h trickle charger and all RAM reads stay with the real chip.
DS_INLINE bool cmdIsSpoofable(uint8_t c) {
  if ((c & 0xC1) != 0x81) return false;          // valid + clock space + read
  const uint8_t a = cmdAddr(c);
  return a <= REG_YEAR || a == 0x1F;
}

// ---- BCD
DS_INLINE uint8_t toBcd(uint8_t v)   { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
DS_INLINE uint8_t fromBcd(uint8_t b) { return (uint8_t)(((b >> 4) * 10) + (b & 0x0F)); }
DS_INLINE bool    isBcd(uint8_t b)   { return (b & 0x0F) <= 9 && (b >> 4) <= 9; }

// ---- Hours register. bit7: 1 = 12 h mode (bit5 = PM, bits4..0 = 1..12), 0 = 24 h (bits5..0).
enum class HourMode : uint8_t { H24 = 0, H12 = 1 };

inline uint8_t encodeHour(int hour24, HourMode mode) {
  if (mode == HourMode::H24) return toBcd((uint8_t)hour24);
  int h12 = hour24 % 12;
  if (h12 == 0) h12 = 12;
  return (uint8_t)(0x80 | (hour24 >= 12 ? 0x20 : 0x00) | toBcd((uint8_t)h12));
}

// Returns 0..23, or -1 if the byte is not a legal hours value. (Called from the ISR.)
DS_INLINE int decodeHour(uint8_t reg) {
  if (reg & 0x80) {
    const uint8_t v = reg & 0x1F;
    if (!isBcd(v)) return -1;
    const int h = fromBcd(v);
    if (h < 1 || h > 12) return -1;
    return (h % 12) + ((reg & 0x20) ? 12 : 0);
  }
  const uint8_t v = reg & 0x3F;
  if (!isBcd(v)) return -1;
  const int h = fromBcd(v);
  return h <= 23 ? h : -1;
}

// ---- Clock image from broken-down local time. Control byte is left 0; the engine
// never drives it (burst byte 7 is released so the real chip's WP bit is read).
inline void buildImage(const std::tm& t, HourMode mode, uint8_t reg[8]) {
  reg[REG_SEC]   = toBcd((uint8_t)(t.tm_sec > 59 ? 59 : t.tm_sec));   // CH = 0; leap second clamps
  reg[REG_MIN]   = toBcd((uint8_t)t.tm_min);
  reg[REG_HOUR]  = encodeHour(t.tm_hour, mode);
  reg[REG_DATE]  = toBcd((uint8_t)t.tm_mday);
  reg[REG_MONTH] = toBcd((uint8_t)(t.tm_mon + 1));
  reg[REG_DAY]   = (uint8_t)(t.tm_wday + 1);                            // 1..7, Sunday = 1
  reg[REG_YEAR]  = toBcd((uint8_t)(t.tm_year % 100));
  reg[REG_CTRL]  = 0x00;
}

// True if the seven clock bytes are plausible BCD time.
inline bool imageLooksValid(const uint8_t reg[7]) {
  const uint8_t sec = reg[REG_SEC] & 0x7F;
  if (!isBcd(sec) || fromBcd(sec) > 59) return false;
  if (!isBcd(reg[REG_MIN]) || fromBcd(reg[REG_MIN]) > 59) return false;
  if (decodeHour(reg[REG_HOUR]) < 0) return false;
  if (!isBcd(reg[REG_DATE]) || fromBcd(reg[REG_DATE]) < 1 || fromBcd(reg[REG_DATE]) > 31) return false;
  if (!isBcd(reg[REG_MONTH]) || fromBcd(reg[REG_MONTH]) < 1 || fromBcd(reg[REG_MONTH]) > 12) return false;
  if (reg[REG_DAY] < 1 || reg[REG_DAY] > 7) return false;
  return isBcd(reg[REG_YEAR]);
}

// "13:05:09 2026-09-11 d6 24h" (+ " CH" if the clock-halt bit is set)
inline void formatImage(const uint8_t reg[7], char* buf, size_t n) {
  const int hour = decodeHour(reg[REG_HOUR]);
  std::snprintf(buf, n, "%02d:%02X:%02X 20%02X-%02X-%02X d%u %s%s",
                hour, reg[REG_MIN], reg[REG_SEC] & 0x7F, reg[REG_YEAR], reg[REG_MONTH],
                reg[REG_DATE], reg[REG_DAY], (reg[REG_HOUR] & 0x80) ? "12h" : "24h",
                (reg[REG_SEC] & 0x80) ? " CH" : "");
}

// "0xBF read clock burst", "0x8E write control", "0xC3 read RAM 1", "0x40 invalid (bit7=0)"
inline void describeCmd(uint8_t c, char* buf, size_t n) {
  if (!cmdValid(c)) { std::snprintf(buf, n, "0x%02X invalid (bit7=0)", c); return; }
  const char* rw = cmdIsRead(c) ? "read" : "write";
  const uint8_t a = cmdAddr(c);
  if (cmdIsRam(c)) {
    if (a == 0x1F) std::snprintf(buf, n, "0x%02X %s RAM burst", c, rw);
    else           std::snprintf(buf, n, "0x%02X %s RAM %u", c, rw, a);
    return;
  }
  static const char* const names[] = { "seconds", "minutes", "hours", "date", "month",
                                       "day", "year", "control", "trickle" };
  if (a == 0x1F)      std::snprintf(buf, n, "0x%02X %s clock burst", c, rw);
  else if (a <= 8)    std::snprintf(buf, n, "0x%02X %s %s", c, rw, names[a]);
  else                std::snprintf(buf, n, "0x%02X %s clock reg %u (undefined)", c, rw, a);
}

}  // namespace ds1302

namespace ds1302 {

enum : uint16_t {
  TF_CE_GLITCH  = 1u << 0,   // CE already low when the ISR ran
  TF_MISALIGNED = 1u << 1,   // SCLK high at entry: ISR ran late (or MCU broke tCC)
  TF_CMD        = 1u << 2,   // all 8 command bits received
  TF_INVALID    = 1u << 3,   // command bit7 = 0
  TF_READ       = 1u << 4,
  TF_BURST      = 1u << 5,
  TF_RAM        = 1u << 6,
  TF_SPOOFED    = 1u << 7,   // ESP32 answered the data phase
  TF_TOO_SLOW   = 1u << 8,   // projected length over the ISR budget: not touched
  TF_LONG       = 1u << 9,   // RAM burst (31 bytes): not touched
  TF_TIMEOUT    = 1u << 10,  // aborted mid-transaction (edge timeout / budget)
  TF_COMPLETE   = 1u << 11,  // every expected data bit was clocked
  TF_LATE_EDGE  = 1u << 12,  // SCLK rose before the ISR started: observed, never answered
  TF_CE_BOUNCE  = 1u << 13,  // CE read low mid-transfer but recovered: glitch, not an end
};

struct TxnRecord {
  int64_t  t0Us;       // ISR entry (esp_timer time)
  uint32_t durUs;      // ISR entry -> exit
  uint32_t spanUs;     // first -> last SCLK rising edge
  uint16_t flags;
  uint16_t rises;      // SCLK rising edges seen
  uint16_t dataBits;   // data bits presented (read) / latched (write)
  uint8_t  cmd;
  uint8_t  data[8];    // passive read: real chip bytes; write: MCU bytes; spoof: bytes sent
};

struct BusPins { uint32_t ce, sclk, io; };   // bit masks within GPIO_IN_REG

class BusMachine {
 public:
  static constexpr uint8_t A_DRIVE   = 0x01;  // enable output at level A_HIGH ? 1 : 0
  static constexpr uint8_t A_HIGH    = 0x02;
  static constexpr uint8_t A_RELEASE = 0x04;  // output off (high-Z)
  static constexpr uint8_t A_DONE    = 0x08;  // stop sampling, leave the ISR

  // Call once with the first sample taken in the ISR. `img` = 8-byte clock image.
  // lateEdge = SCLK rose between the previous transfer and this ISR (possible bit slip).
  DS_INLINE uint8_t begin(const BusPins& pins, uint32_t in, int64_t t0Us, const uint8_t* img,
                          const uint8_t* ram, bool armed, uint32_t budgetUs,
                          uint32_t edgeTimeoutUs, bool lateEdge) {
    p_ = pins;
    ram_ = ram;
    t0_ = t0Us; tFirst_ = t0Us; tLast_ = t0Us;
    budget_ = budgetUs; edgeTimeout_ = edgeTimeoutUs;
    armed_ = armed;
    for (uint8_t i = 0; i < 8; ++i) { img_[i] = img[i]; rec_.data[i] = 0; }
    rec_.t0Us = t0Us; rec_.durUs = 0; rec_.spanUs = 0;
    rec_.flags = 0; rec_.rises = 0; rec_.dataBits = 0; rec_.cmd = 0;
    phase_ = PH_CMD; bits_ = 0; cmd_ = 0; nBytes_ = 0; ceLow_ = 0;
    outIdx_ = 0; capBits_ = 0; wIdx_ = 0; srcByte_ = 0;
    spoof_ = false; burst_ = false; driving_ = false;
    prev_ = in;
    lowIo_ = in & p_.io;
    if (lateEdge)       { rec_.flags |= TF_LATE_EDGE;  armed_ = false; }
    if (!(in & p_.ce))  { rec_.flags |= TF_CE_GLITCH;  return A_DONE; }
    if (in & p_.sclk)   { rec_.flags |= TF_MISALIGNED; return A_DONE; }
    return 0;
  }

  // True if this sample is an SCLK rising edge (caller timestamps those only).
  DS_INLINE bool isRise(uint32_t in) const { return ((in ^ prev_) & in & p_.sclk) != 0; }

  // Feed one GPIO sample. `nowUs` is only read on SCLK rising edges.
  DS_INLINE uint8_t step(uint32_t in, int64_t nowUs) {
    if (!(in & p_.ce)) {
      // Require BUS_CE_LOW_CONFIRM consecutive low samples. One glitched sample used to end
      // the transfer here, releasing the line and zeroing every bit the MCU had left to
      // clock. See the note on BUS_CE_LOW_CONFIRM for why that shows up as 03 or 00:00.
      if (++ceLow_ >= BUS_CE_LOW_CONFIRM) return endOfTransfer();
      rec_.flags |= TF_CE_BOUNCE;
      return 0;
    }
    ceLow_ = 0;
    if (!((in ^ prev_) & p_.sclk)) {                 // no clock edge
      if (!(in & p_.sclk)) lowIo_ = in & p_.io;
      return 0;
    }
    prev_ = in;
    if (in & p_.sclk) return onRise(in, nowUs);
    lowIo_ = in & p_.io;
    return onFall();
  }

  // Call periodically from the sampling loop.
  DS_INLINE uint8_t checkTimeout(int64_t nowUs) {
    const int64_t ref = rec_.rises ? tLast_ : t0_;
    if ((nowUs - t0_) > (int64_t)budget_ || (nowUs - ref) > (int64_t)edgeTimeout_) {
      rec_.flags |= TF_TIMEOUT;
      driving_ = false;
      return A_DONE | A_RELEASE;
    }
    return 0;
  }

  DS_INLINE const TxnRecord& finish(int64_t tEndUs) {
    rec_.durUs = (uint32_t)(tEndUs - t0_);
    rec_.spanUs = (uint32_t)(tLast_ - tFirst_);
    if (phase_ == PH_READ) {
      rec_.dataBits = spoof_ ? outIdx_ : capBits_;
      if (outIdx_ >= 8u * nBytes_) rec_.flags |= TF_COMPLETE;
    } else if (phase_ == PH_WRITE) {
      rec_.dataBits = wIdx_;
      if (wIdx_ >= 8u * nBytes_) rec_.flags |= TF_COMPLETE;
    }
    return rec_;
  }

  DS_INLINE bool driving() const { return driving_; }

  uint8_t ceLow_ = 0;      // consecutive CE-low samples seen in this transfer

 private:
  enum : uint8_t { PH_CMD, PH_READ, PH_WRITE };

  DS_INLINE uint8_t onRise(uint32_t in, int64_t nowUs) {
    if (rec_.rises == 0) tFirst_ = nowUs;
    tLast_ = nowUs;
    ++rec_.rises;

    if (phase_ == PH_CMD) {
      if (in & p_.io) cmd_ |= (uint8_t)(1u << bits_);
      if (++bits_ == 8) return decide();
      return 0;
    }
    if (phase_ == PH_READ) {
      // Passive: the bit the chip presented during the low phase that just ended.
      if (!spoof_ && outIdx_ > capBits_ && outIdx_ <= 64) capture(outIdx_);
      return 0;
    }
    // PH_WRITE: MCU data is latched on the rising edge, like the chip does.
    if (wIdx_ < 8u * nBytes_) {
      if (in & p_.io) rec_.data[wIdx_ >> 3] |= (uint8_t)(1u << (wIdx_ & 7));
      ++wIdx_;
    }
    return 0;
  }

  DS_INLINE uint8_t onFall() {
    if (phase_ != PH_READ) return 0;
    const uint16_t n = outIdx_++;
    if (!spoof_) return 0;
    if (burst_) {
      if (ramBurst_) return drive(0);                // RAM burst: zeros, never floating
      if (n < 56) return drive((img_[n >> 3] >> (n & 7)) & 1u);
#if RTC_CHIP_REMOVED
      if (n < 64) return drive((img_[7] >> (n & 7)) & 1u);   // byte 7 (WP): emulate it too
#endif
      return release();                              // byte 7 (WP) and beyond: real chip
    }
    return drive((outByte_ >> (n & 7)) & 1u);        // single read: repeat the byte
  }

  DS_INLINE uint8_t decide() {
    rec_.cmd = cmd_;
    rec_.flags |= TF_CMD;
    if (!cmdValid(cmd_)) { rec_.flags |= TF_INVALID; return A_DONE; }
    const bool rd = cmdIsRead(cmd_);
    burst_ = cmdIsBurst(cmd_);
    if (rd) rec_.flags |= TF_READ;
    if (burst_) rec_.flags |= TF_BURST;
    if (cmdIsRam(cmd_)) rec_.flags |= TF_RAM;
    nBytes_ = cmdDataBytes(cmd_);
    if (nBytes_ > 8) { rec_.flags |= TF_LONG; return A_DONE; }

    // Project the remaining length from the command-phase clock rate.
    const uint32_t period  = (uint32_t)(tLast_ - tFirst_) / 7u;
    const uint32_t elapsed = (uint32_t)(tLast_ - t0_);
    if (elapsed + period * (8u * nBytes_ + 2u) > budget_) { rec_.flags |= TF_TOO_SLOW; return A_DONE; }

    phase_ = rd ? PH_READ : PH_WRITE;
    ramBurst_ = burst_ && cmdIsRam(cmd_);
#if RTC_CHIP_REMOVED
    spoof_ = rd;                 // nothing else is on the bus: every read must be answered
#else
    spoof_ = rd && armed_ && cmdIsSpoofable(cmd_);
#endif
    if (spoof_) {
      rec_.flags |= TF_SPOOFED;
      const uint8_t a = cmdAddr(cmd_);
      srcByte_ = burst_ ? 0 : a;
      // Single read: clock space 0..7 comes from the image, anything else answers zeros.
      outByte_ = cmdIsRam(cmd_) ? ((ram_ && a < 31) ? ram_[a] : 0x00)
                                : ((a <= 7) ? img_[a] : 0x00);
      if (burst_) { for (uint8_t i = 0; i < 7; ++i) rec_.data[i] = ramBurst_ ? 0 : img_[i]; }
      else        { rec_.data[0] = outByte_; }
#if IO_FIRST_BIT_EARLY
      // Present bit 0 now; the falling edges below carry bits 1..7.
      outIdx_ = 1;
      return drive(burst_ ? ((ramBurst_ ? 0u : (uint32_t)img_[0]) & 1u)
                          : ((uint32_t)outByte_ & 1u));
#endif
    }
    return 0;
  }

  DS_INLINE uint8_t endOfTransfer() {
    // Last read bit may never get a following rising edge (MCU samples, then drops CE).
    if (phase_ == PH_READ && !spoof_ && outIdx_ > capBits_ && outIdx_ <= 64) capture(outIdx_);
    const uint8_t a = A_DONE | (driving_ ? A_RELEASE : 0);
    driving_ = false;
    return a;
  }

  DS_INLINE void capture(uint16_t presented) {
    const uint16_t idx = (uint16_t)(presented - 1);
    if (lowIo_) rec_.data[idx >> 3] |= (uint8_t)(1u << (idx & 7));
    capBits_ = presented;
  }

  DS_INLINE uint8_t drive(uint32_t level) { driving_ = true; return (uint8_t)(A_DRIVE | (level ? A_HIGH : 0)); }
  DS_INLINE uint8_t release() {
    if (!driving_) return 0;
    driving_ = false;
    return A_RELEASE;
  }

  BusPins   p_{};
  TxnRecord rec_{};
  int64_t   t0_ = 0, tFirst_ = 0, tLast_ = 0;
  uint32_t  budget_ = 0, edgeTimeout_ = 0;
  uint32_t  prev_ = 0, lowIo_ = 0;
  uint16_t  outIdx_ = 0, capBits_ = 0, wIdx_ = 0;
  uint8_t   img_[8]{};
  uint8_t   phase_ = PH_CMD, bits_ = 0, cmd_ = 0, nBytes_ = 0, srcByte_ = 0;
  bool      armed_ = false, spoof_ = false, burst_ = false, driving_ = false;
  uint8_t   outByte_ = 0;          // byte repeated for a single-register read
  const uint8_t* ram_ = nullptr;   // emulated DS1302 RAM, 31 bytes
  bool      ramBurst_ = false;     // RAM burst: answered with zeros
};

}  // namespace ds1302

// ---------------------------------------------------------------- POSIX TZ rule handling
//
// POSIX form: stdoffset[dst[offset][,start[/time],end[/time]]]
// Offsets are the value added to local time to get UTC, so North America is positive.
// A name is either three or more letters or a quoted form such as <+0530> or <-07>.
//
// DST mode 1 strips the daylight half of the rule (standard time all year); mode 2 keeps
// the daylight offset all year (what a province that abolishes the change would use).

namespace tzrule {

struct Parts {
  char stdName[16];
  char dstName[16];
  long stdOffset;     // seconds
  long dstOffset;     // seconds
  bool hasDst;
};

inline const char* parseName(const char* s, char* out, size_t n) {
  size_t len = 0;
  if (*s == '<') {
    ++s;
    while (*s && *s != '>' && len + 1 < n) {
      const char c = *s;
      const bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '+' || c == '-';
      if (!ok) return nullptr;
      out[len++] = *s++;
    }
    if (*s != '>') return nullptr;
    ++s;
  } else {
    while (((*s >= 'A' && *s <= 'Z') || (*s >= 'a' && *s <= 'z')) && len + 1 < n) out[len++] = *s++;
  }
  if (len < 1) return nullptr;
  out[len] = '\0';
  return s;
}

// Offsets: [+|-]hh[:mm[:ss]]
inline const char* parseOffset(const char* s, long& secs, bool& present) {
  present = false;
  int sign = 1;
  if (*s == '+') { ++s; }
  else if (*s == '-') { sign = -1; ++s; }
  if (*s < '0' || *s > '9') return s;               // no offset here
  long v[3] = { 0, 0, 0 };
  for (int field = 0; field < 3; ++field) {
    long acc = 0;
    int digits = 0;
    while (*s >= '0' && *s <= '9' && digits < 3) { acc = acc * 10 + (*s++ - '0'); ++digits; }
    if (digits == 0) return nullptr;
    v[field] = acc;
    if (*s != ':') break;
    ++s;
  }
  if (v[0] > 24 || v[1] > 59 || v[2] > 59) return nullptr;
  secs = sign * (v[0] * 3600 + v[1] * 60 + v[2]);
  present = true;
  return s;
}

inline bool parse(const char* tz, Parts& p) {
  if (!tz || !*tz) return false;
  std::memset(&p, 0, sizeof p);
  const char* s = parseName(tz, p.stdName, sizeof p.stdName);
  if (!s) return false;
  bool present = false;
  s = parseOffset(s, p.stdOffset, present);
  if (!s || !present) return false;
  if (*s == '\0' || *s == ',') return true;         // no daylight half
  s = parseName(s, p.dstName, sizeof p.dstName);
  if (!s) return false;
  p.hasDst = true;
  p.dstOffset = p.stdOffset - 3600;                 // POSIX default: one hour ahead
  s = parseOffset(s, p.dstOffset, present);
  return s != nullptr;
}

inline void formatOffset(long secs, char* out, size_t n) {
  const long a = secs < 0 ? -secs : secs;
  const int h = (int)(a / 3600), m = (int)((a % 3600) / 60), sec = (int)(a % 60);
  const char* sign = secs < 0 ? "-" : "";
  if (sec) std::snprintf(out, n, "%s%d:%02d:%02d", sign, h, m, sec);
  else if (m) std::snprintf(out, n, "%s%d:%02d", sign, h, m);
  else std::snprintf(out, n, "%s%d", sign, h);
}

inline void formatName(const char* name, char* out, size_t n) {
  bool plain = std::strlen(name) >= 3;
  for (const char* c = name; *c && plain; ++c)
    if (!((*c >= 'A' && *c <= 'Z') || (*c >= 'a' && *c <= 'z'))) plain = false;
  if (plain) std::snprintf(out, n, "%s", name);
  else std::snprintf(out, n, "<%s>", name);
}

// mode 0 = rule unchanged, 1 = standard offset all year, 2 = daylight offset all year.
// Returns false and copies the input unchanged when the rule cannot be parsed.
inline bool applyMode(const char* tz, uint8_t mode, char* out, size_t n) {
  if (n == 0) return false;
  const char* src = tz ? tz : "";
  size_t i = 0;
  for (; src[i] && i + 1 < n; ++i) out[i] = src[i];
  out[i] = '\0';
  if (mode == 0) return true;
  Parts p;
  if (!parse(tz, p)) return false;
  const bool wantDst = (mode == 2) && p.hasDst;
  char name[20], off[16];
  formatName(wantDst ? p.dstName : p.stdName, name, sizeof name);
  formatOffset(wantDst ? p.dstOffset : p.stdOffset, off, sizeof off);
  std::snprintf(out, n, "%s%s", name, off);
  return true;
}

// "follows the zone rule", "standard time all year", "daylight time all year"
inline const char* modeName(uint8_t mode) {
  if (mode == 1) return "standard time all year";
  if (mode == 2) return "daylight time all year";
  return "follows the zone rule";
}

}  // namespace tzrule

// ================================================================ PURE LOGIC END

struct BusCounters {
  uint32_t ceRises;       // CE rising-edge interrupts
  uint32_t glitches;      // CE already low when the ISR ran
  uint32_t misaligned;    // SCLK high at ISR entry
  uint32_t lateEdge;      // SCLK rose before the ISR started (not answered)
  uint32_t invalid;       // command bit7 = 0
  uint32_t singleReads;   // valid read commands, one register
  uint32_t burstReads;    // valid read commands, burst
  uint32_t writes;        // valid write commands
  uint32_t spoofed;       // reads answered by the ESP32
  uint32_t passReads;     // complete clock reads answered by the real chip (captured)
  uint32_t tooSlow;       // skipped: projected length over the ISR budget
  uint32_t longTxn;       // skipped: RAM burst
  uint32_t timeouts;      // aborted mid-transaction
  uint32_t incomplete;    // CE dropped before all data bits were clocked
  uint32_t ceBounces;     // transfers where a CE low sample was rejected as a glitch
  uint32_t spanUsSum;     // sum of first->last SCLK rising edge spans
  uint32_t spanPeriods;   // sum of (rising edges - 1)
};

struct BusSnapshot {
  BusCounters c;
  ds1302::TxnRecord last;       // last transaction with a full command byte
  ds1302::TxnRecord lastRead;
  ds1302::TxnRecord lastWrite;
  uint8_t  real[7];             // real DS1302 clock bytes captured from passive reads
  uint8_t  realMask;            // bit n set = real[n] captured
  int64_t  realUs;              // esp_timer time of the latest capture
  int64_t  lastCeUs;            // esp_timer time of the latest CE rise
  uint32_t pollGapUs;           // latest gap between polling cycles
  int8_t   learnedHour;         // -1 unknown, 0 = MCU uses 24 h, 1 = MCU uses 12 h
  uint32_t cmdSeen[8];          // 256-bit set of command bytes decoded
};

namespace bus {

// Configure pins and install the ISR. Returns false if the ISR could not be installed.
bool begin();

// True once the CE interrupt is live.
bool isrInstalled();

// True if the GPIO ISR service was installed IRAM-safe by this module.
bool isrIramSafe();

// Hour format the MCU was seen using: -1 unknown, 0 = 24 h, 1 = 12 h.
int8_t learnedHour();

// Latest gap between polling cycles, microseconds (0 = not seen yet).
uint32_t pollGapUs();

// Hand a new 8-byte clock image to the ISR. armed = ISR may answer clock reads.
void setImage(const uint8_t reg[8], bool armed);

// Consistent copy of all diagnostics.
void snapshot(BusSnapshot& out);

// Reset the command-seen set (used by the passive bus check).
void clearCmdSeen();

}  // namespace bus

// ---------------------------------------------------------------- implementation

namespace bus {
namespace impl {

using namespace ds1302;

struct Shared {
  volatile uint32_t seq;   // seqlock: odd while the ISR is publishing
  BusCounters c;
  TxnRecord last, lastRead, lastWrite;
  uint8_t  real[7];
  uint8_t  realMask;
  int64_t  realUs;
  int64_t  lastCeUs;
  uint32_t pollGapUs;
  int8_t   learnedHour;
  uint32_t cmdSeen[8];
};

struct ImageSlot { uint8_t reg[8]; };

Shared            s_sh;
ImageSlot         s_img[2];
volatile uint8_t  s_imgActive = 0;     // written only by setImage()
DRAM_ATTR uint8_t s_ram[31] = {0};     // emulated DS1302 scratch RAM (chip-removed mode)
volatile bool     s_armed = false;
bool              s_iramSafe = false;
bool              s_installed = false;
BusMachine        s_machine;           // single instance: the ISR is not re-entrant

DRAM_ATTR const BusPins s_pins = { 1UL << PIN_CE, 1UL << PIN_SCLK, 1UL << PIN_IO };

constexpr uint32_t IO_MASK = 1UL << PIN_IO;
constexpr uint32_t SCLK_MASK = 1UL << PIN_SCLK;

int levelFlag() {
#if BUS_INTR_LEVEL == 1
  return ESP_INTR_FLAG_LEVEL1;
#elif BUS_INTR_LEVEL == 2
  return ESP_INTR_FLAG_LEVEL2;
#else
  return ESP_INTR_FLAG_LEVEL3;
#endif
}

inline __attribute__((always_inline)) void learnHour(Shared& s, uint8_t reg) {
  if (decodeHour(reg) >= 0) s.learnedHour = (reg & 0x80) ? 1 : 0;
}

IRAM_ATTR void publish(const TxnRecord& r) {
  Shared& s = s_sh;
  s.seq = s.seq + 1;
  BusCounters& c = s.c;
  const uint16_t f = r.flags;

  ++c.ceRises;
  if (s.lastCeUs != 0 && (r.t0Us - s.lastCeUs) > (int64_t)POLL_GROUP_GAP_US)
    s.pollGapUs = (uint32_t)(r.t0Us - s.lastCeUs);
  s.lastCeUs = r.t0Us;

  if (f & TF_CE_GLITCH)  ++c.glitches;
  if (f & TF_CE_BOUNCE)  ++c.ceBounces;
  if (f & TF_MISALIGNED) ++c.misaligned;
  if (f & TF_LATE_EDGE)  ++c.lateEdge;
  if (f & TF_TIMEOUT)    ++c.timeouts;
  if (f & TF_TOO_SLOW)   ++c.tooSlow;
  if (f & TF_LONG)       ++c.longTxn;
  if (r.rises >= 2) { c.spanUsSum += r.spanUs; c.spanPeriods += (uint32_t)(r.rises - 1); }

  if (f & TF_CMD) {
    s.cmdSeen[r.cmd >> 5] |= 1UL << (r.cmd & 31);
    s.last = r;
    if (f & TF_INVALID) {
      ++c.invalid;
    } else {
      const bool handled = !(f & (TF_TOO_SLOW | TF_LONG | TF_TIMEOUT | TF_LATE_EDGE));
      const bool clock   = !(f & TF_RAM);
      const bool burst   = (f & TF_BURST) != 0;
      const uint8_t addr = cmdAddr(r.cmd);
      if (handled && !(f & TF_COMPLETE)) ++c.incomplete;

      // Store single-byte RAM writes. The MCU keeps its own state in the DS1302's RAM and
      // must read back what it wrote; answering zeros silently corrupts it.
      if (handled && !clock && !burst && !(f & TF_READ) && (f & TF_COMPLETE) && addr < 31)
        s_ram[addr] = r.data[0];

      if (f & TF_READ) {
        if (burst) ++c.burstReads; else ++c.singleReads;
        s.lastRead = r;
        if (f & TF_SPOOFED) {
          ++c.spoofed;
        } else if (handled && clock) {
          if (burst && r.dataBits >= 56) {              // 7 clock bytes captured
            for (uint8_t i = 0; i < 7; ++i) s.real[i] = r.data[i];
            s.realMask = 0x7F;
            s.realUs = r.t0Us;
            learnHour(s, r.data[REG_HOUR]);
            ++c.passReads;
          } else if (!burst && addr <= REG_YEAR && r.dataBits >= 8) {
            s.real[addr] = r.data[0];
            s.realMask |= (uint8_t)(1u << addr);
            s.realUs = r.t0Us;
            if (addr == REG_HOUR) learnHour(s, r.data[0]);
            ++c.passReads;
          }
        }
      } else {
        ++c.writes;
        s.lastWrite = r;
        if (handled && clock) {
          if (burst && r.dataBits >= 24) learnHour(s, r.data[REG_HOUR]);
          else if (!burst && addr == REG_HOUR && r.dataBits >= 8) learnHour(s, r.data[0]);
        }
      }
    }
  }
  s.seq = s.seq + 1;
}

// CE rising edge: follow the whole transaction by polling, then return.
IRAM_ATTR void ceRiseIsr(void*) {
  const int64_t t0 = esp_timer_get_time();
  uint32_t in = REG_READ(GPIO_IN_REG);
#if BUS_SLIP_GUARD
  const bool lateEdge = (REG_READ(GPIO_STATUS_REG) & SCLK_MASK) != 0;
#else
  const bool lateEdge = false;
#endif
  BusMachine& m = s_machine;
  uint8_t a = m.begin(s_pins, in, t0, s_img[s_imgActive].reg, s_ram, s_armed,
                      BUS_TXN_BUDGET_US, BUS_EDGE_TIMEOUT_US, lateEdge);
  bool outOn = false;
  uint32_t spins = 0;

  while (!(a & BusMachine::A_DONE)) {
    in = REG_READ(GPIO_IN_REG);
    a = m.step(in, m.isRise(in) ? esp_timer_get_time() : 0);
    if (a & BusMachine::A_DRIVE) {
      // Level first, then output enable: no glitch on the first driven bit.
#if IO_OPEN_DRAIN
      if (a & BusMachine::A_HIGH) {                  // high = let the external pull-up do it
        if (outOn) { REG_WRITE(GPIO_ENABLE_W1TC_REG, IO_MASK); outOn = false; }
      } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, IO_MASK);
        if (!outOn) { REG_WRITE(GPIO_ENABLE_W1TS_REG, IO_MASK); outOn = true; }
      }
#else
      REG_WRITE((a & BusMachine::A_HIGH) ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, IO_MASK);
      if (!outOn) { REG_WRITE(GPIO_ENABLE_W1TS_REG, IO_MASK); outOn = true; }
#endif
    } else if (a & BusMachine::A_RELEASE) {
      REG_WRITE(GPIO_ENABLE_W1TC_REG, IO_MASK);
      outOn = false;
    }
    if ((++spins & 0x3FF) == 0 && !(a & BusMachine::A_DONE))
      a |= m.checkTimeout(esp_timer_get_time());
  }

  REG_WRITE(GPIO_ENABLE_W1TC_REG, IO_MASK);   // always leave the ISR high-Z
  REG_WRITE(GPIO_STATUS_W1TC_REG, SCLK_MASK); // arm the slip latch for the next transfer
  publish(m.finish(esp_timer_get_time()));
}

}  // namespace impl

using namespace impl;

bool begin() {
  s_sh.learnedHour = -1;

  // CE + SCLK: plain inputs, no pulls (the DS1302 has 40k pulldowns, the MCU drives both).
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = (1ULL << PIN_CE) | (1ULL << PIN_SCLK);
  cfg.mode = GPIO_MODE_INPUT;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  if (gpio_config(&cfg) != ESP_OK) return false;

  // I/O: input always on; the ISR toggles output enable. Output latch preset low.
  gpio_set_level((gpio_num_t)PIN_IO, 0);
  cfg.pin_bit_mask = 1ULL << PIN_IO;
  cfg.mode = GPIO_MODE_INPUT_OUTPUT;
  if (gpio_config(&cfg) != ESP_OK) return false;
  REG_WRITE(GPIO_ENABLE_W1TC_REG, IO_MASK);   // high-Z
  gpio_set_drive_capability((gpio_num_t)PIN_IO, (gpio_drive_cap_t)IO_DRIVE_STRENGTH);

  const esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | levelFlag());
  if (err == ESP_OK) s_iramSafe = true;
  else if (err != ESP_ERR_INVALID_STATE) return false;   // INVALID_STATE: installed elsewhere

  // SCLK: rising-edge status latch only. Its interrupt stays disabled, so the GPIO ISR
  // service never dispatches it (it only sees status & enable); the CE ISR reads and
  // clears the raw bit itself.
  if (gpio_set_intr_type((gpio_num_t)PIN_SCLK, GPIO_INTR_POSEDGE) != ESP_OK) return false;
  gpio_intr_disable((gpio_num_t)PIN_SCLK);
  REG_WRITE(GPIO_STATUS_W1TC_REG, SCLK_MASK);

  if (gpio_set_intr_type((gpio_num_t)PIN_CE, GPIO_INTR_POSEDGE) != ESP_OK) return false;
  if (gpio_isr_handler_add((gpio_num_t)PIN_CE, ceRiseIsr, nullptr) != ESP_OK) return false;
  s_installed = gpio_intr_enable((gpio_num_t)PIN_CE) == ESP_OK;
  return s_installed;
}

bool isrInstalled() { return s_installed; }

bool isrIramSafe() { return s_iramSafe; }

int8_t learnedHour() { return s_sh.learnedHour; }

uint32_t pollGapUs() { return s_sh.pollGapUs; }

// Single writer (the time task). The ISR only ever reads the active slot.
void setImage(const uint8_t reg[8], bool armed) {
  if (!armed) s_armed = false;
  const uint8_t next = (uint8_t)(s_imgActive ^ 1);
  std::memcpy(s_img[next].reg, reg, 8);
  s_imgActive = next;
  if (armed) s_armed = true;
}

void snapshot(BusSnapshot& o) {
  for (;;) {
    const uint32_t seq = s_sh.seq;
    o.c = s_sh.c;
    o.last = s_sh.last;
    o.lastRead = s_sh.lastRead;
    o.lastWrite = s_sh.lastWrite;
    std::memcpy(o.real, s_sh.real, sizeof o.real);
    o.realMask = s_sh.realMask;
    o.realUs = s_sh.realUs;
    o.lastCeUs = s_sh.lastCeUs;
    o.pollGapUs = s_sh.pollGapUs;
    o.learnedHour = s_sh.learnedHour;
    std::memcpy(o.cmdSeen, s_sh.cmdSeen, sizeof o.cmdSeen);
    if (!(seq & 1) && seq == s_sh.seq) return;
  }
}

void clearCmdSeen() {
  for (uint8_t i = 0; i < 8; ++i) s_sh.cmdSeen[i] = 0;
}

}  // namespace bus

// ================================================================ SETTINGS (NVS)

enum : uint8_t { HOURFMT_AUTO = 0, HOURFMT_24 = 1, HOURFMT_12 = 2 };
enum : uint8_t { DST_AUTO = 0, DST_STANDARD = 1, DST_DAYLIGHT = 2 };

struct Settings {
  char     ntpServer[64];
  char     tz[64];            // POSIX rule as chosen in the UI, before the DST mode is applied
  uint32_t syncIntervalS;
  uint8_t  dstMode;
  uint8_t  hourFormat;
  bool     spoofEnabled;
  bool     bootCountdown;
  char     wxStation[8];      // SWOB-ML station code, e.g. CWWB
  int8_t   wifiTxDbm;         // 2..20, quantised by the radio to the steps above
  uint8_t  wifiPsMode;        // 0 none, 1 min modem, 2 max modem
};

// The transmit powers the radio can actually produce, in dBm. Anything else rounds down.
const int8_t WIFI_TX_STEPS[] = { 2, 5, 7, 8, 11, 13, 14, 15, 16, 18, 20 };
const uint8_t WIFI_TX_STEPS_N = sizeof WIFI_TX_STEPS / sizeof WIFI_TX_STEPS[0];

Settings g_settings;

namespace settings {

namespace impl {

const char* const NVS_NS = "ntpclock";

void copyStr(char* dst, size_t n, const char* src) {
  std::strncpy(dst, src, n - 1);
  dst[n - 1] = '\0';
}

bool isAlnum(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
}

}  // namespace impl

// Hostname or IPv4 literal, 1..63 characters.
bool validNtpServer(const char* s) {
  const size_t n = std::strlen(s);
  if (n == 0 || n > 63) return false;
  char prev = '.';                                   // forbids a leading '.' or '-'
  for (size_t i = 0; i < n; ++i) {
    const char c = s[i];
    if (c == '.') {
      if (prev == '.' || prev == '-') return false;   // empty label or label ending in '-'
    } else if (c == '-') {
      if (prev == '.') return false;                  // label starting with '-'
    } else if (!impl::isAlnum(c)) {
      return false;
    }
    prev = c;
  }
  return prev != '.' && prev != '-';
}

// Snap a requested transmit power to a step the radio can actually produce, never upward:
// asking for 12 dBm and silently getting 11 is the radio's behaviour, so make it visible.
int8_t quantiseTxDbm(int v) {
  if (v <= WIFI_TX_STEPS[0]) return WIFI_TX_STEPS[0];
  for (uint8_t i = WIFI_TX_STEPS_N; i-- > 0; )
    if (v >= WIFI_TX_STEPS[i]) return WIFI_TX_STEPS[i];
  return WIFI_TX_STEPS[0];
}

// SWOB-ML station code: 3..7 characters, upper-case letters and digits only.
bool validWxStation(const char* s) {
  const size_t n = std::strlen(s);
  if (n < 3 || n > 7) return false;
  for (size_t i = 0; i < n; ++i) {
    const char c = s[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) return false;
  }
  return true;
}

// POSIX TZ character set, 3..63 characters, and it must parse as a rule.
bool validTz(const char* s) {
  const size_t n = std::strlen(s);
  if (n < 3 || n > 63) return false;
  if (!(impl::isAlnum(s[0]) || s[0] == '<')) return false;
  for (size_t i = 0; i < n; ++i) {
    const char c = s[i];
    if (!(impl::isAlnum(c) || std::strchr("<>+-:,./", c))) return false;
  }
  tzrule::Parts p;
  return tzrule::parse(s, p);
}

uint32_t clampInterval(uint32_t s) {
  if (s < NTP_MIN_INTERVAL_S) return NTP_MIN_INTERVAL_S;
  if (s > NTP_MAX_INTERVAL_S) return NTP_MAX_INTERVAL_S;
  return s;
}

const char* hourFormatName(uint8_t f) {
  if (f == HOURFMT_24) return "24h";
  if (f == HOURFMT_12) return "12h";
  return "auto";
}

bool parseHourFormat(const char* s, uint8_t& out) {
  if (!std::strcmp(s, "auto")) { out = HOURFMT_AUTO; return true; }
  if (!std::strcmp(s, "24h")) { out = HOURFMT_24; return true; }
  if (!std::strcmp(s, "12h")) { out = HOURFMT_12; return true; }
  return false;
}

const char* dstModeName(uint8_t m) {
  if (m == DST_STANDARD) return "standard";
  if (m == DST_DAYLIGHT) return "daylight";
  return "auto";
}

bool parseDstMode(const char* s, uint8_t& out) {
  if (!std::strcmp(s, "auto")) { out = DST_AUTO; return true; }
  if (!std::strcmp(s, "standard")) { out = DST_STANDARD; return true; }
  if (!std::strcmp(s, "daylight")) { out = DST_DAYLIGHT; return true; }
  return false;
}

// Effective POSIX rule: the stored zone rule with the DST mode applied.
void effectiveTz(char* out, size_t n) {
  tzrule::applyMode(g_settings.tz, g_settings.dstMode, out, n);
}

void load() {
  impl::copyStr(g_settings.ntpServer, sizeof g_settings.ntpServer, DEFAULT_NTP_SERVER);
  impl::copyStr(g_settings.tz, sizeof g_settings.tz, DEFAULT_TZ);
  g_settings.syncIntervalS = DEFAULT_SYNC_INTERVAL_S;
  g_settings.dstMode = DEFAULT_DST_MODE;
  g_settings.hourFormat = DEFAULT_HOUR_FORMAT;
  g_settings.spoofEnabled = DEFAULT_SPOOF_ENABLED;
  g_settings.bootCountdown = DEFAULT_BOOT_COUNTDOWN;
  impl::copyStr(g_settings.wxStation, sizeof g_settings.wxStation, WX_STATION_DEFAULT);
  g_settings.wifiTxDbm = WIFI_TX_DBM_DEFAULT;
  g_settings.wifiPsMode = WIFI_PS_MODE_DEFAULT;

  Preferences p;
  if (!p.begin(impl::NVS_NS, false)) return;
  char buf[64];
  if (p.isKey("ntp") && p.getString("ntp", buf, sizeof buf) > 0 && validNtpServer(buf))
    impl::copyStr(g_settings.ntpServer, sizeof g_settings.ntpServer, buf);
  if (p.isKey("tz") && p.getString("tz", buf, sizeof buf) > 0 && validTz(buf))
    impl::copyStr(g_settings.tz, sizeof g_settings.tz, buf);
  g_settings.syncIntervalS = clampInterval(p.getULong("ival", DEFAULT_SYNC_INTERVAL_S));
  const uint8_t dst = p.getUChar("dst", DEFAULT_DST_MODE);
  g_settings.dstMode = dst <= DST_DAYLIGHT ? dst : (uint8_t)DST_AUTO;
  const uint8_t hf = p.getUChar("hrfmt", DEFAULT_HOUR_FORMAT);
  g_settings.hourFormat = hf <= HOURFMT_12 ? hf : (uint8_t)HOURFMT_AUTO;
  g_settings.spoofEnabled = p.getBool("spoof", DEFAULT_SPOOF_ENABLED);
  g_settings.bootCountdown = p.getBool("bootcd", DEFAULT_BOOT_COUNTDOWN);
  if (p.isKey("wxstn") && p.getString("wxstn", buf, sizeof buf) > 0 && validWxStation(buf))
    impl::copyStr(g_settings.wxStation, sizeof g_settings.wxStation, buf);
  g_settings.wifiTxDbm = quantiseTxDbm(p.getChar("wtx", WIFI_TX_DBM_DEFAULT));
  const uint8_t ps = p.getUChar("wps", WIFI_PS_MODE_DEFAULT);
  g_settings.wifiPsMode = ps <= 2 ? ps : (uint8_t)0;
  p.end();
}

bool save() {
  Preferences p;
  if (!p.begin(impl::NVS_NS, false)) return false;
  bool ok = p.putString("ntp", g_settings.ntpServer) > 0;
  ok = p.putString("tz", g_settings.tz) > 0 && ok;
  ok = p.putULong("ival", g_settings.syncIntervalS) > 0 && ok;
  ok = p.putUChar("dst", g_settings.dstMode) > 0 && ok;
  ok = p.putUChar("hrfmt", g_settings.hourFormat) > 0 && ok;
  ok = p.putBool("spoof", g_settings.spoofEnabled) > 0 && ok;
  ok = p.putBool("bootcd", g_settings.bootCountdown) > 0 && ok;
  ok = p.putString("wxstn", g_settings.wxStation) > 0 && ok;
  ok = p.putChar("wtx", g_settings.wifiTxDbm) > 0 && ok;
  ok = p.putUChar("wps", g_settings.wifiPsMode) > 0 && ok;
  p.end();
  return ok;
}

}  // namespace settings

// ================================================================ TIMEKEEPING (SNTP + TZ)

namespace timekeeping {

namespace impl {

char              server_[64];          // lwIP keeps this pointer, so it must stay valid
char              activeTz_[64];        // effective POSIX rule in use
SemaphoreHandle_t tzMutex_ = nullptr;
portMUX_TYPE      mux_ = portMUX_INITIALIZER_UNLOCKED;
time_t            lastSyncEpoch_ = 0;
int64_t           lastSyncUs_ = -1;
int64_t           baseNtpUs_ = 0;       // NTP time at the drift baseline sync, microseconds
int64_t           baseEspUs_ = 0;       // esp_timer at that same sync
bool              haveBase_ = false;
float             driftPpm_ = 0.0f;     // crystal fast(+) / slow(-) versus NTP
uint32_t          driftSpanS_ = 0;      // baseline length the figure is measured over
uint32_t          syncCount_ = 0;
uint32_t          intervalS_ = DEFAULT_SYNC_INTERVAL_S;

constexpr time_t PLAUSIBLE_EPOCH = 1735689600;   // 2025-01-01T00:00:00Z

void onSync(struct timeval* tv) {                // runs in the lwIP task
  const int64_t now = esp_timer_get_time();
  portENTER_CRITICAL(&mux_);
  lastSyncEpoch_ = tv->tv_sec;
  lastSyncUs_ = now;
  ++syncCount_;
  // Crystal drift: esp_timer measures elapsed time on the local crystal, NTP measures the
  // real elapsed time. The difference over a long baseline is the crystal error.
  const int64_t ntpUs = (int64_t)tv->tv_sec * 1000000LL + (int64_t)tv->tv_usec;
  if (!haveBase_) { baseNtpUs_ = ntpUs; baseEspUs_ = now; haveBase_ = true; }
  else {
    const int64_t dNtp = ntpUs - baseNtpUs_;
    const int64_t dEsp = now - baseEspUs_;
    if (dNtp > 0) {
      driftPpm_ = (float)((double)(dEsp - dNtp) * 1000000.0 / (double)dNtp);
      driftSpanS_ = (uint32_t)(dNtp / 1000000LL);
    }
  }
  portEXIT_CRITICAL(&mux_);
}

}  // namespace impl

void setTz(const char* tz) {
  xSemaphoreTake(impl::tzMutex_, portMAX_DELAY);
  std::strncpy(impl::activeTz_, tz, sizeof impl::activeTz_ - 1);
  impl::activeTz_[sizeof impl::activeTz_ - 1] = '\0';
  setenv("TZ", impl::activeTz_, 1);
  tzset();
  xSemaphoreGive(impl::tzMutex_);
}

void begin(const char* ntpServer, const char* tz, uint32_t intervalS) {
  if (!impl::tzMutex_) impl::tzMutex_ = xSemaphoreCreateMutex();
  std::strncpy(impl::server_, ntpServer, sizeof impl::server_ - 1);
  impl::server_[sizeof impl::server_ - 1] = '\0';
  impl::intervalS_ = intervalS;
  setTz(tz);
}

// (Re)start SNTP: one server, no fallbacks, no DHCP-supplied servers, immediate request.
void restart() {
  if (esp_sntp_enabled()) esp_sntp_stop();
  esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
#if CONFIG_LWIP_DHCP_GET_NTP_SRV
  esp_sntp_servermode_dhcp(false);
#endif
  esp_sntp_setservername(0, impl::server_);
  for (uint8_t i = 1; i < CONFIG_LWIP_SNTP_MAX_SERVERS; ++i)
    esp_sntp_setserver(i, nullptr);                // clears address and name: no fallbacks
  sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);        // step the clock, do not slew
  sntp_set_sync_interval(impl::intervalS_ * 1000UL);
  sntp_set_time_sync_notification_cb(impl::onSync);
  esp_sntp_init();
}

void setServer(const char* ntpServer) {
  if (esp_sntp_enabled()) esp_sntp_stop();         // stop before the buffer changes
  std::strncpy(impl::server_, ntpServer, sizeof impl::server_ - 1);
  impl::server_[sizeof impl::server_ - 1] = '\0';
  restart();
}

void setSyncInterval(uint32_t intervalS) {
  impl::intervalS_ = intervalS;
  sntp_set_sync_interval(intervalS * 1000UL);
  if (esp_sntp_enabled()) sntp_restart();          // apply now instead of after the old interval
}

// Immediate poll without changing any setting.
bool syncNow() {
  if (!esp_sntp_enabled()) { restart(); return true; }
  return sntp_restart();
}

bool localNow(std::tm& out, time_t* epoch = nullptr) {
  const time_t now = time(nullptr);
  if (epoch) *epoch = now;
  xSemaphoreTake(impl::tzMutex_, portMAX_DELAY);
  localtime_r(&now, &out);
  xSemaphoreGive(impl::tzMutex_);
  return now >= impl::PLAUSIBLE_EPOCH;
}

void toLocal(time_t epoch, std::tm& out) {
  xSemaphoreTake(impl::tzMutex_, portMAX_DELAY);
  localtime_r(&epoch, &out);
  xSemaphoreGive(impl::tzMutex_);
}

uint32_t syncCount() {
  portENTER_CRITICAL(&impl::mux_);
  const uint32_t n = impl::syncCount_;
  portEXIT_CRITICAL(&impl::mux_);
  return n;
}

bool everSynced() { return syncCount() > 0; }

// Measured crystal drift versus NTP. Valid once the baseline is long enough to be meaningful.
bool     driftValid() { return impl::driftSpanS_ >= 300; }
float    driftPpm()   { return impl::driftPpm_; }
uint32_t driftSpanS() { return impl::driftSpanS_; }

time_t lastSyncEpoch() {
  portENTER_CRITICAL(&impl::mux_);
  const time_t t = impl::lastSyncEpoch_;
  portEXIT_CRITICAL(&impl::mux_);
  return t;
}

int64_t lastSyncAgeUs() {
  portENTER_CRITICAL(&impl::mux_);
  const int64_t t = impl::lastSyncUs_;
  portEXIT_CRITICAL(&impl::mux_);
  return t < 0 ? -1 : esp_timer_get_time() - t;
}

uint32_t syncIntervalS() { return impl::intervalS_; }
const char* server() { return impl::server_; }
const char* activeTz() { return impl::activeTz_; }

}  // namespace timekeeping

// ================================================================ SPOOF CONTROL
//
// One task rebuilds the DS1302 register image from local time every 20 ms and decides
// whether the ISR may answer reads. It also owns the two display sequences that drive
// deliberately fake times: the boot countdown (00:00, 03:00, 02:00, 01:00) and the
// display march started from the web UI.

namespace spoof {

struct Status {
  bool        armed;        // ISR answers clock reads
  bool        haveTime;     // image holds a real time (or a sequence time)
  bool        march;        // display march running
  bool        countdown;    // boot countdown running
  bool        hour12;       // hours register encoded in 12 hour mode
  uint8_t     reg[8];
  const char* reason;
};

namespace impl {

portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
Status       status_ = {};
volatile int forcedHour_ = -1;          // display march (set from the web UI test)
volatile int forcedMin_ = 0;
volatile bool countdownWanted_ = false;
bool         countdownRunning_ = false;
uint8_t      countdownStep_ = 0;
uint32_t     countdownStepMs_ = BOOT_COUNTDOWN_STEP_MS;
uint32_t     countdownStartedMs_ = 0;

const uint8_t COUNTDOWN_HOURS[4] = { 0, 3, 2, 1 };

ds1302::HourMode resolveHourMode() {
  if (g_settings.hourFormat == HOURFMT_12) return ds1302::HourMode::H12;
  if (g_settings.hourFormat == HOURFMT_24) return ds1302::HourMode::H24;
  return bus::learnedHour() == 1 ? ds1302::HourMode::H12 : ds1302::HourMode::H24;
}

bool ntpFresh() {
#if NTP_HOLDOVER_HOURS == 0
  return WiFi.isConnected() && timekeeping::everSynced();
#else
  const int64_t age = timekeeping::lastSyncAgeUs();
  return age >= 0 && age <= (int64_t)NTP_HOLDOVER_HOURS * 3600LL * 1000000LL;
#endif
}

// Step length: at least one MCU polling cycle and a half, so every step is actually read.
uint32_t sequenceStepMs() {
  const uint32_t gapMs = bus::pollGapUs() / 1000;
  uint32_t ms = BOOT_COUNTDOWN_STEP_MS;
  if (gapMs + gapMs / 2 > ms) ms = gapMs + gapMs / 2;
  if (ms > 5000UL) ms = 5000UL;
  return ms;
}

void buildFromHm(const std::tm& base, bool baseValid, int hour, int minute,
                 ds1302::HourMode mode, uint8_t reg[8]) {
  std::tm t = base;
  if (!baseValid) { t.tm_year = 100; t.tm_mon = 0; t.tm_mday = 1; t.tm_wday = 6; }  // 2000-01-01
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = 0;
  ds1302::buildImage(t, mode, reg);
}

void imageTask(void*) {
  uint8_t    lastReg[8] = {};
  bool       lastArmed = false;
  bool       first = true;
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    std::tm tm = {};
    const bool timeOk = timekeeping::localNow(tm);
    const ds1302::HourMode mode = resolveHourMode();
    const uint32_t nowMs = millis();
    const int forced = forcedHour_;

    Status st = {};
    st.hour12 = (mode == ds1302::HourMode::H12);

    if (forced >= 0) {                                  // display march wins
      countdownRunning_ = false;
      countdownWanted_ = false;
      buildFromHm(tm, timeOk, forced, forcedMin_, mode, st.reg);
      st.armed = true; st.haveTime = true; st.march = true;
      st.reason = "on: display march";
    } else {
      const bool wouldArm = g_settings.spoofEnabled && timekeeping::everSynced() && ntpFresh();
      if (countdownWanted_ && wouldArm && !countdownRunning_) {
        countdownRunning_ = true;
        countdownStep_ = 0;
        countdownStepMs_ = sequenceStepMs();
        countdownStartedMs_ = nowMs;
      }
      if (countdownRunning_) {
        const uint32_t elapsed = nowMs - countdownStartedMs_;
        countdownStep_ = (uint8_t)(elapsed / countdownStepMs_);
        if (countdownStep_ >= (uint8_t)(sizeof COUNTDOWN_HOURS) || !wouldArm) {
          countdownRunning_ = false;
          countdownWanted_ = false;
        }
      }
      if (countdownRunning_) {
        buildFromHm(tm, timeOk, COUNTDOWN_HOURS[countdownStep_], 0, mode, st.reg);
        st.armed = true; st.haveTime = true; st.countdown = true;
        st.reason = "on: boot countdown";
      } else {
        if (timeOk) { ds1302::buildImage(tm, mode, st.reg); st.haveTime = true; }
#if RTC_CHIP_REMOVED
        // No chip on the bus: answer regardless of sync state. Until the first NTP sync,
        // hand out a valid BCD image so the MCU never latches garbage.
        if (!timeOk) {
          st.reg[0] = 0x00; st.reg[1] = 0x00; st.reg[2] = 0x00;
          st.reg[3] = 0x01; st.reg[4] = 0x01; st.reg[5] = 0x01; st.reg[6] = 0x26;
        }
        st.reg[7] = 0x80;                      // control register reads back write-protected
        st.armed = true; st.haveTime = true;
        st.reason = timekeeping::everSynced() ? "on: NTP time (DS1302 removed)"
                                              : "on: emulating, waiting for first NTP sync";
#else
        if (!g_settings.spoofEnabled)        st.reason = "off: spoof disabled (passthrough)";
        else if (!timekeeping::everSynced()) st.reason = "off: waiting for first NTP sync";
        else if (!ntpFresh())                st.reason = NTP_HOLDOVER_HOURS == 0
                                                 ? "off: Wi-Fi down (passthrough)"
                                                 : "off: NTP holdover expired (passthrough)";
        else { st.armed = true;              st.reason = "on: NTP time"; }
#endif
      }
    }

    if (first || st.armed != lastArmed || std::memcmp(st.reg, lastReg, 8) != 0) {
      bus::setImage(st.reg, st.armed);
      std::memcpy(lastReg, st.reg, 8);
      lastArmed = st.armed;
      first = false;
    }

    portENTER_CRITICAL(&mux_);
    status_ = st;
    portEXIT_CRITICAL(&mux_);

    xTaskDelayUntil(&wake, pdMS_TO_TICKS(20));
  }
}

}  // namespace impl

void begin() {
  impl::countdownWanted_ = g_settings.bootCountdown;
  xTaskCreate(impl::imageTask, "ds1302img", 4096, nullptr, 3, nullptr);
}

void getStatus(Status& out) {
  portENTER_CRITICAL(&impl::mux_);
  out = impl::status_;
  portEXIT_CRITICAL(&impl::mux_);
}

// hour 0..23 forces that time onto the display; hour < 0 returns to normal operation.
void setForced(int hour, int minute) {
  impl::forcedMin_ = minute;
  impl::forcedHour_ = hour;
}

// Replay the power-up countdown (also used by the web UI button).
void requestCountdown() {
  impl::countdownRunning_ = false;
  impl::countdownWanted_ = true;
}

bool countdownPending() { return impl::countdownWanted_ || impl::countdownRunning_; }

}  // namespace spoof

namespace diag {

struct Rates {
  float       cePerSec;     // CE assertions per second (last 1 s window)
  float       sclkHz;       // average SCLK frequency of recent transfers, 0 = none seen
  uint32_t    pollGapMs;    // latest gap between polling cycles, 0 = unknown
  const char* pollStyle;    // "burst", "per-register", "mixed", "none"
};

void begin();
void loop();                       // call every loop() pass

void getRates(Rates& out);

bool startPassiveCheck();          // false if a test is already running
bool startMarch();
bool startPinScan();               // watch every usable GPIO for edges
bool startIsrSelfTest();           // drive PIN_CE from the ESP32 and prove the ISR fires

bool        testRunning();
const char* testName();            // "passive", "march" or "" if none has run
uint32_t    testElapsedMs();
uint32_t    testDurationMs();      // planned length of the running/last test
const String& testReport();        // multi-line text; first line "RESULT: ..."

}  // namespace diag

// ---------------------------------------------------------------- implementation

namespace diag {
namespace impl {

enum class Kind : uint8_t { None, Passive, March, PinScan, IsrSelf };

constexpr uint8_t MARCH_STEPS   = 5;     // 01:00 .. 05:00
constexpr uint8_t STYLE_WINDOWS = 5;     // polling style judged over the last 5 s

struct RateState {
  bool        init = false;
  BusSnapshot prev = {};
  uint32_t    prevMs = 0;
  diag::Rates rates = { 0.0f, 0.0f, 0, "none" };
  uint32_t    singleHist[STYLE_WINDOWS] = {};
  uint32_t    burstHist[STYLE_WINDOWS] = {};
  uint8_t     histIdx = 0;
};

struct TestState {
  Kind        kind = Kind::None;
  bool        running = false;
  uint32_t    startMs = 0, durMs = 0, endMs = 0;
  BusSnapshot s0 = {};
  uint8_t     step = 0;
  uint32_t    stepStartMs = 0, stepMs = 1000;
  BusSnapshot stepSnap = {};
  uint32_t    answered[MARCH_STEPS] = {};
  uint32_t    unanswered[MARCH_STEPS] = {};
  String      report;
};

RateState R;
TestState T;

__attribute__((format(printf, 3, 4)))
void addf(String& s, const char* tag, const char* fmt, ...) {
  char buf[220];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  s += tag;
  s += ' ';
  s += buf;
  s += '\n';
}

const char* pollStyle(uint32_t single, uint32_t burst) {
  if (!single && !burst) return "none";
  if (!single) return "burst";
  if (!burst) return "per-register";
  return "mixed";
}

void updateRates(uint32_t now) {
  if (!R.init) { bus::snapshot(R.prev); R.prevMs = now; R.init = true; return; }
  const uint32_t dtMs = now - R.prevMs;
  if (dtMs < 1000) return;

  BusSnapshot cur;
  bus::snapshot(cur);
  R.rates.cePerSec = (float)(cur.c.ceRises - R.prev.c.ceRises) * 1000.0f / (float)dtMs;
  const uint32_t dSpan = cur.c.spanUsSum - R.prev.c.spanUsSum;
  const uint32_t dPer  = cur.c.spanPeriods - R.prev.c.spanPeriods;
  if (dPer && dSpan) R.rates.sclkHz = 1e6f * (float)dPer / (float)dSpan;   // keeps last value between polls
  R.rates.pollGapMs = cur.pollGapUs / 1000;

  R.singleHist[R.histIdx] = cur.c.singleReads - R.prev.c.singleReads;
  R.burstHist[R.histIdx]  = cur.c.burstReads - R.prev.c.burstReads;
  R.histIdx = (uint8_t)((R.histIdx + 1) % STYLE_WINDOWS);
  uint32_t s = 0, b = 0;
  for (uint8_t i = 0; i < STYLE_WINDOWS; ++i) { s += R.singleHist[i]; b += R.burstHist[i]; }
  R.rates.pollStyle = pollStyle(s, b);

  R.prev = cur;
  R.prevMs = now;
}

// ---- GPIO activity scan -------------------------------------------------------
// Borrows every usable pin on the C3 (GPIO0-10) with an any-edge interrupt for a few
// seconds. Finds bus traffic even if it is wired to a different pin than expected, and
// proves whether anything on the board is switching at all.
constexpr uint8_t SCAN_LAST = 10;
volatile uint32_t g_scanEdges[SCAN_LAST + 1];

void IRAM_ATTR scanIsr(void* arg) {
  const uint32_t p = (uint32_t)(uintptr_t)arg;
  if (p <= SCAN_LAST) g_scanEdges[p] = g_scanEdges[p] + 1;
}

void scanAttach() {
  gpio_intr_disable((gpio_num_t)PIN_CE);
  gpio_isr_handler_remove((gpio_num_t)PIN_CE);
  for (uint8_t p = 0; p <= SCAN_LAST; ++p) {
    g_scanEdges[p] = 0;
    gpio_config_t c = {};
    c.pin_bit_mask = 1ULL << p;
    c.mode = GPIO_MODE_INPUT;
    c.pull_up_en = GPIO_PULLUP_DISABLE;
    c.pull_down_en = GPIO_PULLDOWN_DISABLE;
    c.intr_type = GPIO_INTR_ANYEDGE;
    if (gpio_config(&c) != ESP_OK) continue;
    gpio_isr_handler_add((gpio_num_t)p, scanIsr, (void*)(uintptr_t)p);
    gpio_intr_enable((gpio_num_t)p);
  }
}

void finishPinScan() {
  const uint32_t levels = REG_READ(GPIO_IN_REG);
  uint32_t edges[SCAN_LAST + 1];
  for (uint8_t p = 0; p <= SCAN_LAST; ++p) {
    edges[p] = g_scanEdges[p];
    gpio_intr_disable((gpio_num_t)p);
    gpio_isr_handler_remove((gpio_num_t)p);
  }
  bus::begin();                       // restore the normal tap on CE/SCLK/IO

  unsigned long total = 0;
  for (uint8_t p = 0; p <= SCAN_LAST; ++p) total += edges[p];

  String r = total ? "RESULT: PASS\n" : "RESULT: FAIL\n";
  addf(r, "INFO", "Watched GPIO0-%u for %.1f s with any-edge interrupts",
       (unsigned)SCAN_LAST, (double)(T.durMs / 1000.0f));
  for (uint8_t p = 0; p <= SCAN_LAST; ++p) {
    const char* who = (p == PIN_CE)   ? "  <- CE, DS1302 pin 5"
                    : (p == PIN_SCLK) ? "  <- SCLK, DS1302 pin 7"
                    : (p == PIN_IO)   ? "  <- I/O, DS1302 pin 6" : "";
    addf(r, edges[p] ? "PASS" : "INFO", "GPIO%-2u edges=%-8lu level=%u%s",
         (unsigned)p, (unsigned long)edges[p], (unsigned)((levels >> p) & 1u), who);
  }
  if (!total)
    addf(r, "FAIL", "No edges on ANY pin. Nothing reaching this board is switching: either the clock MCU does not read the DS1302 while running, or no wire carries the signal to the ESP32.");
  else if (!edges[PIN_CE])
    addf(r, "FAIL", "Edges exist, but not on GPIO%u. The CE wire is landing on a different pin: use the pin above that shows edges.", (unsigned)PIN_CE);
  T.report = r;
}

void finishPassive() {
  BusSnapshot s1;
  bus::snapshot(s1);
  const BusCounters& a = T.s0.c;
  const BusCounters& b = s1.c;
  const unsigned long ce      = b.ceRises - a.ceRises;
  const unsigned long single  = b.singleReads - a.singleReads;
  const unsigned long burst   = b.burstReads - a.burstReads;
  const unsigned long writes  = b.writes - a.writes;
  const unsigned long invalid = b.invalid - a.invalid;
  const unsigned long late    = b.misaligned - a.misaligned;
  const unsigned long lateEdg = b.lateEdge - a.lateEdge;
  const unsigned long glitch  = b.glitches - a.glitches;
  const unsigned long slow    = b.tooSlow - a.tooSlow;
  const unsigned long tmo     = b.timeouts - a.timeouts;
  const unsigned long lng     = b.longTxn - a.longTxn;
  const unsigned long inc     = b.incomplete - a.incomplete;
  const unsigned long spoofed = b.spoofed - a.spoofed;
  const unsigned long pass    = b.passReads - a.passReads;
  const unsigned long valid   = single + burst + writes;
  const float secs = (float)T.durMs / 1000.0f;

  String r;
  bool ok = true;
  addf(r, "INFO", "Window %.1f s: %lu CE rises (%.1f/s)", secs, ce, (float)ce / secs);
  if (ce == 0) {
    ok = false;
    addf(r, "FAIL", "No CE rising edges. Check DS1302 pin 5 -> GPIO%d, common GND, level shifting.", PIN_CE);
  } else {
    addf(r, "PASS", "CE is toggling");
    if (valid == 0) {
      ok = false;
      addf(r, "FAIL", "CE toggles but no valid command byte decoded. Check SCLK (pin 7 -> GPIO%d) and I/O (pin 6 -> GPIO%d).",
           PIN_SCLK, PIN_IO);
    } else {
      addf(r, "PASS", "%lu valid command bytes: %lu single reads, %lu burst reads, %lu writes",
           valid, single, burst, writes);
    }
  }
  if (invalid)       addf(r, "WARN", "%lu command bytes with bit7=0 (noise, wrong pad, or ISR entered mid-transfer)", invalid);
  if (late || glitch) addf(r, "WARN", "ISR entered late: %lu with SCLK already high, %lu with CE already low", late, glitch);
  if (lateEdg) {
    addf(r, "WARN", "%lu of %lu transfers had an SCLK edge before the ISR ran; those are never answered", lateEdg, ce);
    if (lateEdg * 10 >= ce * 9)
      addf(r, "WARN", "That is nearly every transfer: SCLK is likely clocked while CE is low (shared line). If so, set BUS_SLIP_GUARD 0.");
  }
  if (slow || tmo || lng) addf(r, "WARN", "Not followed: %lu over ISR budget, %lu timed out, %lu RAM bursts", slow, tmo, lng);
  if (inc)           addf(r, "INFO", "%lu transfers ended before all data bits were clocked", inc);

  String cmds;
  char d[48];
  for (int c = 0; c < 256; ++c) {
    if (s1.cmdSeen[c >> 5] & (1UL << (c & 31))) {
      ds1302::describeCmd((uint8_t)c, d, sizeof d);
      cmds += "       ";
      cmds += d;
      cmds += '\n';
    }
  }
  if (cmds.length()) { r += "INFO Command bytes seen:\n"; r += cmds; }

  addf(r, "INFO", "Polling: %s, cycle gap %lu ms", pollStyle(single, burst), (unsigned long)(s1.pollGapUs / 1000));
  const uint32_t dSpan = b.spanUsSum - a.spanUsSum;
  const uint32_t dPer  = b.spanPeriods - a.spanPeriods;
  if (dPer && dSpan)
    addf(r, "INFO", "SCLK average %.0f Hz (%.1f us per clock)", 1e6f * (float)dPer / (float)dSpan, (float)dSpan / (float)dPer);

  if (spoofed) {
    addf(r, "INFO", "Spoof answered %lu reads in this window; turn spoof off to capture the real chip's data", spoofed);
  } else if (pass && s1.realMask == 0x7F && s1.realUs != T.s0.realUs) {
    char t[48];
    ds1302::formatImage(s1.real, t, sizeof t);
    if (ds1302::imageLooksValid(s1.real)) {
      addf(r, "PASS", "I/O data decodes as a valid time: %s (compare with the display)", t);
    } else {
      ok = false;
      addf(r, "FAIL", "Captured clock bytes are not valid BCD [%02X %02X %02X %02X %02X %02X %02X]: I/O wire or sampling",
           s1.real[0], s1.real[1], s1.real[2], s1.real[3], s1.real[4], s1.real[5], s1.real[6]);
    }
  } else if (valid) {
    addf(r, "WARN", "No complete clock read captured (registers 0-6): data line not verified");
  }

  addf(r, "INFO", "MCU hour format: %s", s1.learnedHour < 0 ? "unknown" : (s1.learnedHour ? "12h" : "24h"));
  T.report = String("RESULT: ") + (ok ? "PASS" : "FAIL") + "\n" + r;
}

void finishMarch() {
  String r;
  bool ok = true;
  addf(r, "INFO", "Step length %.1f s (MCU polling gap %lu ms)", (float)T.stepMs / 1000.0f,
       (unsigned long)(T.stepSnap.pollGapUs / 1000));
  for (uint8_t i = 0; i < MARCH_STEPS; ++i) {
    const unsigned long ans = T.answered[i];
    const unsigned long un  = T.unanswered[i];
    if (ans == 0) {
      ok = false;
      addf(r, "FAIL", "%02u:00  no read answered (%lu reads left to the real chip)", i + 1u, un);
    } else {
      addf(r, "PASS", "%02u:00  answered %lu reads, %lu not answered", i + 1u, ans, un);
    }
  }
  if (ok) addf(r, "INFO", "Bus side OK. The ESP32 cannot see the display: confirm the digits walked 01:00 -> 05:00.");
  else    addf(r, "INFO", "Run the passive bus check: unanswered reads show up there as late ISR entry, over budget or unsupported commands.");
  addf(r, "INFO", "Reverted to normal operation (NTP time if spoof is on and synced, otherwise passthrough).");
  T.report = String("RESULT: ") + (ok ? "PASS (bus side)" : "FAIL") + "\n" + r;
}

void runMarch(uint32_t now) {
  if (now - T.stepStartMs < T.stepMs) return;
  BusSnapshot cur;
  bus::snapshot(cur);
  const uint32_t reads = (cur.c.singleReads + cur.c.burstReads) - (T.stepSnap.c.singleReads + T.stepSnap.c.burstReads);
  const uint32_t sp = cur.c.spoofed - T.stepSnap.c.spoofed;
  T.answered[T.step] = sp;
  T.unanswered[T.step] = reads - sp;
  T.stepSnap = cur;
  if (++T.step < MARCH_STEPS) {
    spoof::setForced(T.step + 1, 0);
    T.stepStartMs = now;
    return;
  }
  spoof::setForced(-1, 0);
  finishMarch();
  T.running = false;
  T.endMs = now;
}

}  // namespace impl

using namespace impl;

void begin() { T.report = "RESULT: none\n"; }

void loop() {
  const uint32_t now = millis();
  updateRates(now);
  if (!T.running) return;
  if (T.kind == Kind::March) { runMarch(now); return; }
  if (now - T.startMs >= T.durMs) {
    if (T.kind == Kind::PinScan) finishPinScan(); else finishPassive();
    T.running = false;
    T.endMs = now;
  }
}

void getRates(Rates& out) { out = R.rates; }

bool startPassiveCheck() {
  if (T.running) return false;
  BusSnapshot cur;
  bus::snapshot(cur);
  T.durMs = std::min<uint32_t>(std::max<uint32_t>(3000, (cur.pollGapUs / 1000) * 3), 10000);
  bus::clearCmdSeen();
  bus::snapshot(T.s0);
  T.kind = Kind::Passive;
  T.startMs = millis();
  T.report = "RESULT: running\n";
  T.running = true;
  return true;
}

bool startMarch() {
  if (T.running) return false;
  bus::snapshot(T.stepSnap);
  const uint32_t gapMs = T.stepSnap.pollGapUs / 1000;
  T.stepMs = std::min<uint32_t>(std::max<uint32_t>(1000, gapMs + gapMs / 2), 5000);
  T.durMs = T.stepMs * MARCH_STEPS;
  for (uint8_t i = 0; i < MARCH_STEPS; ++i) { T.answered[i] = 0; T.unanswered[i] = 0; }
  T.step = 0;
  T.kind = Kind::March;
  T.startMs = T.stepStartMs = millis();
  T.report = "RESULT: running\n";
  T.running = true;
  spoof::setForced(1, 0);
  return true;
}

bool startPinScan() {
  if (T.running) return false;
  T.durMs = 5000;
  scanAttach();
  T.kind = Kind::PinScan;
  T.startMs = millis();
  T.report = "RESULT: running\n";
  T.running = true;
  return true;
}

// Synchronous: drives PIN_CE from the ESP32 itself and checks the CE interrupt fired.
// Current into the outside world is limited by the series resistor on the wire.
bool startIsrSelfTest() {
  if (T.running) return false;
  BusSnapshot s0;
  bus::snapshot(s0);
  const unsigned long before = s0.c.ceRises;
  const uint8_t pulses = 3;

  gpio_set_level((gpio_num_t)PIN_CE, 0);
  gpio_set_direction((gpio_num_t)PIN_CE, GPIO_MODE_INPUT_OUTPUT);
  for (uint8_t i = 0; i < pulses; ++i) {
    gpio_set_level((gpio_num_t)PIN_CE, 0);
    delayMicroseconds(300);
    gpio_set_level((gpio_num_t)PIN_CE, 1);
    delayMicroseconds(300);
    gpio_set_level((gpio_num_t)PIN_CE, 0);
    delayMicroseconds(300);
  }
  gpio_set_direction((gpio_num_t)PIN_CE, GPIO_MODE_INPUT);

  BusSnapshot s1;
  bus::snapshot(s1);
  const unsigned long got = s1.c.ceRises - before;

  String r = (got >= pulses) ? "RESULT: PASS\n" : "RESULT: FAIL\n";
  addf(r, "INFO", "Generated %u rising edges on GPIO%u from the ESP32 itself",
       (unsigned)pulses, (unsigned)PIN_CE);
  addf(r, got >= pulses ? "PASS" : "FAIL", "CE interrupt fired %lu time(s)", got);
  if (got >= pulses)
    addf(r, "INFO", "GPIO config, interrupt and ISR all work. Zero edges during normal running is therefore a missing signal, not a firmware fault.");
  else
    addf(r, "FAIL", "Interrupt did not fire on a self-generated edge: that is a firmware/GPIO fault.");
  addf(r, "INFO", "Timeout and invalid counters tick up from this test: the edges had no SCLK behind them. Expected.");
  T.report = r;
  T.kind = Kind::IsrSelf;
  T.running = false;
  T.startMs = T.endMs = millis();
  T.durMs = 0;
  return true;
}

bool testRunning() { return T.running; }

const char* testName() {
  switch (T.kind) {
    case Kind::Passive: return "passive";
    case Kind::March:   return "march";
    case Kind::PinScan: return "gpio scan";
    case Kind::IsrSelf: return "isr self-test";
    default:            return "";
  }
}

uint32_t testElapsedMs() {
  if (T.kind == Kind::None) return 0;
  return (T.running ? millis() : T.endMs) - T.startMs;
}

uint32_t testDurationMs() { return T.durMs; }

const String& testReport() { return T.report; }

}  // namespace diag

// ================================================================ WI-FI MANAGER
//
// The device never stops trying to reconnect. Retries back off from WIFI_RETRY_MIN_MS to
// WIFI_RETRY_MAX_MS, and every WIFI_RADIO_RESET_TRIES failures the radio is taken down and
// brought back up, which clears driver states that a plain begin() does not.

namespace wifimgr {

struct Stats {
  uint32_t connects;
  uint32_t disconnects;
  uint32_t retries;
  uint32_t radioResets;
  uint8_t  lastReason;
  uint32_t downSinceMs;     // millis() when the link was last lost, 0 while connected
};

namespace impl {

Stats    stats_ = {};
uint32_t lastAttemptMs_ = 0;
uint32_t retryDelayMs_ = WIFI_RETRY_MIN_MS;
bool     txApplied_ = false;
uint32_t failStreak_ = 0;
volatile bool gotIp_ = false;

void onEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      ++stats_.connects;
      stats_.downSinceMs = 0;
      failStreak_ = 0;
      retryDelayMs_ = WIFI_RETRY_MIN_MS;
      gotIp_ = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      ++stats_.disconnects;
      stats_.lastReason = info.wifi_sta_disconnected.reason;
      if (!stats_.downSinceMs) {
        const uint32_t t = millis();
        stats_.downSinceMs = t ? t : 1;
      }
      break;
    default:
      break;
  }
}

}  // namespace impl

const char* reasonName(uint8_t reason) {
  switch (reason) {
    case 0:   return "none";
    case 1:   return "unspecified";
    case 2:   return "auth expired";
    case 4:   return "association expired";
    case 8:   return "AP dropped us";
    case 15:  return "4-way handshake timeout (check the password)";
    case 200: return "beacon timeout";
    case 201: return "no AP found";
    case 202: return "auth failed";
    case 203: return "association failed";
    case 204: return "handshake timeout";
    case 205: return "connection failed";
    default:  return "see esp_wifi_types.h";
  }
}

// Both knobs revert to their defaults whenever the radio restarts, so this runs after every
// STA start, not once at boot. setTxPower needs the station already started or it refuses.
void applyRadioPower() {
  const wifi_ps_type_t ps = g_settings.wifiPsMode == 2 ? WIFI_PS_MAX_MODEM
                          : g_settings.wifiPsMode == 1 ? WIFI_PS_MIN_MODEM
                                                       : WIFI_PS_NONE;
  WiFi.setSleep(ps);
  esp_wifi_set_ps(ps);
  // 0.25 dBm units. quantiseTxDbm has already snapped the value to a producible step.
  const int8_t q = (int8_t)(g_settings.wifiTxDbm * 4);
  impl::txApplied_ = WiFi.setTxPower((wifi_power_t)q);
}

void begin() {
  WiFi.persistent(false);                 // credentials come from the configuration block
  WiFi.setHostname(DEVICE_HOSTNAME);      // must precede WiFi.mode()
  WiFi.onEvent(impl::onEvent);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  applyRadioPower();
  const uint32_t t = millis();
  impl::lastAttemptMs_ = t;
  impl::stats_.downSinceMs = t ? t : 1;
}

void loop() {
  const uint32_t now = millis();
  if (WiFi.isConnected()) {
    impl::retryDelayMs_ = WIFI_RETRY_MIN_MS;
    impl::failStreak_ = 0;
    return;
  }
  if (now - impl::lastAttemptMs_ < impl::retryDelayMs_) return;
  impl::lastAttemptMs_ = now;
  ++impl::stats_.retries;
  ++impl::failStreak_;

  if (impl::failStreak_ % WIFI_RADIO_RESET_TRIES == 0) {
    ++impl::stats_.radioResets;
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_STA);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  applyRadioPower();

  impl::retryDelayMs_ *= 2;
  if (impl::retryDelayMs_ > WIFI_RETRY_MAX_MS) impl::retryDelayMs_ = WIFI_RETRY_MAX_MS;
}

// What the radio reports back, not what we asked for. 0 when the station is not started.
int8_t actualTxDbm() {
  if (!WiFi.isConnected()) return 0;
  const int v = (int)WiFi.getTxPower();
  return (int8_t)(v / 4);
}
bool txApplied() { return impl::txApplied_; }

// True once per new IP; clears the flag.
bool takeGotIp() {
  if (!impl::gotIp_) return false;
  impl::gotIp_ = false;
  return true;
}

void getStats(Stats& out) { out = impl::stats_; }
uint32_t retryDelayMs() { return impl::retryDelayMs_; }

uint32_t downSeconds() {
  if (WiFi.isConnected() || !impl::stats_.downSinceMs) return 0;
  return (millis() - impl::stats_.downSinceMs) / 1000;
}

}  // namespace wifimgr

// ================================================================ WEB PAGE

// Web UI page, gzipped, stored as a byte array (not a raw string) and kept in the .ino.
// A multi-line raw string in a .ino gets #line directives injected by the Arduino sketch
// preprocessor, which corrupts the served HTML. A byte array is immune. Regenerate from
// web_page.h if the page changes.
static const uint8_t INDEX_HTML_GZ[] PROGMEM = {
  31,139,8,0,122,35,166,106,2,255,197,125,253,119,218,70,179,255,239,254,
  43,182,180,79,2,13,200,128,95,226,128,193,215,113,220,39,105,227,196,39,
  38,77,159,246,244,18,129,22,80,172,23,170,21,198,142,195,253,219,239,103,
  102,87,66,2,65,156,220,239,57,223,147,22,208,106,102,118,118,118,118,94,
  246,205,199,63,56,225,48,190,155,74,49,137,125,175,187,115,76,95,194,179,
  131,113,167,36,131,82,247,120,34,109,167,123,236,203,216,22,195,137,29,41,
  25,119,74,179,120,84,59,42,153,210,192,246,101,167,116,227,202,249,52,140,
  226,146,24,134,65,44,3,64,205,93,39,158,116,28,121,227,14,101,141,31,
  170,110,224,198,174,237,213,212,208,246,100,167,81,66,125,177,27,123,178,251,
  166,119,41,206,188,112,120,45,46,66,231,120,87,23,238,28,171,248,142,190,
  91,81,24,198,247,181,218,96,220,250,113,180,143,127,141,118,173,54,180,35,
  7,143,163,17,126,143,240,162,49,196,63,27,15,254,44,110,253,120,104,227,
  223,62,158,60,55,144,173,31,101,93,214,29,122,25,94,3,210,121,106,239,
  61,195,195,220,142,130,214,143,207,246,15,155,245,58,30,7,54,8,14,246,
  155,123,141,35,60,217,195,33,129,238,75,231,104,177,243,95,190,116,92,187,
  60,141,228,72,70,170,54,12,189,48,66,43,38,210,151,45,199,142,174,43,
  247,89,30,27,123,248,215,76,121,108,56,248,55,48,108,202,1,254,29,37,
  108,62,27,224,223,65,202,230,94,115,175,217,148,134,205,125,219,145,71,245,
  148,205,209,96,48,106,238,39,108,142,142,158,54,158,54,18,54,159,218,118,
  115,52,90,44,118,126,190,31,132,183,53,229,126,118,131,113,107,16,70,142,
  140,106,40,89,12,66,231,238,222,183,163,177,27,180,234,237,17,186,168,213,
  216,159,222,238,54,172,253,3,161,238,84,44,253,218,204,173,214,236,233,212,
  147,53,93,80,189,146,227,80,138,247,175,170,202,14,84,77,201,200,29,181,
  7,246,240,122,28,133,179,192,105,221,216,81,153,218,91,105,179,56,204,243,
  104,92,89,236,248,182,27,160,186,91,221,237,173,163,163,250,244,182,157,84,
  47,236,89,28,182,167,182,227,16,147,141,195,233,237,98,135,180,76,70,247,
  142,171,166,158,125,215,26,121,242,182,77,31,181,121,100,79,91,244,209,182,
  61,119,28,212,92,48,166,90,67,40,152,140,218,99,188,3,186,160,166,36,
  228,241,83,212,133,161,218,184,167,166,146,60,100,171,241,52,195,195,226,199,
  33,105,27,191,110,29,214,235,162,73,116,102,110,205,15,131,80,77,237,161,
  172,94,200,192,11,171,103,97,160,66,207,86,213,244,197,98,199,162,110,189,
  95,147,4,149,86,218,90,232,173,6,200,1,209,117,132,126,73,253,155,188,
  172,69,182,227,206,84,171,65,82,73,229,64,140,19,219,134,71,116,91,28,
  135,62,151,163,37,205,108,75,154,128,138,229,109,92,139,35,244,204,40,140,
  252,214,108,58,149,209,208,86,178,237,201,24,178,169,17,175,68,215,170,63,
  149,126,174,135,160,120,149,101,103,212,197,17,85,16,219,3,79,222,235,238,
  106,212,235,255,74,88,5,162,103,79,149,108,37,63,22,177,115,159,240,124,
  64,178,110,223,200,40,118,49,160,107,220,67,173,56,156,38,200,248,89,44,
  8,212,23,181,70,110,164,226,218,112,226,122,142,0,209,12,78,29,149,100,
  95,223,175,177,175,25,221,59,252,87,34,191,90,228,142,39,49,139,6,29,
  68,157,165,5,54,178,125,215,187,107,61,168,103,219,33,154,50,242,194,185,
  214,58,59,184,155,79,100,68,29,142,33,55,150,105,179,81,135,120,134,46,
  200,247,230,51,42,90,233,36,126,156,75,102,13,90,182,174,28,195,89,20,
  65,151,207,168,125,168,39,12,114,77,13,175,43,11,43,28,141,214,218,191,
  176,200,38,228,138,169,0,229,224,52,87,140,103,72,219,179,7,210,75,71,
  215,128,84,63,209,0,82,66,40,1,141,160,181,90,118,220,96,58,139,255,
  34,247,208,33,125,251,187,154,41,8,102,254,64,70,127,87,149,244,228,48,
  206,170,78,34,167,163,84,70,15,26,14,164,251,15,49,47,218,124,185,1,
  250,198,141,105,52,78,174,243,134,131,204,2,213,93,96,49,86,73,153,177,
  198,138,202,3,205,138,194,249,58,53,30,169,95,183,71,89,106,90,17,199,
  145,235,164,228,232,161,77,31,53,32,161,36,150,52,188,102,126,0,83,48,
  138,4,254,231,202,18,235,101,124,206,210,140,194,77,77,111,43,247,154,232,
  70,58,240,2,131,25,76,71,112,159,149,83,182,83,180,189,44,16,126,113,
  95,193,195,84,214,59,134,75,181,56,217,9,67,147,21,126,79,67,151,68,
  145,240,96,41,57,204,90,74,54,88,83,155,116,62,215,23,68,109,161,81,
  90,16,23,25,35,231,62,36,19,22,223,181,224,163,18,242,142,28,217,51,
  15,189,14,79,124,63,159,64,250,108,232,100,11,207,220,55,197,26,244,141,
  90,152,218,228,140,219,50,227,36,113,158,77,118,158,7,15,117,25,14,34,
  37,215,83,66,205,124,208,187,187,207,75,107,147,121,214,154,132,138,23,86,
  16,198,242,126,147,21,63,202,176,102,172,207,30,171,31,98,177,224,62,99,
  234,185,227,147,214,61,35,77,104,174,105,194,81,222,140,237,21,41,198,138,
  217,162,106,44,25,69,235,150,71,191,218,96,172,118,142,119,117,128,119,188,
  171,163,76,138,82,16,85,34,120,160,64,148,67,2,4,160,141,124,116,40,
  142,33,211,64,184,14,130,78,25,33,222,132,168,85,167,4,51,89,18,76,
  174,83,202,154,221,253,250,170,92,16,184,238,18,9,170,182,65,65,102,66,
  142,67,130,82,183,86,107,241,127,9,84,250,158,189,64,90,33,63,9,170,
  182,107,89,214,146,164,102,123,231,216,113,111,52,85,72,32,69,210,15,19,
  215,113,36,1,3,166,187,3,14,96,64,221,48,72,129,16,72,80,228,221,
  236,94,197,118,60,83,32,218,236,30,179,135,166,128,25,50,137,157,238,235,
  16,46,87,196,174,47,17,46,59,84,196,181,197,92,158,214,71,42,136,22,
  105,144,221,56,90,226,247,128,41,62,135,193,10,122,239,243,3,112,63,184,
  181,95,220,60,222,7,119,228,22,3,83,231,33,118,68,95,229,49,222,196,
  211,7,84,245,14,74,25,230,17,185,232,1,168,175,109,21,11,174,253,46,
  24,230,73,92,161,164,24,233,66,218,106,22,73,71,56,136,118,227,60,214,
  11,42,218,82,87,36,145,29,173,240,74,69,155,106,242,195,232,46,15,142,
  178,135,116,29,172,190,8,103,49,92,241,74,231,225,197,3,240,175,166,97,
  56,90,17,8,21,109,81,20,133,209,46,226,80,196,19,41,46,206,222,231,
  145,95,249,118,102,92,20,212,187,171,117,23,99,68,43,250,118,157,71,252,
  10,243,100,180,158,130,91,174,101,84,50,99,202,32,144,255,51,69,248,228,
  0,71,0,184,83,10,160,87,57,165,227,119,128,225,216,69,112,236,82,162,
  104,166,196,116,9,92,192,199,122,50,24,35,93,45,29,238,149,56,65,25,
  134,112,173,50,150,137,101,153,74,207,67,190,55,188,6,43,182,167,36,213,
  173,199,239,58,11,238,13,134,32,243,48,69,212,44,216,198,163,72,148,209,
  232,48,112,84,165,152,41,29,81,105,182,152,132,240,221,160,83,106,28,48,
  131,157,210,209,33,140,25,25,57,57,69,105,134,1,243,53,77,100,67,206,
  162,212,237,161,175,136,135,203,48,244,132,173,174,21,49,39,130,80,64,239,
  36,122,18,86,109,20,206,34,129,194,145,123,35,197,63,51,4,10,82,9,
  148,79,80,110,137,87,134,111,37,224,77,101,36,236,1,148,78,60,67,182,
  164,118,224,194,197,64,66,233,181,11,115,132,29,11,91,120,108,148,150,194,
  23,101,120,98,16,169,138,55,167,87,85,84,19,201,185,237,121,21,17,217,
  208,164,72,51,97,139,233,108,224,185,67,144,10,61,24,210,105,94,152,241,
  103,244,103,198,90,105,201,29,235,176,83,43,32,65,144,114,81,201,3,181,
  36,254,92,234,94,190,189,122,245,135,232,253,41,162,153,39,191,166,39,64,
  248,127,173,38,142,130,105,120,97,223,121,228,168,132,178,111,160,246,75,54,
  50,13,100,192,157,227,112,202,35,6,61,50,67,125,84,123,169,251,11,20,
  44,156,243,184,36,241,112,83,68,121,8,185,194,57,197,115,119,40,33,223,
  59,105,71,208,57,141,191,70,72,197,118,224,240,240,123,59,26,181,68,242,
  200,190,69,160,183,24,125,35,182,99,248,55,216,201,227,70,236,101,55,229,
  149,55,35,151,73,52,242,65,239,37,148,80,193,168,142,93,104,124,68,175,
  124,40,217,154,37,42,144,151,33,80,40,176,83,124,182,64,44,21,27,136,
  136,114,115,159,117,30,138,30,187,30,234,144,193,82,94,43,100,154,251,147,
  82,215,192,111,130,105,52,1,211,104,174,192,124,163,134,222,58,3,191,164,
  125,173,224,184,217,119,105,184,205,179,54,45,59,10,52,252,106,255,52,235,
  224,182,46,156,231,190,168,9,10,241,66,97,130,232,42,216,139,99,140,225,
  141,125,219,56,66,43,142,8,119,51,200,33,64,14,183,131,28,0,228,96,
  59,200,62,64,246,183,131,236,1,100,111,59,8,108,98,163,97,154,250,12,
  223,136,83,209,74,109,182,26,187,71,220,221,70,128,27,72,160,193,219,219,
  251,180,212,125,186,21,0,141,221,222,214,38,250,195,240,8,243,238,250,51,
  159,249,210,93,131,60,72,78,99,181,109,188,172,41,202,84,249,161,35,19,
  77,161,223,190,80,158,148,211,66,53,73,160,87,217,170,243,0,78,117,196,
  246,230,246,157,18,97,176,89,218,165,238,133,27,0,1,233,220,53,124,65,
  60,199,168,17,47,122,175,46,240,96,195,207,169,109,34,184,176,111,129,234,
  133,48,83,112,34,204,110,21,95,232,28,197,195,27,234,158,239,167,77,118,
  99,221,233,41,216,29,118,112,100,0,67,225,73,56,34,37,230,110,60,17,
  242,198,69,224,61,36,215,53,113,3,135,96,124,246,138,4,60,8,201,234,
  69,179,32,128,29,166,177,97,237,244,254,208,250,34,134,118,0,250,96,108,
  54,156,136,193,76,145,121,3,148,69,9,73,34,110,116,39,12,31,90,1,
  238,71,100,177,226,89,196,164,48,108,209,246,185,29,15,39,38,219,200,133,
  74,148,192,11,53,244,174,133,68,70,97,18,137,170,40,128,164,249,41,212,
  14,168,216,128,65,78,78,17,77,178,190,208,122,149,128,161,163,158,131,107,
  199,181,199,200,73,99,119,168,32,125,52,250,78,248,51,146,127,108,223,145,
  7,255,44,163,208,18,103,51,142,192,68,239,143,29,221,122,56,178,144,198,
  146,130,84,230,98,14,23,199,2,35,101,85,74,71,0,2,121,15,164,76,
  197,156,70,85,197,224,142,159,148,13,63,160,35,27,36,75,80,252,156,135,
  79,162,191,9,242,174,156,227,101,31,58,8,111,181,243,85,58,60,229,40,
  85,200,128,167,7,68,25,30,87,116,160,126,74,197,19,196,25,227,73,75,
  143,36,137,24,228,197,85,99,175,222,52,106,148,13,184,190,169,222,65,24,
  198,67,152,231,231,248,22,195,16,222,193,9,231,1,245,38,85,100,230,117,
  68,185,94,111,213,235,85,81,223,211,95,77,253,213,192,87,166,226,140,205,
  143,194,57,170,213,211,29,166,94,53,27,192,188,155,214,218,55,208,228,43,
  124,30,239,106,160,76,18,58,242,213,56,147,193,154,177,64,206,241,129,241,
  245,219,25,26,65,42,143,100,65,34,14,67,182,35,70,82,58,28,111,175,
  141,167,243,224,198,141,194,192,39,175,11,93,67,26,238,250,164,175,103,58,
  194,56,179,3,219,177,197,213,135,183,207,107,23,175,171,130,66,16,40,19,
  20,12,193,30,34,17,212,160,159,195,192,218,121,161,253,142,56,251,240,225,
  185,112,21,52,50,242,160,103,36,132,75,23,128,53,173,48,26,92,156,159,
  157,157,61,134,5,130,188,223,218,215,55,174,71,228,144,104,136,153,146,10,
  234,37,135,54,126,17,10,26,224,170,29,4,182,9,170,187,68,177,196,219,
  1,197,162,230,133,98,151,236,221,181,17,21,65,79,104,30,68,132,158,147,
  132,162,240,216,4,19,201,79,144,30,84,76,135,169,59,252,206,244,182,116,
  140,2,175,228,227,191,176,8,51,105,209,156,146,119,185,37,93,12,51,140,
  81,196,145,199,6,219,15,73,229,208,28,207,129,48,86,116,50,79,235,3,
  1,61,52,99,30,73,216,169,60,254,47,84,84,140,114,70,131,2,35,44,
  143,192,165,15,168,240,165,180,167,48,35,81,8,51,82,80,47,189,126,64,
  94,185,49,158,202,122,201,185,138,131,18,79,168,240,168,128,217,78,83,9,
  61,254,185,70,2,202,133,254,153,144,255,233,67,35,254,172,167,206,178,48,
  248,192,60,60,10,6,106,218,78,107,55,102,128,205,13,3,36,45,193,8,
  214,86,32,209,235,165,53,216,88,65,64,150,101,91,5,4,144,171,128,251,
  22,89,225,124,141,122,177,131,77,36,232,33,42,71,184,239,88,62,100,17,
  90,227,161,53,180,119,227,16,241,255,110,70,177,213,174,19,14,119,213,60,
  28,212,110,125,175,111,26,210,39,100,107,168,110,244,72,202,215,160,59,226,
  246,130,204,92,77,3,60,200,174,245,50,246,44,153,28,41,178,104,111,122,
  103,20,160,96,112,75,31,14,213,105,139,127,95,190,122,251,148,38,124,110,
  100,198,131,193,246,156,190,56,131,100,28,50,50,236,95,68,19,145,116,56,
  241,171,58,154,104,90,77,49,251,5,65,193,142,158,112,22,8,173,194,120,
  66,54,133,168,92,126,184,176,196,107,118,160,28,247,192,87,121,250,41,244,
  98,50,100,29,168,33,217,30,75,252,110,74,212,132,204,29,44,16,124,49,
  7,171,182,83,221,49,193,4,153,70,157,160,71,82,65,126,33,25,76,132,
  22,74,155,0,159,194,11,88,53,176,76,61,18,105,195,104,137,43,73,137,
  57,51,80,101,147,183,108,161,24,209,164,245,14,153,117,18,141,169,209,141,
  140,129,123,192,160,66,90,2,125,123,103,207,77,11,203,245,218,210,83,225,
  29,162,60,132,195,141,134,113,87,149,130,33,199,36,86,38,64,120,218,163,
  110,102,61,14,40,172,223,168,241,60,229,181,77,227,115,115,98,122,72,65,
  34,137,130,124,101,64,197,60,189,139,184,88,1,167,252,200,145,227,246,89,
  97,35,216,20,228,27,161,167,105,234,86,195,180,167,214,76,90,68,41,217,
  198,6,189,165,26,183,180,135,222,175,91,136,144,89,252,122,123,38,80,56,
  202,172,161,48,166,57,85,177,71,252,21,182,138,161,55,52,235,32,105,214,
  65,210,172,195,109,205,122,201,21,111,105,23,3,228,26,198,92,98,56,41,
  173,92,155,236,211,67,212,212,161,152,149,122,158,167,223,76,84,178,76,144,
  178,109,214,160,69,10,153,76,195,53,154,153,73,184,108,154,189,222,248,213,
  6,103,3,245,165,177,227,42,223,164,115,118,122,108,162,87,35,152,4,133,
  33,174,131,32,19,174,232,252,130,235,55,195,57,12,5,98,111,239,142,118,
  237,160,113,73,38,70,185,15,51,167,172,213,104,209,24,246,245,14,160,150,
  229,116,139,226,125,216,60,173,42,156,41,177,182,164,125,177,147,31,107,23,
  118,132,64,33,71,34,107,150,125,122,189,1,247,10,76,175,12,84,178,43,
  138,138,69,57,114,105,99,138,96,63,91,201,104,194,186,251,232,50,15,130,
  84,87,65,46,248,201,162,74,146,60,150,53,77,9,96,208,144,225,155,70,
  72,94,148,201,192,19,51,6,171,234,82,238,107,9,234,15,14,50,168,0,
  78,114,231,226,252,244,234,253,187,243,23,58,222,66,33,103,141,166,83,34,
  89,67,250,152,88,100,238,29,90,142,20,200,6,246,171,226,168,42,26,123,
  248,31,223,205,38,254,127,138,161,135,239,189,167,176,227,118,112,199,142,99,
  71,34,130,128,1,71,222,2,146,48,228,236,77,92,29,219,33,214,246,194,
  49,76,52,124,143,158,225,115,44,241,111,154,186,53,250,64,64,163,25,218,
  135,46,83,148,69,195,38,196,30,101,187,104,190,220,153,71,110,108,242,81,
  74,99,218,57,95,71,42,166,117,142,64,60,123,12,39,36,6,238,88,124,
  154,249,83,171,80,210,203,254,161,218,149,168,83,149,92,191,51,139,239,208,
  210,3,161,187,64,203,145,55,116,176,208,61,155,115,78,194,225,201,3,118,
  60,110,176,99,199,156,64,80,198,238,85,5,229,237,119,186,97,169,78,83,
  190,78,238,81,39,121,234,159,25,101,250,115,240,13,71,23,67,222,119,73,
  28,44,105,40,80,93,196,190,150,225,14,149,251,174,227,120,137,132,169,224,
  221,25,101,42,228,48,65,215,119,149,226,148,128,230,201,73,74,90,92,74,
  124,10,7,156,158,216,172,25,38,12,160,126,81,201,156,185,179,99,194,129,
  81,20,250,137,243,38,129,232,216,160,202,4,109,18,130,63,39,166,117,7,
  22,11,246,185,164,201,201,116,216,101,115,77,51,17,65,129,2,168,97,40,
  34,171,87,238,56,96,13,228,6,34,195,119,227,153,195,26,75,147,165,59,
  110,160,72,52,150,248,64,65,203,0,41,215,53,164,84,77,114,101,206,129,
  108,146,64,128,124,69,101,101,157,33,70,162,172,183,180,162,145,152,143,136,
  252,147,35,138,30,196,145,22,140,19,133,83,214,186,58,67,179,230,3,168,
  118,100,9,54,229,182,8,228,24,241,31,244,85,15,51,61,10,129,192,83,
  35,144,179,181,49,30,164,100,245,155,35,194,149,25,143,194,101,212,179,115,
  1,44,218,193,132,96,85,236,138,124,34,227,156,109,72,224,174,206,94,255,
  134,158,150,255,204,100,48,204,103,91,206,203,207,197,56,151,161,231,241,12,
  127,22,152,10,183,36,99,72,54,124,8,51,143,67,111,30,154,205,81,7,
  231,177,223,161,228,161,216,100,49,242,11,196,206,7,42,122,200,194,109,102,
  22,102,109,149,154,152,120,200,34,53,45,1,152,137,255,21,25,227,69,49,
  202,171,171,119,121,208,87,42,218,200,161,35,120,70,98,165,211,127,161,178,
  135,229,157,122,91,7,28,187,222,215,145,166,195,24,147,60,205,136,124,0,
  186,106,94,30,27,79,194,138,21,208,2,69,186,48,154,16,122,112,174,163,
  98,101,210,155,149,121,165,100,15,68,102,7,73,189,180,226,107,47,1,141,
  97,152,247,182,13,75,152,114,158,224,100,87,187,193,87,23,184,249,166,37,
  94,24,19,197,94,94,79,127,137,90,87,212,15,240,99,147,211,135,57,203,
  211,217,131,33,31,210,162,32,188,8,229,101,236,184,109,136,225,198,141,239,
  54,80,161,30,206,17,217,183,4,212,0,214,207,27,213,244,2,71,33,94,
  126,150,66,135,28,146,155,48,200,205,247,109,98,158,54,17,228,195,37,148,
  240,234,103,54,157,206,108,48,137,47,163,112,125,242,110,213,171,34,130,104,
  104,103,217,210,83,199,20,218,209,26,88,160,44,209,204,230,169,212,79,163,
  100,249,150,253,41,91,117,138,21,105,83,166,210,30,254,252,234,114,175,153,
  58,14,41,119,50,254,132,221,134,54,191,186,116,12,111,151,152,225,72,38,
  155,24,104,69,29,238,133,230,230,41,192,185,147,108,170,35,153,211,215,227,
  93,179,115,71,13,17,174,197,221,29,90,3,136,69,239,207,206,95,59,127,
  149,204,76,225,174,120,127,85,170,150,78,125,248,205,161,189,123,21,247,127,
  13,39,129,66,209,155,171,222,94,107,175,254,230,69,175,122,177,103,53,173,
  122,245,162,209,176,26,86,189,244,119,117,35,254,75,219,115,71,72,50,81,
  114,213,219,63,253,38,220,94,24,133,65,28,162,228,252,170,119,112,254,77,
  184,31,220,32,112,167,114,140,162,179,171,222,225,217,55,33,191,147,24,153,
  182,65,221,6,120,238,248,196,97,128,162,139,171,222,211,139,111,170,229,119,
  59,128,6,211,22,169,106,233,242,170,119,116,249,109,13,164,189,117,147,48,
  82,210,84,190,13,248,63,146,86,83,175,3,119,36,191,139,213,87,255,32,
  30,112,227,239,234,137,55,114,222,255,79,24,93,127,23,242,217,4,159,227,
  240,187,122,241,133,12,180,112,191,189,189,151,147,80,6,238,237,3,36,251,
  58,84,253,83,196,137,158,84,223,213,141,167,193,16,157,72,219,115,80,246,
  219,85,239,217,233,111,15,194,191,180,135,238,200,29,238,190,132,243,243,102,
  222,12,69,47,175,122,13,3,251,26,81,92,32,76,29,153,218,46,228,173,
  59,12,251,103,48,213,57,237,222,4,223,115,63,205,108,30,9,219,90,182,
  9,251,223,51,196,171,190,237,217,15,170,235,18,21,249,182,209,146,237,160,
  207,195,113,24,83,193,113,173,126,208,253,10,240,107,215,127,40,232,153,29,
  217,67,91,105,232,253,238,254,118,232,43,59,136,93,173,156,26,28,159,123,
  221,234,197,51,72,230,112,183,185,95,189,216,55,191,182,211,57,141,198,200,
  2,96,112,118,159,35,88,37,125,114,35,105,152,216,235,238,125,141,137,176,
  127,137,112,40,124,32,252,5,29,254,161,69,214,135,34,188,182,65,255,243,
  3,69,242,210,190,177,19,195,121,176,28,173,187,169,194,236,54,190,46,211,
  176,255,34,164,181,219,208,56,141,175,168,205,12,142,52,236,191,131,94,231,
  224,207,103,200,118,104,84,233,31,24,168,129,195,134,250,223,23,189,250,243,
  43,102,237,128,24,2,107,117,250,85,140,246,130,118,62,37,104,175,30,140,
  246,218,85,3,174,237,195,121,175,254,225,252,193,120,23,182,67,83,100,16,
  224,121,175,214,56,91,34,38,104,187,123,197,136,151,118,228,170,239,192,123,
  30,205,144,93,121,223,131,122,234,211,182,31,199,246,191,167,90,25,105,177,
  126,43,226,159,51,244,250,228,59,16,223,133,190,252,14,180,223,93,25,104,
  141,254,214,14,129,81,159,125,79,141,31,236,72,217,243,239,64,188,138,195,
  225,245,36,244,190,167,63,222,42,182,32,223,138,118,134,143,96,2,239,245,
  61,93,249,18,106,231,6,215,46,149,0,181,121,158,25,38,123,41,242,134,
  225,124,74,97,180,250,46,212,231,51,58,39,137,80,249,187,176,127,187,115,
  111,190,11,241,21,237,219,27,204,60,50,165,79,96,119,107,27,196,114,17,
  170,97,56,231,208,227,183,53,152,211,216,35,175,51,68,136,122,119,253,201,
  190,113,175,141,105,210,112,167,35,178,137,136,17,46,244,44,218,185,205,173,
  212,197,112,111,116,110,3,81,167,173,121,168,119,235,248,108,116,171,100,160,
  127,221,59,60,216,109,30,60,132,206,107,56,61,18,253,135,83,116,248,195,
  42,118,163,48,47,180,125,136,234,96,55,209,145,253,212,73,110,39,132,84,
  4,89,146,84,131,89,68,145,253,21,204,125,173,249,16,196,55,196,193,128,
  85,13,76,239,61,4,229,116,56,140,236,111,144,238,169,55,118,101,148,26,
  210,237,40,202,69,99,36,108,175,237,73,26,175,175,168,33,175,180,195,220,
  39,113,28,230,253,196,54,58,207,165,27,205,10,149,121,57,4,31,64,230,
  157,123,103,59,147,85,237,220,134,1,223,104,187,26,97,191,91,219,255,58,
  66,79,78,34,59,48,85,236,213,81,9,242,201,7,52,208,30,79,28,219,
  89,99,13,239,18,144,223,40,114,155,16,51,151,191,245,106,7,5,0,161,
  119,109,115,204,72,162,62,88,214,155,35,18,79,104,50,111,166,107,58,216,
  63,232,2,114,191,128,218,139,137,125,109,6,209,97,183,118,184,14,240,31,
  27,241,139,105,233,33,181,244,176,176,198,231,0,187,14,175,53,220,211,110,
  237,233,58,200,175,168,41,98,198,63,188,122,94,4,112,133,80,201,158,134,
  145,212,84,142,186,181,163,130,166,205,16,134,247,95,207,252,233,44,218,6,
  119,97,7,174,103,226,253,162,247,72,55,198,253,223,240,65,249,198,111,133,
  32,61,27,121,183,171,99,192,162,247,87,52,175,62,177,183,65,200,144,205,
  36,210,161,218,179,130,10,194,235,59,178,38,191,22,191,126,239,217,176,179,
  54,58,123,107,75,79,61,223,230,44,136,122,186,91,164,50,61,91,77,174,
  17,153,111,131,249,205,78,45,250,62,117,243,126,97,55,211,49,44,127,16,
  26,66,4,87,172,128,191,123,182,227,222,132,42,54,42,209,0,100,163,0,
  238,63,18,186,140,240,55,48,118,112,35,123,111,66,80,115,7,110,164,214,
  116,236,237,80,162,175,25,114,166,226,200,246,0,126,137,80,154,44,192,233,
  135,101,199,20,193,189,176,163,57,71,111,167,212,131,207,210,182,20,193,158,
  58,210,179,145,111,100,161,79,57,53,128,105,106,80,148,176,207,137,193,222,
  102,18,207,17,218,14,236,128,73,192,186,165,34,41,130,189,186,115,2,121,
  183,132,60,61,255,166,170,46,164,55,8,103,81,182,174,111,164,240,50,28,
  96,188,126,35,122,146,205,159,206,134,215,180,72,76,179,109,127,18,122,243,
  205,159,132,254,76,199,83,95,193,62,155,192,128,113,56,14,213,105,146,245,
  106,52,97,190,240,176,135,7,67,101,151,138,82,82,169,117,91,167,246,139,
  251,201,53,164,136,208,38,48,36,249,28,112,78,10,59,38,129,186,12,163,
  184,127,1,35,165,6,119,107,170,189,14,222,179,39,110,204,181,3,168,187,
  25,238,116,234,106,51,220,128,91,104,108,20,76,15,6,11,246,96,58,91,
  131,125,223,59,67,89,250,137,138,254,110,155,105,209,159,58,110,167,235,132,
  195,25,109,173,180,198,50,62,135,171,198,207,231,119,175,156,178,91,73,192,
  164,26,118,84,167,123,21,99,56,142,203,170,98,69,52,67,61,148,229,221,
  191,30,29,119,75,127,239,142,171,195,78,183,124,95,122,84,106,149,30,217,
  254,180,77,76,208,111,47,166,159,93,250,57,166,159,143,75,143,241,243,159,
  89,136,135,197,95,195,191,43,105,29,8,176,59,62,42,241,213,113,163,94,
  175,159,248,234,73,73,248,74,208,228,67,11,133,207,234,84,122,129,222,183,
  120,45,179,236,171,93,2,172,0,108,9,133,48,75,30,174,64,29,214,13,
  24,18,110,13,72,165,123,214,161,60,172,88,113,248,139,123,43,157,114,131,
  0,38,252,58,225,200,153,69,212,106,170,250,132,152,81,165,22,42,216,207,
  115,65,228,13,109,208,5,89,212,182,74,21,20,61,25,139,222,249,197,101,
  255,226,244,143,206,65,3,25,242,239,252,115,175,126,208,208,111,105,129,233,
  117,104,59,210,233,240,14,190,234,96,166,238,204,207,145,237,122,16,190,180,
  175,59,245,246,206,104,22,232,21,25,90,127,61,11,131,160,60,244,84,213,
  87,227,202,189,102,124,216,249,169,172,15,121,86,218,238,168,252,131,126,101,
  233,3,159,157,56,154,201,118,36,105,95,120,123,49,180,120,218,255,13,95,
  159,66,40,162,244,4,212,218,67,139,118,30,158,153,91,84,64,160,157,226,
  51,75,237,197,206,146,143,145,235,121,189,207,229,164,122,69,213,211,113,168,
  10,93,3,33,198,157,96,230,121,213,80,127,183,119,68,239,79,11,141,61,
  71,44,83,46,255,53,142,166,85,186,188,165,74,103,134,254,174,116,186,247,
  224,24,133,63,116,58,96,122,220,193,207,54,80,83,29,29,66,10,177,52,
  106,90,46,133,211,152,150,182,169,174,112,108,241,6,23,70,81,150,61,69,
  218,230,156,209,205,13,229,112,92,1,195,66,104,254,194,78,32,231,226,45,
  239,233,47,167,117,51,129,28,18,112,160,157,34,79,42,131,90,58,131,65,
  12,125,62,237,4,37,31,242,83,169,162,113,194,64,175,173,119,202,166,77,
  202,226,245,102,180,43,133,100,49,149,42,250,69,199,0,180,23,192,79,222,
  132,1,111,10,210,68,52,247,55,157,28,154,133,49,233,151,233,16,60,189,
  244,59,36,92,23,122,25,119,186,241,95,205,191,59,157,206,77,165,109,72,
  119,252,19,31,101,173,132,1,84,149,237,198,248,54,40,199,21,226,245,135,
  184,162,21,228,113,110,179,16,239,67,11,104,115,196,93,186,247,255,49,184,
  213,176,226,227,79,247,48,20,101,244,145,239,84,22,199,131,168,187,142,157,
  128,140,104,111,71,101,241,211,189,249,121,82,18,95,4,172,68,9,69,176,
  4,128,192,103,223,7,8,202,9,138,142,38,244,225,35,165,90,232,221,34,
  10,47,104,125,141,94,186,42,234,207,240,98,150,28,53,248,152,107,24,31,
  250,118,63,203,114,84,185,55,188,18,23,81,198,142,253,119,249,221,249,213,
  251,215,189,150,184,60,189,186,178,126,254,66,95,149,221,177,95,93,17,65,
  128,54,36,59,9,31,87,118,68,17,137,95,78,95,189,6,9,250,42,32,
  49,64,148,159,165,145,161,240,225,244,221,155,117,4,58,167,94,234,210,187,
  20,39,215,188,72,210,193,200,178,83,185,103,197,161,245,128,74,110,248,150,
  110,74,79,28,139,14,160,0,92,107,151,62,101,158,7,115,44,90,34,183,
  248,248,228,73,246,193,82,83,207,197,96,19,165,202,95,13,168,79,114,54,
  189,100,52,85,159,246,222,76,236,203,151,146,94,245,139,69,121,110,235,237,
  63,180,98,248,166,119,89,73,104,244,62,175,16,128,54,25,26,241,231,62,
  175,189,202,133,40,167,133,142,138,251,116,134,168,79,56,139,202,71,144,209,
  67,96,142,170,231,240,136,134,46,31,9,175,88,46,50,234,232,101,239,226,
  117,103,206,183,1,240,206,122,88,132,147,143,107,221,155,190,78,142,173,144,
  2,146,190,204,45,165,92,199,104,228,220,138,240,180,224,51,84,75,0,119,
  106,94,235,199,9,194,220,202,66,11,225,35,106,107,125,92,215,4,199,85,
  15,169,145,143,124,160,245,179,8,165,244,208,215,99,3,250,28,221,209,30,
  160,159,238,51,94,9,220,81,121,63,241,145,11,65,195,197,51,123,49,84,
  24,180,210,26,168,176,175,11,89,150,149,197,71,35,58,58,27,95,208,39,
  65,60,181,244,193,90,170,95,239,143,210,140,233,119,201,65,99,98,48,33,
  165,79,203,103,187,225,99,239,15,150,98,124,219,119,6,62,203,145,36,244,
  164,156,20,81,159,35,161,123,244,104,165,224,135,78,82,112,242,81,20,140,
  147,178,62,58,134,65,133,112,76,101,234,48,248,139,74,98,35,96,109,42,
  203,42,233,210,41,87,58,39,165,82,235,177,88,239,166,50,239,156,210,48,
  149,204,248,23,79,62,102,186,107,170,150,74,137,214,39,175,245,166,36,98,
  101,217,219,42,233,61,218,185,197,250,164,127,115,177,105,2,70,140,121,71,
  5,125,93,144,10,149,151,254,179,50,213,242,231,14,165,219,6,78,140,65,
  94,41,134,50,172,219,229,178,54,187,43,160,125,50,194,234,103,163,66,212,
  76,211,253,244,142,119,38,44,248,94,3,149,202,148,148,188,200,122,209,185,
  24,218,151,144,119,28,38,94,152,130,115,62,92,85,13,131,142,154,90,118,
  228,75,71,55,82,223,118,81,201,70,41,250,202,139,210,147,114,24,156,208,
  128,109,113,11,42,121,248,172,214,2,174,12,170,188,35,228,164,196,95,136,
  228,166,86,186,181,226,164,148,254,4,53,230,4,38,170,84,105,149,50,199,
  186,18,67,165,239,95,88,53,118,36,21,190,172,161,15,205,119,181,101,73,
  164,165,203,167,83,191,91,63,41,61,49,78,110,229,85,26,49,54,33,104,
  60,147,165,43,175,192,252,92,183,234,71,135,251,149,28,172,218,117,236,187,
  138,224,67,121,217,81,168,177,72,210,60,14,217,248,248,124,173,4,221,186,
  34,2,41,29,37,14,56,36,14,71,186,19,243,76,105,84,248,229,242,132,
  246,118,195,131,20,211,126,82,170,112,147,210,161,206,151,77,100,213,178,188,
  123,121,250,230,213,217,151,231,239,222,126,120,243,246,125,239,11,111,245,112,
  194,241,174,107,209,110,14,80,101,205,54,86,168,178,110,150,181,203,52,202,
  156,131,93,164,131,249,227,166,96,99,19,70,58,120,121,159,205,143,212,122,
  250,149,232,245,23,49,155,166,18,157,77,201,233,36,146,124,82,206,130,118,
  27,39,143,11,13,145,54,141,145,100,250,236,228,239,244,142,153,250,222,250,
  49,62,58,5,145,218,21,178,76,70,156,116,25,71,206,110,142,34,41,5,
  43,199,68,218,211,62,61,194,198,55,247,179,73,199,66,92,63,103,123,31,
  241,193,86,190,106,43,131,99,202,55,162,233,243,175,82,171,84,130,4,77,
  41,68,72,4,146,165,44,142,197,81,227,89,83,156,136,66,83,58,138,236,
  49,133,239,25,75,42,90,226,241,227,202,210,137,223,146,23,191,253,242,229,
  222,68,195,230,28,91,86,18,63,204,111,45,115,10,243,164,32,78,77,46,
  112,34,223,152,236,119,77,106,163,209,192,232,104,98,223,142,105,95,103,92,
  76,5,118,139,143,132,137,20,42,107,194,136,14,200,208,0,233,155,77,172,
  40,42,10,42,140,131,184,181,204,225,35,136,14,118,253,214,34,162,253,225,
  194,108,178,205,88,81,246,75,183,218,20,203,40,10,163,85,111,199,162,100,
  143,110,88,75,93,122,22,43,51,64,180,183,43,140,65,130,48,217,131,203,
  76,101,42,45,137,26,134,254,58,89,26,241,105,172,173,123,136,206,10,230,
  2,173,91,43,28,168,62,13,28,26,209,134,70,82,84,225,230,145,127,185,
  14,48,12,184,197,104,98,217,132,56,26,144,221,15,68,21,122,14,51,150,
  150,117,241,224,219,183,250,225,164,72,201,174,122,167,175,207,51,227,9,33,
  34,215,177,234,220,181,247,227,19,157,1,115,82,53,71,31,56,98,133,89,
  92,113,247,136,126,141,39,48,7,26,115,150,14,108,205,169,180,79,169,121,
  127,136,236,139,114,222,47,95,214,203,233,50,19,120,25,80,39,225,20,106,
  158,57,196,21,138,233,76,77,86,213,174,72,197,86,106,89,209,42,209,225,
  195,5,25,48,60,178,61,211,170,166,203,6,178,31,200,113,161,178,149,233,
  128,46,109,166,206,106,110,213,28,23,179,105,63,124,186,83,155,195,116,2,
  0,173,62,111,246,62,41,85,245,174,111,242,172,220,31,89,197,52,18,213,
  71,60,215,6,121,126,148,162,3,88,0,169,166,176,90,154,247,208,136,144,
  85,6,223,203,166,49,196,36,142,193,48,162,179,19,50,250,47,123,189,75,
  177,212,246,244,229,34,29,40,140,201,117,135,215,140,194,131,109,28,134,142,
  200,87,29,94,231,106,133,134,125,41,144,94,192,246,84,205,134,67,72,49,
  13,245,31,47,171,162,57,6,8,231,100,13,157,58,87,27,161,127,102,114,
  182,68,109,17,83,1,194,16,29,254,27,150,168,160,239,106,175,159,200,85,
  239,49,93,137,229,211,200,19,195,80,11,143,99,207,240,218,148,241,37,16,
  210,161,50,146,142,160,73,40,253,138,133,69,143,106,25,226,78,233,86,222,
  12,12,63,27,32,188,134,217,243,164,126,195,63,251,250,116,51,191,35,207,
  33,212,181,59,53,212,201,145,208,99,26,233,234,35,184,121,246,19,64,115,
  170,132,140,135,254,73,46,43,255,178,216,207,149,19,95,165,225,205,211,54,
  20,109,65,208,218,90,215,220,173,144,169,138,11,54,58,200,86,41,117,36,
  24,199,105,206,75,167,2,115,246,195,140,26,29,170,22,13,241,44,0,141,
  223,19,58,224,168,207,191,96,84,233,216,118,33,86,224,104,83,238,98,183,
  160,16,226,255,175,149,98,62,154,212,87,11,101,26,91,84,37,106,106,81,
  64,152,125,49,124,82,50,230,38,117,12,232,218,101,142,101,128,233,68,20,
  44,246,122,211,232,197,146,111,13,182,80,91,40,197,212,97,38,220,46,160,
  103,251,3,151,174,11,72,73,26,248,53,171,88,64,154,13,224,250,8,86,
  179,65,141,238,166,40,50,110,162,156,24,55,138,134,69,57,181,132,149,173,
  226,8,175,89,115,181,93,54,101,100,149,197,46,158,147,249,226,133,40,255,
  15,199,97,90,220,55,58,165,207,101,1,191,103,206,234,26,71,103,242,210,
  4,109,58,247,251,147,207,43,168,148,213,93,191,252,188,172,155,160,6,46,
  134,101,13,159,73,24,146,163,112,156,123,234,207,237,32,102,48,246,222,69,
  81,48,65,72,103,141,19,198,44,102,167,74,135,150,17,144,140,102,138,252,
  77,188,154,176,183,10,230,211,8,227,205,219,158,56,237,245,78,207,94,158,
  191,48,215,56,76,221,64,159,153,67,16,44,201,155,242,190,245,128,239,9,
  243,60,75,60,102,37,127,252,75,114,134,70,220,138,230,127,147,0,132,188,
  29,114,138,68,84,94,159,191,56,75,46,20,9,35,62,17,240,120,41,22,
  55,248,242,229,105,229,201,99,170,104,22,216,55,48,121,20,120,90,107,147,
  4,250,120,236,82,216,250,57,159,20,102,94,36,138,154,166,251,124,253,72,
  46,23,200,201,225,167,251,92,66,188,72,130,78,100,186,43,169,79,65,4,
  244,133,207,42,154,44,18,24,201,83,127,178,16,147,124,160,103,110,218,203,
  155,99,160,184,84,76,89,111,242,123,33,196,95,203,167,254,68,222,46,254,
  254,104,226,39,29,230,15,16,229,15,102,74,211,165,99,78,121,162,3,107,
  40,251,83,48,161,50,38,213,0,191,252,188,6,204,83,196,147,207,221,220,
  250,76,90,76,43,49,64,106,241,84,164,159,92,176,184,180,198,250,16,212,
  42,77,186,58,175,207,135,89,158,148,205,211,152,146,33,197,17,193,240,110,
  8,175,134,2,8,45,247,118,33,124,149,9,108,244,97,169,108,215,209,92,
  251,128,195,135,74,155,0,248,60,84,49,0,103,22,26,74,31,123,218,0,
  198,199,164,146,10,249,104,83,22,112,64,74,224,113,232,157,204,11,101,138,
  182,205,9,25,176,236,84,208,150,233,158,116,141,160,37,134,246,52,102,33,
  243,225,17,62,145,72,77,81,153,243,32,78,40,245,232,76,110,90,74,167,
  134,184,143,233,124,213,74,212,66,23,165,209,221,43,90,77,253,225,172,79,
  119,169,112,136,161,227,167,68,127,103,17,79,197,45,232,190,67,190,73,168,
  149,121,163,79,115,233,105,99,170,137,14,239,228,194,206,1,175,41,208,161,
  69,216,137,181,212,144,77,206,217,185,190,83,49,154,77,99,110,67,10,221,
  210,151,51,65,45,220,64,207,92,19,19,250,230,4,219,67,126,61,174,100,
  67,121,83,87,100,251,39,235,139,13,175,222,157,94,212,148,61,146,250,32,
  18,31,44,91,162,22,77,181,37,112,124,251,33,93,189,151,242,37,232,92,
  47,223,38,222,210,119,185,209,159,17,160,101,78,158,144,7,141,137,62,107,
  167,16,200,223,209,117,46,116,18,117,25,111,66,86,233,2,227,64,207,158,
  1,191,26,135,113,103,8,217,37,87,146,60,25,90,201,197,79,248,201,70,
  68,223,22,133,39,186,101,170,207,3,146,110,153,2,65,150,190,62,234,150,
  11,132,64,180,72,230,143,91,235,18,50,33,244,199,37,7,232,232,44,67,
  164,28,9,71,252,42,121,224,112,119,121,155,21,191,203,240,187,72,226,61,
  10,253,147,219,177,8,38,223,138,69,54,107,103,232,21,179,60,36,51,54,
  128,184,134,82,157,232,78,74,77,52,148,104,236,185,136,11,101,230,194,159,
  60,74,158,190,150,76,65,126,81,163,163,173,97,192,17,138,62,80,40,204,
  165,68,120,49,143,66,74,37,233,54,196,204,124,83,110,186,105,97,140,112,
  144,239,137,183,3,98,202,194,216,163,116,161,188,236,247,10,18,240,105,185,
  252,215,117,245,134,150,109,63,38,7,26,127,186,191,94,174,104,245,119,199,
  85,90,54,90,36,103,27,241,250,102,177,60,197,248,177,98,125,10,221,160,
  108,194,162,148,8,45,224,136,100,162,60,131,155,46,222,168,12,145,28,18,
  79,181,231,48,178,83,238,153,154,215,235,51,19,240,57,228,100,82,126,67,
  109,217,137,250,60,94,118,194,62,83,107,234,250,120,133,12,9,199,210,23,
  206,212,221,155,112,222,137,45,115,249,155,153,208,78,142,73,86,172,100,70,
  139,22,128,205,233,199,149,194,36,207,203,21,242,241,198,149,50,109,239,210,
  34,83,183,241,239,124,62,112,197,17,106,0,242,28,177,69,75,230,139,228,
  138,58,203,178,146,165,39,31,29,9,255,80,205,120,95,60,254,12,229,225,
  191,217,224,208,106,148,6,180,111,203,141,106,108,193,242,232,155,120,124,85,
  169,84,22,255,34,143,9,30,220,81,89,87,82,209,243,201,249,28,41,93,
  204,141,45,189,198,83,209,40,63,44,55,80,84,238,129,24,240,234,149,94,
  247,206,46,90,145,35,229,123,109,243,47,151,171,86,237,252,178,124,186,8,
  217,94,89,148,47,87,18,251,197,190,61,7,157,172,78,18,142,190,7,52,
  93,229,207,121,32,122,175,76,92,199,103,93,209,25,128,48,211,154,134,190,
  185,255,45,7,177,156,126,230,75,20,210,173,37,203,28,2,121,217,237,151,
  47,201,139,182,217,111,146,38,17,250,181,46,37,46,204,61,57,249,110,79,
  177,169,233,116,69,14,13,250,219,162,226,180,245,166,118,126,197,235,50,249,
  87,58,180,53,13,211,87,187,172,64,228,242,186,149,220,173,213,172,27,84,
  190,35,43,193,44,211,92,241,163,71,244,153,206,175,126,249,82,162,27,222,
  74,73,85,124,47,105,130,96,54,50,37,43,131,28,92,153,43,41,215,64,
  204,146,30,253,193,135,116,131,14,111,161,89,236,44,118,118,108,94,213,74,
  151,226,41,4,44,243,22,10,26,49,102,19,69,155,119,242,48,206,14,252,
  238,221,61,88,162,141,49,17,241,70,207,81,199,166,101,113,61,65,80,46,
  237,218,83,119,87,241,165,235,165,234,253,208,70,183,83,212,90,163,219,150,
  100,105,1,83,61,164,5,148,114,32,227,243,40,170,220,211,2,213,252,94,
  209,221,77,128,67,64,187,224,205,46,52,38,34,43,188,174,228,222,211,220,
  77,169,74,51,93,173,200,210,149,208,20,59,51,244,41,97,232,147,97,40,
  178,62,33,111,40,167,53,242,164,206,122,157,92,108,106,53,155,17,62,65,
  94,217,77,75,233,86,165,82,169,74,83,162,52,118,12,81,73,251,22,196,
  18,250,201,147,182,230,126,89,212,237,52,25,136,74,229,163,71,210,226,138,
  59,157,14,55,183,178,36,142,104,140,246,233,233,83,198,17,95,188,194,39,
  138,249,207,63,89,226,85,172,47,214,51,17,33,37,115,250,190,82,242,26,
  83,186,200,228,237,212,92,58,241,254,234,121,18,175,249,24,240,116,209,21,
  146,198,70,227,160,89,175,183,192,71,114,75,96,20,207,166,98,64,27,246,
  35,113,45,233,50,17,88,37,73,231,184,198,213,76,160,203,213,210,160,165,
  106,107,32,9,139,239,185,215,210,211,247,83,30,32,131,55,215,20,80,232,
  200,247,190,240,149,31,123,214,30,94,81,72,167,44,206,38,4,7,113,235,
  130,224,126,93,147,196,11,110,184,217,160,131,200,130,103,61,75,79,164,69,
  10,128,132,136,118,97,100,212,109,123,21,186,155,55,213,193,87,34,211,149,
  40,176,103,1,95,137,168,175,41,161,158,224,193,44,126,189,122,251,198,18,
  103,100,196,210,139,196,116,56,156,171,119,141,190,57,209,63,155,58,20,133,
  145,86,232,85,29,62,124,14,53,4,29,154,142,210,220,250,82,41,202,67,
  211,95,45,201,59,178,4,148,115,49,66,60,238,121,119,247,203,173,117,237,
  69,209,32,86,113,153,238,183,169,210,159,130,32,197,51,251,198,166,113,231,
  158,110,195,9,157,86,233,242,237,85,175,84,213,127,103,65,181,238,75,127,
  212,222,209,28,130,66,252,86,163,43,78,16,226,189,137,167,252,247,34,46,
  66,167,180,88,104,47,197,20,65,200,162,31,29,250,72,3,128,188,21,224,
  250,1,88,49,91,245,50,99,113,103,7,230,106,196,142,72,95,216,217,209,
  13,144,157,238,189,180,166,116,236,62,136,205,53,151,229,229,154,26,215,72,
  219,215,222,191,123,125,37,41,124,184,180,145,115,168,242,61,252,95,43,239,
  49,205,174,178,42,57,202,214,138,195,172,198,159,91,69,91,208,170,16,50,
  252,94,43,239,17,171,236,254,90,171,126,176,202,94,175,181,238,254,78,74,
  13,136,174,94,170,106,175,215,42,240,127,41,8,89,43,178,235,173,85,3,
  95,213,198,188,181,102,214,23,38,63,230,203,77,51,1,16,27,103,146,42,
  221,107,186,178,121,74,95,137,142,56,167,100,204,183,22,103,98,35,89,91,
  140,205,54,127,52,160,164,21,199,184,30,67,115,25,193,124,178,104,65,97,
  45,143,33,158,156,159,238,63,81,56,66,145,35,217,22,106,44,18,10,186,
  205,33,45,209,171,237,201,236,236,39,227,195,126,200,44,41,49,78,239,15,
  140,137,228,45,134,186,243,220,47,101,87,237,90,27,151,216,63,89,43,75,
  135,198,30,19,223,149,213,109,170,100,244,83,59,94,212,218,130,44,46,210,
  35,197,12,229,52,189,92,20,118,141,169,165,189,54,76,145,89,243,40,129,
  51,42,238,20,126,201,27,96,153,243,13,161,173,105,109,123,233,141,22,218,
  139,131,159,124,252,29,6,67,207,29,94,243,102,76,170,92,119,58,5,241,
  187,211,4,168,189,147,13,206,183,96,232,169,122,3,159,196,237,91,224,151,
  59,84,12,142,9,235,183,49,133,196,159,129,52,130,142,249,183,192,187,42,
  162,203,74,8,126,69,212,49,203,51,220,40,233,132,136,143,208,162,200,194,
  132,149,100,48,36,87,40,173,142,7,179,254,187,54,40,194,107,82,113,4,
  148,43,250,93,227,181,76,82,112,124,103,212,154,199,143,28,23,102,200,201,
  252,189,144,1,220,31,95,56,197,147,51,217,85,98,179,72,252,13,99,99,
  69,255,139,26,248,13,67,64,235,220,219,209,104,165,171,116,15,220,235,240,
  185,181,18,92,47,76,15,191,212,225,116,1,30,5,218,173,213,136,59,139,
  214,172,111,65,164,123,5,19,96,125,67,94,33,44,175,66,181,96,156,171,
  188,136,196,245,233,139,12,51,21,174,104,214,60,209,44,138,233,111,47,214,
  173,239,60,140,174,31,104,126,231,183,95,211,190,164,134,175,218,98,189,208,
  74,202,100,242,9,86,185,228,102,103,189,33,34,125,151,219,12,241,127,213,
  159,117,22,191,89,123,62,232,220,40,219,67,247,171,41,211,202,179,113,225,
  86,28,190,167,63,225,120,102,43,9,3,56,55,253,170,155,217,90,193,161,
  68,68,235,4,223,231,155,175,208,160,114,44,67,42,145,168,79,175,200,52,
  26,253,97,147,248,53,253,97,34,122,221,52,33,193,42,197,173,220,201,155,
  152,188,27,215,206,249,33,138,180,217,144,221,147,181,89,201,122,23,15,50,
  110,69,182,141,175,127,21,29,97,76,156,89,231,227,223,63,235,204,124,55,
  73,181,191,182,230,151,85,191,255,143,214,203,216,8,189,49,117,189,127,50,
  47,243,161,151,238,130,165,63,82,12,212,46,130,95,198,3,201,177,147,182,
  241,215,232,225,228,143,5,149,169,168,202,50,107,211,237,80,250,46,168,227,
  93,253,183,221,118,249,175,12,255,47,70,132,108,70,117,120,0,0
};


// ================================================================ WEB SERVER
//
// Security posture (LAN device, no internet exposure intended):
//   - Host header must match this device, which blocks DNS-rebinding attacks from a
//     browser that has been pointed at our IP by a hostile name server.
//   - Every state-changing call is POST, must carry X-Requested-With, and any Origin
//     header must match. A browser cannot set that header cross-origin without a
//     preflight, and no CORS headers are served, so other sites cannot drive the device.
//   - Optional HTTP basic auth (WEB_AUTH_USER / WEB_AUTH_PASS).
//   - Every field is validated and length-bounded before it reaches NVS or SNTP.
//   - Responses never contain the Wi-Fi password or any other secret.

// ================================================================ TEMPERATURE OUTPUT
namespace tempout {

// Temperature -> raw curve. Divider solved from two measurements on this board: node at
// 2.00 V on the 5 V rail gives NTC = 1.5 x lower leg, and 1.59k measured in circuit across
// the bead pads is NTC in parallel with that leg. Together: lower leg 2.7k, bead 4.0k at
// 21 C, i.e. a 3.3k NTC. The clock therefore expects 1.11 V at 0 C to 2.93 V at 40 C.
//
// MEASURED on this board 2026-09-13 with 220 ohm and 2.2 uF fitted, 9-bit duty. Each pair is
// a raw value driven by the temperature march and the number the clock face actually settled
// on. Not a model - re-measure with the march if the resistor, the capacitor, TEMP_PWM_BITS
// or the board changes, because every one of those moves the raw scale.
//
// The nine measured points fit a single NTC curve to +/-0.5 C (least squares: B = 3576 K,
// R25 = 2983 ohm), which is what says the readings are real and not noise. Residuals are
// 4/8 positive, mean 0.00, so the clock ROUNDS rather than truncating - no half-degree bias.
//
// { 39, 509 } is the one EXTRAPOLATED entry, read off that fitted curve 2 C past the last
// measured point. Full duty 511 is the hard ceiling, about 39.3 C. Verify it if it ever matters.
struct TempPoint { int8_t c; uint16_t raw; };
const TempPoint TEMP_CURVE[] = {
  {  0, 186 }, {  4, 220 }, {  8, 258 }, { 13, 297 }, { 18, 337 },
  { 22, 377 }, { 27, 416 }, { 32, 455 }, { 37, 491 }, { 39, 509 },
};
const uint8_t TEMP_CURVE_N = sizeof TEMP_CURVE / sizeof TEMP_CURVE[0];

// Raw ripple sweep. Step 0 is a constant low and the last step is 99.8 percent duty, both
// effectively DC; every step between them is a real square wave. If the clock face is steady
// on the two ends and jumps on the middle steps, the node has no usable low-pass filter and
// the MCU is sampling PWM ripple - that is a hardware fix, not a firmware one.
const uint16_t RAW_SWEEP[] = { 0, TEMP_RAW_MAX / 8, TEMP_RAW_MAX / 4, (TEMP_RAW_MAX * 3) / 8,
                               TEMP_RAW_MAX / 2, (TEMP_RAW_MAX * 5) / 8, (TEMP_RAW_MAX * 3) / 4,
                               (TEMP_RAW_MAX * 7) / 8, TEMP_RAW_MAX };
const uint8_t RAW_SWEEP_N = sizeof RAW_SWEEP / sizeof RAW_SWEEP[0];

// Output sources, highest priority first: march beats hold, hold beats the ambient target,
// and the manual raw value is the fallback. evaluate() picks one every time anything changes,
// so an expiring hold needs no saved previous value to restore.
namespace impl {
uint16_t raw_        = 0;        // what is currently on the pin
uint16_t rawManual_  = 0;        // persisted fallback
bool     attached_   = false;
uint32_t freqHz_     = 0;        // frequency actually achieved, 0 = nothing attached
float    offsetC_    = 0.0f;     // calibration: display reads low by this much

bool     targetValid_ = false;   // ambient temperature (weather feed)
float    targetC_     = 0.0f;

uint32_t holdUntil_ = 0;         // 0 = no hold active
float    holdC_     = 0.0f;

bool     marchRunning_ = false;
bool     marchRaw_     = false;   // true = stepping RAW_SWEEP, false = stepping TEMP_CURVE
uint8_t  marchStep_    = 0;
uint32_t marchNext_    = 0;
uint32_t marchDwell_   = TEMP_MARCH_STEP_MS;

bool     negActive_ = false;     // driving a sub-zero magnitude
bool     negLow_    = false;     // blink phase: true = showing 0 C
uint32_t negNext_   = 0;

void applyRaw(uint16_t v) {                 // drive only, no NVS write
  if (v > TEMP_RAW_MAX) v = TEMP_RAW_MAX;
  raw_ = v;
  if (attached_) ledcWrite(TEMP_OUT_PIN, v);
}
}

// Calibrated temperature -> raw. Saturates at both ends of the curve.
uint16_t rawForC(float c) {
  c += impl::offsetC_;
  if (c <= (float)TEMP_CURVE[0].c) return TEMP_CURVE[0].raw;
  if (c >= (float)TEMP_CURVE[TEMP_CURVE_N - 1].c) return TEMP_CURVE[TEMP_CURVE_N - 1].raw;
  for (uint8_t i = 1; i < TEMP_CURVE_N; ++i) {
    if (c <= (float)TEMP_CURVE[i].c) {
      const float lo = (float)TEMP_CURVE[i - 1].c, hi = (float)TEMP_CURVE[i].c;
      const float f = (c - lo) / (hi - lo);
      return (uint16_t)(TEMP_CURVE[i - 1].raw +
                        f * (float)(TEMP_CURVE[i].raw - TEMP_CURVE[i - 1].raw) + 0.5f);
    }
  }
  return TEMP_CURVE[TEMP_CURVE_N - 1].raw;
}

namespace impl {

// Drive one temperature, encoding sub-zero values as a magnitude the display can show.
void driveC(float c) {
  if (c >= 0.0f) {
    negActive_ = false;
    applyRaw(rawForC(c));
    return;
  }
  if (!negActive_) {                        // entering the sub-zero encoding
    negActive_ = true;
    negLow_ = false;
    negNext_ = millis() + TEMP_NEG_HIGH_MS;
  }
#if TEMP_NEG_BLINK
  applyRaw(negLow_ ? rawForC(0.0f) : rawForC(-c));
#else
  applyRaw(rawForC(-c));
#endif
}

uint8_t marchSteps() { return marchRaw_ ? RAW_SWEEP_N : TEMP_CURVE_N; }

uint16_t marchRawAt(uint8_t i) {
  if (marchRaw_) return i < RAW_SWEEP_N ? RAW_SWEEP[i] : 0;
  return i < TEMP_CURVE_N ? TEMP_CURVE[i].raw : 0;
}

void evaluate() {
  if (marchRunning_) { negActive_ = false; applyRaw(marchRawAt(marchStep_)); return; }
  if (holdUntil_)    { driveC(holdC_);    return; }
  if (targetValid_)  { driveC(targetC_);  return; }
  negActive_ = false;
  applyRaw(rawManual_);
}
}

void begin() {
#if TEMP_OUT_ENABLE
  Preferences p;
  if (p.begin("ntpclock", true)) {
    impl::rawManual_ = p.getUShort("traw", 0);
    impl::offsetC_ = p.getFloat("toff", 0.0f);
    p.end();
  }
  if (impl::rawManual_ > TEMP_RAW_MAX) impl::rawManual_ = TEMP_RAW_MAX;

  // ledcAttach fails outright when frequency x 2^bits exceeds the LEDC source clock, and a
  // failure here means the pin is never driven. Walk down until one takes, and say which.
  static const uint32_t kFreqs[] = { TEMP_PWM_FREQ_HZ, TEMP_PWM_FALLBACKS };
  impl::attached_ = false;                  // never report a stale attach from an earlier call
  impl::freqHz_ = 0;
  for (uint8_t i = 0; i < sizeof kFreqs / sizeof kFreqs[0]; ++i) {
    if (ledcAttach(TEMP_OUT_PIN, kFreqs[i], TEMP_PWM_BITS)) {
      impl::attached_ = true;
      impl::freqHz_ = kFreqs[i];
      if (i) Serial.printf("WARN: temp PWM fell back to %lu Hz (%lu Hz refused)\n",
                           (unsigned long)kFreqs[i], (unsigned long)TEMP_PWM_FREQ_HZ);
      else   Serial.printf("Temp PWM: GPIO%d at %lu Hz, %d bits (0-%d)\n", TEMP_OUT_PIN,
                           (unsigned long)kFreqs[i], TEMP_PWM_BITS, TEMP_RAW_MAX);
      break;
    }
  }
  if (!impl::attached_)
    Serial.printf("ERROR: temp PWM refused every frequency on GPIO%d - output is dead\n",
                  TEMP_OUT_PIN);
  impl::evaluate();
#endif
}

uint16_t raw()           { return impl::raw_; }
bool     attached()      { return impl::attached_; }
uint32_t freqHz()        { return impl::freqHz_; }
float    offsetC()       { return impl::offsetC_; }
bool     marchRunning()  { return impl::marchRunning_; }
bool     marchIsRaw()    { return impl::marchRaw_; }
uint8_t  marchStep()     { return impl::marchStep_; }
uint8_t  marchSteps()    { return impl::marchSteps(); }
uint32_t marchDwellS()   { return impl::marchDwell_ / 1000UL; }
// Temperature the current march step corresponds to. Meaningless during a raw sweep.
int      marchStepC()    { return impl::marchStep_ < TEMP_CURVE_N
                                    ? (int)TEMP_CURVE[impl::marchStep_].c : 0; }
bool     negActive()     { return impl::negActive_; }
bool     targetValid()   { return impl::targetValid_; }
float    targetC()       { return impl::targetC_; }

// Whole seconds of hold left. Guards the wrap: millis() can pass holdUntil_ before loop()
// clears it, and an unsigned subtract there would report ~49 days remaining.
uint32_t holdLeftS() {
  if (!impl::holdUntil_) return 0;
  const int32_t left = (int32_t)(impl::holdUntil_ - millis());
  return left > 0 ? (uint32_t)left / 1000UL : 0;
}

void setRaw(uint16_t v) {
#if TEMP_OUT_ENABLE
  impl::holdUntil_ = 0;                  // a manual value overrides hold, march and ambient
  impl::marchRunning_ = false;
  impl::targetValid_ = false;
  impl::rawManual_ = v > TEMP_RAW_MAX ? TEMP_RAW_MAX : v;
  impl::evaluate();
  Preferences p;                       // manual action only, not a loop: NVS write is fine
  if (p.begin("ntpclock", false)) { p.putUShort("traw", impl::rawManual_); p.end(); }
#endif
}

void setOffsetC(float c) {
  if (c < -20.0f) c = -20.0f;
  if (c >  20.0f) c =  20.0f;
  impl::offsetC_ = c;
  impl::evaluate();                      // recalibrate whatever is on the pin right now
  Preferences p;
  if (p.begin("ntpclock", false)) { p.putFloat("toff", c); p.end(); }
}

// Ambient temperature from the weather feed. Persists until cleared; survives hold and march.
void setTargetC(float c) {
  impl::targetC_ = c;
  impl::targetValid_ = true;
  impl::evaluate();
}

void clearTarget() {
  impl::targetValid_ = false;
  impl::evaluate();
}

// Show a known temperature for a while. When it expires evaluate() falls back on its own.
void holdC(float c, uint32_t ms) {
  impl::marchRunning_ = false;
  impl::holdC_ = c;
  impl::holdUntil_ = millis() + ms;
  if (!impl::holdUntil_) impl::holdUntil_ = 1;   // never land on the "no hold" sentinel
  impl::evaluate();
}

// Walk a table so the display can be read against known raw values. raw = false steps
// TEMP_CURVE by temperature (calibration), raw = true steps RAW_SWEEP by duty (ripple check).
// dwellMs 0 takes the default. The dwell has to beat the clock's own update and averaging
// rate, which is unknown and evidently slower than 12 s, or a step is read mid-settle.
bool startMarch(bool raw, uint32_t dwellMs) {
  if (impl::marchRunning_) return false;
  if (!dwellMs) dwellMs = raw ? TEMP_SWEEP_STEP_MS : TEMP_MARCH_STEP_MS;
  impl::holdUntil_ = 0;
  impl::marchStep_ = 0;
  impl::marchRaw_ = raw;
  impl::marchDwell_ = dwellMs;
  impl::marchRunning_ = true;
  impl::marchNext_ = millis() + dwellMs;
  impl::evaluate();
  return true;
}

void loop() {
  const uint32_t now = millis();
  bool dirty = false;

  if (impl::marchRunning_ && (int32_t)(now - impl::marchNext_) >= 0) {
    if (++impl::marchStep_ >= impl::marchSteps()) {
      impl::marchRunning_ = false;
      impl::marchStep_ = 0;
    } else {
      impl::marchNext_ = now + impl::marchDwell_;
    }
    dirty = true;
  }

  if (impl::holdUntil_ && (int32_t)(now - impl::holdUntil_) >= 0) {
    impl::holdUntil_ = 0;
    dirty = true;
  }

#if TEMP_NEG_BLINK
  if (impl::negActive_ && (int32_t)(now - impl::negNext_) >= 0) {
    impl::negLow_ = !impl::negLow_;
    impl::negNext_ = now + (impl::negLow_ ? TEMP_NEG_LOW_MS : TEMP_NEG_HIGH_MS);
    dirty = true;
  }
#endif

  if (dirty) impl::evaluate();
}

}  // namespace tempout

// ---- Outdoor temperature feed: ECCC SWOB-ML -> tempout::setTargetC()
// Streams the response and scans for two quoted field names rather than buffering the
// document, so a malformed or oversized reply cannot grow the heap. See the WX_ config block.
namespace wx {

struct Status {
  bool     haveReading;        // a good reading has been applied at least once
  float    tempC;              // last accepted air_temp
  char     obsTime[28];        // date_tm as published, UTC
  bool     obsTimeValid;
  bool     ageKnown;           // false until the clock itself is NTP-synced
  uint32_t obsAgeS;            // age of the accepted reading at the moment it was accepted
  uint32_t attempts, accepted, httpFails, parseFails, staleRejects, heapSkips;
  int      lastHttpCode;
  char     lastError[56];
  uint32_t lastAttemptAgoS, lastOkAgoS;
  uint32_t nextInS;
  uint32_t heapBefore, heapAfter, largestBefore;
  bool     pending;            // a manual fetch is queued
};

namespace impl {

uint32_t nextMs_      = 0;
uint32_t lastAttempt_ = 0;
uint32_t lastOk_      = 0;
bool     everAttempt_ = false;
bool     everOk_      = false;
bool     pending_     = false;
bool     haveReading_ = false;
float    tempC_       = 0.0f;
char     obsTime_[28] = { 0 };
bool     obsValid_    = false;
bool     ageKnown_    = false;
uint32_t obsAgeS_     = 0;
uint32_t attempts_ = 0, accepted_ = 0, httpFails_ = 0, parseFails_ = 0;
uint32_t staleRejects_ = 0, heapSkips_ = 0;
int      lastCode_ = 0;
char     lastError_[56] = { 0 };
uint32_t heapBefore_ = 0, heapAfter_ = 0, largestBefore_ = 0;

void setError(const char* m) {
  std::strncpy(lastError_, m, sizeof lastError_ - 1);
  lastError_[sizeof lastError_ - 1] = '\0';
}

// Days since 1970-01-01 from a proleptic Gregorian date. Howard Hinnant's days_from_civil.
int64_t daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153u * (m + (m > 2 ? -3u : 9u)) + 2u) / 5u + d - 1u;
  const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

// "2026-09-13T02:00:00.000Z" -> UTC epoch. Rejects anything that is not that exact shape,
// and rejects a non-Z suffix rather than silently treating a local time as UTC.
bool epochFromIso(const char* s, time_t* out) {
  if (std::strlen(s) < 20) return false;
  if (s[4] != '-' || s[7] != '-' || s[13] != ':' || s[16] != ':') return false;
  if (s[10] != 'T' && s[10] != ' ') return false;
  for (int i = 0; i < 19; ++i) {
    if (i == 4 || i == 7 || i == 10 || i == 13 || i == 16) continue;
    if (s[i] < '0' || s[i] > '9') return false;
  }
  if (!std::strchr(s + 19, 'Z')) return false;
  const int Y  = (s[0]-'0')*1000 + (s[1]-'0')*100 + (s[2]-'0')*10 + (s[3]-'0');
  const int Mo = (s[5]-'0')*10 + (s[6]-'0');
  const int D  = (s[8]-'0')*10 + (s[9]-'0');
  const int H  = (s[11]-'0')*10 + (s[12]-'0');
  const int Mi = (s[14]-'0')*10 + (s[15]-'0');
  const int Se = (s[17]-'0')*10 + (s[18]-'0');
  if (Y < 2020 || Y > 2199 || Mo < 1 || Mo > 12 || D < 1 || D > 31) return false;
  if (H > 23 || Mi > 59 || Se > 60) return false;
  *out = (time_t)(daysFromCivil(Y, (unsigned)Mo, (unsigned)D) * 86400LL
                  + (int64_t)H * 3600 + (int64_t)Mi * 60 + Se);
  return true;
}

// Finds name="<needle>" ... value="<v>" in a byte stream, one character at a time.
// The needle carries its own quotes so that "air_temp" cannot match avg_air_temp_pst1hr.
class FieldScan {
 public:
  explicit FieldScan(const char* needle) : needle_(needle) {}

  void feed(char c) {
    if (done_) return;
    if (state_ == 0) {
      np_ = (c == needle_[np_]) ? (uint8_t)(np_ + 1) : (uint8_t)(c == needle_[0] ? 1 : 0);
      if (needle_[np_] == '\0') { state_ = 1; tp_ = 0; }
      return;
    }
    if (state_ == 1) {
      tp_ = (c == TAG[tp_]) ? (uint8_t)(tp_ + 1) : (uint8_t)(c == TAG[0] ? 1 : 0);
      if (TAG[tp_] == '\0') { state_ = 2; len_ = 0; }
      return;
    }
    if (c == '"') { buf_[len_] = '\0'; done_ = true; return; }
    if (len_ < sizeof buf_ - 1) buf_[len_++] = c;
    else { buf_[len_] = '\0'; done_ = true; }      // value absurdly long: take what we have
  }

  bool        done()  const { return done_; }
  const char* value() const { return buf_; }

 private:
  static constexpr const char* TAG = " value=\"";
  const char* needle_;
  uint8_t np_ = 0, tp_ = 0, state_ = 0, len_ = 0;
  char buf_[28] = { 0 };
  bool done_ = false;
};

bool accept(const FieldScan& fsTemp, const FieldScan& fsTime);

// One fetch attempt. Returns true only when a fresh reading was applied to the output.
bool doFetch() {
  lastAttempt_ = millis();
  everAttempt_ = true;
  lastCode_ = 0;

  if (!WiFi.isConnected()) { httpFails_++; setError("Wi-Fi down"); return false; }

  largestBefore_ = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  heapBefore_ = (uint32_t)ESP.getFreeHeap();
  if (WX_USE_HTTPS && largestBefore_ < WX_MIN_LARGEST_BLOCK) {
    heapSkips_++;
    setError("skipped: largest heap block too small for TLS");
    return false;
  }

  attempts_++;

  char url[192];
  std::snprintf(url, sizeof url, "%s://%s%s%s%s", WX_USE_HTTPS ? "https" : "http",
                WX_HOST, WX_PATH_PREFIX, g_settings.wxStation, WX_PATH_SUFFIX);

  bool applied = false;
  {
#if WX_USE_HTTPS
    WiFiClientSecure client;
    client.setInsecure();                 // no cert store on the device; public data only
#else
    WiFiClient client;
#endif
    HTTPClient http;
    http.setConnectTimeout(WX_TIMEOUT_MS);
    // Milliseconds: HTTPClient passes this straight to the client, which inherits
    // Stream::setTimeout. Do not also call client.setTimeout() - it is the same knob.
    http.setTimeout(WX_TIMEOUT_MS);
    http.setReuse(false);
    // Do not follow: a redirect is information (it means this scheme is wrong), not a detour.
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    if (!http.begin(client, url)) {
      httpFails_++; setError("begin failed (bad URL?)");
    } else {
      http.addHeader("Accept", "application/xml");
      lastCode_ = http.GET();
      if (lastCode_ != HTTP_CODE_OK) {
        httpFails_++;
        char m[56];
        std::snprintf(m, sizeof m, "HTTP %d%s", lastCode_,
                      lastCode_ == 404 ? " (station code wrong?)" : "");
        setError(m);
      } else {
        WiFiClient* s = http.getStreamPtr();
        FieldScan fsTime("\"date_tm\""), fsTemp("\"air_temp\"");
        uint32_t seen = 0, idle = millis();
        uint8_t chunk[64];
        while (seen < WX_MAX_BYTES && !(fsTime.done() && fsTemp.done())) {
          const int avail = s->available();
          if (avail > 0) {
            const int want = avail > (int)sizeof chunk ? (int)sizeof chunk : avail;
            const int got = s->read(chunk, (size_t)want);
            if (got <= 0) { delay(2); continue; }
            for (int i = 0; i < got; ++i) {
              fsTime.feed((char)chunk[i]);
              fsTemp.feed((char)chunk[i]);
            }
            seen += (uint32_t)got;
            idle = millis();
            continue;
          }
          if (!http.connected()) break;
          if (millis() - idle > (uint32_t)WX_TIMEOUT_MS) break;
          delay(2);
        }
        applied = accept(fsTemp, fsTime);
      }
      http.end();
    }
  }

  heapAfter_ = (uint32_t)ESP.getFreeHeap();
  if (applied) { lastOk_ = millis(); everOk_ = true; }
  return applied;
}

// Validate what the scan found and drive the output. Separated so it can be reasoned about
// without the network in the way.
bool accept(const FieldScan& fsTemp, const FieldScan& fsTime) {
  if (!fsTemp.done()) { parseFails_++; setError("air_temp not found in reply"); return false; }

  char* endp = nullptr;
  const double v = std::strtod(fsTemp.value(), &endp);
  if (endp == fsTemp.value()) { parseFails_++; setError("air_temp not a number"); return false; }
  while (*endp == ' ') ++endp;
  if (*endp != '\0')          { parseFails_++; setError("air_temp has trailing junk"); return false; }
  if (!(v >= -80.0 && v <= 60.0)) { parseFails_++; setError("air_temp outside -80..60"); return false; }

  // Staleness. CXTO once served HTTP 200 with a date_tm five weeks old, so a missing or
  // unparseable timestamp is a rejection, not a shrug.
  obsValid_ = false;
  ageKnown_ = false;
  obsTime_[0] = '\0';
  if (!fsTime.done()) { parseFails_++; setError("date_tm not found in reply"); return false; }
  std::strncpy(obsTime_, fsTime.value(), sizeof obsTime_ - 1);
  obsTime_[sizeof obsTime_ - 1] = '\0';
  time_t obs = 0;
  if (!epochFromIso(obsTime_, &obs)) { parseFails_++; setError("date_tm unparseable"); return false; }
  obsValid_ = true;

  const time_t now = time(nullptr);
  if (now > 1700000000) {                     // clock is synced, so the age is meaningful
    ageKnown_ = true;
    const int64_t age = (int64_t)now - (int64_t)obs;
    if (age > (int64_t)WX_MAX_AGE_S) {
      staleRejects_++;
      obsAgeS_ = (uint32_t)age;
      char m[56];
      std::snprintf(m, sizeof m, "reading %lld min old, rejected", (long long)(age / 60));
      setError(m);
      return false;                           // keep the previous value rather than show this
    }
    obsAgeS_ = age > 0 ? (uint32_t)age : 0;
  }

  tempC_ = (float)v;
  haveReading_ = true;
  accepted_++;
  setError("");
  tempout::setTargetC(tempC_);
  Serial.printf("[wx] %s %.1f C obs %s%s\n", g_settings.wxStation, (double)tempC_, obsTime_,
                ageKnown_ ? "" : " (age unknown, clock not synced)");
  return true;
}

}  // namespace impl

void begin() {
#if WX_ENABLE
  // First attempt is deferred so NTP can land; the staleness check needs a real clock.
  impl::nextMs_ = millis() + 15000UL;
  if (!impl::nextMs_) impl::nextMs_ = 1;
#endif
}

void requestNow() { impl::pending_ = true; }

void loop() {
#if WX_ENABLE
  if (!WiFi.isConnected()) return;
  if (diag::testRunning()) return;           // a blocking fetch would skew a timed bus test
  const uint32_t now = millis();
  const bool due = impl::pending_ || (int32_t)(now - impl::nextMs_) >= 0;
  if (!due) return;
  impl::pending_ = false;
  const bool ok = impl::doFetch();
  impl::nextMs_ = millis() + (ok ? WX_FETCH_INTERVAL_S : WX_RETRY_S) * 1000UL;
  if (!impl::nextMs_) impl::nextMs_ = 1;
#endif
}

void get(Status& out) {
  const uint32_t now = millis();
  out.haveReading = impl::haveReading_;
  out.tempC       = impl::tempC_;
  out.obsTimeValid = impl::obsValid_;
  std::strncpy(out.obsTime, impl::obsTime_, sizeof out.obsTime - 1);
  out.obsTime[sizeof out.obsTime - 1] = '\0';
  out.ageKnown     = impl::ageKnown_;
  out.obsAgeS      = impl::obsAgeS_;
  out.attempts     = impl::attempts_;
  out.accepted     = impl::accepted_;
  out.httpFails    = impl::httpFails_;
  out.parseFails   = impl::parseFails_;
  out.staleRejects = impl::staleRejects_;
  out.heapSkips    = impl::heapSkips_;
  out.lastHttpCode = impl::lastCode_;
  std::strncpy(out.lastError, impl::lastError_, sizeof out.lastError - 1);
  out.lastError[sizeof out.lastError - 1] = '\0';
  out.lastAttemptAgoS = impl::everAttempt_ ? (now - impl::lastAttempt_) / 1000UL : 0;
  out.lastOkAgoS      = impl::everOk_ ? (now - impl::lastOk_) / 1000UL : 0;
  const int32_t left  = (int32_t)(impl::nextMs_ - now);
  out.nextInS         = left > 0 ? (uint32_t)left / 1000UL : 0;
  out.heapBefore      = impl::heapBefore_;
  out.heapAfter       = impl::heapAfter_;
  out.largestBefore   = impl::largestBefore_;
  out.pending         = impl::pending_;
}

bool everAttempted() { return impl::everAttempt_; }
bool everSucceeded() { return impl::everOk_; }

}  // namespace wx

// ---- Why the last boot happened. Captured in setup(), reported on serial and in the UI so
// an unattended reboot explains itself without anyone watching a serial monitor.
esp_reset_reason_t g_resetReason = ESP_RST_UNKNOWN;
uint32_t g_bootCount = 0;          // boots since flashing, persisted in NVS

const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external reset pin";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "PANIC: exception or assert";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep-sleep wake";
    case ESP_RST_BROWNOUT:  return "BROWNOUT: supply sagged";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "unknown";
  }
}

namespace webui {

namespace impl {

WebServer server(80);
const char* COLLECT_HEADERS[] = { "Origin", "X-Requested-With" };
uint32_t statusServed_ = 0;    // completed /api/status responses, for the serial heartbeat

// Minimal JSON writer with string escaping.
class Json {
 public:
  explicit Json(String& out) : s_(out) {}
  Json& open(const char* key = nullptr) { sep(); if (key) name(key); s_ += '{'; first_ = true; return *this; }
  Json& close() { s_ += '}'; first_ = false; return *this; }
  Json& str(const char* key, const char* v) { sep(); name(key); quote(v ? v : ""); return *this; }
  Json& str(const char* key, const String& v) { return str(key, v.c_str()); }
  Json& num(const char* key, long v) { sep(); name(key); s_ += String(v); return *this; }
  Json& unum(const char* key, unsigned long v) { sep(); name(key); s_ += String(v); return *this; }
  Json& real(const char* key, float v, unsigned char dp) { sep(); name(key); if (std::isfinite(v)) s_ += String(v, (unsigned int)dp); else s_ += '0'; return *this; }
  Json& boolean(const char* key, bool v) { sep(); name(key); s_ += v ? "true" : "false"; return *this; }
  Json& null(const char* key) { sep(); name(key); s_ += "null"; return *this; }

 private:
  void sep() { if (!first_) s_ += ','; first_ = false; }
  void name(const char* k) { quote(k); s_ += ':'; }
  void quote(const char* v) {
    s_ += '"';
    for (const char* p = v; *p; ++p) {
      const unsigned char c = (unsigned char)*p;
      if (c == '"') s_ += "\\\"";
      else if (c == '\\') s_ += "\\\\";
      else if (c == '\n') s_ += "\\n";
      else if (c < 0x20) { char u[8]; std::snprintf(u, sizeof u, "\\u%04x", c); s_ += u; }
      else s_ += (char)c;
    }
    s_ += '"';
  }
  String& s_;
  bool first_ = true;
};

void sendJson(int code, const String& body) {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.send(code, "application/json", body);
}

void sendError(int code, const char* msg) {
  String out;
  Json(out).open().boolean("ok", false).str("error", msg).close();
  sendJson(code, out);
}

String hostOnly(const String& hostHeader) {
  const int colon = hostHeader.indexOf(':');
  return colon < 0 ? hostHeader : hostHeader.substring(0, colon);
}

// DNS-rebinding guard: the Host header must name this device.
bool hostAllowed() {
  String h = hostOnly(server.hostHeader());
  if (h.length() == 0) return false;
  h.toLowerCase();
  String me(DEVICE_HOSTNAME);
  me.toLowerCase();
  if (h == me || h == me + ".local") return true;
  if (h == WiFi.localIP().toString()) return true;
  if (h == "localhost") return true;
  return false;
}

bool authOk() {
  if (std::strlen(WEB_AUTH_USER) == 0) return true;
  if (server.authenticate(WEB_AUTH_USER, WEB_AUTH_PASS)) return true;
  server.requestAuthentication();
  return false;
}

// Cross-site request forgery guard for state-changing calls.
bool csrfOk() {
  if (!server.hasHeader("X-Requested-With") || server.header("X-Requested-With") != "NtpClockMod") return false;
  if (server.hasHeader("Origin")) {
    const String origin = server.header("Origin");
    const String expect = "http://" + server.hostHeader();
    if (origin != expect) return false;
  }
  return true;
}

bool guard(bool stateChanging) {
  if (!hostAllowed()) { sendError(403, "host not allowed"); return false; }
  if (!authOk()) return false;                       // 401 already sent
  if (stateChanging && !csrfOk()) { sendError(403, "cross-site request rejected"); return false; }
  return true;
}

String fmtTm(const std::tm& tm) {
  char b[40];
  strftime(b, sizeof b, "%Y-%m-%d %H:%M:%S %Z", &tm);
  return String(b);
}

String flagText(uint16_t f) {
  String s;
  auto add = [&](bool on, const char* t) { if (on) { if (s.length()) s += ", "; s += t; } };
  add(f & ds1302::TF_SPOOFED, "answered by ESP32");
  add((f & ds1302::TF_READ) && !(f & ds1302::TF_SPOOFED) &&
      !(f & (ds1302::TF_TOO_SLOW | ds1302::TF_LONG | ds1302::TF_TIMEOUT)), "answered by DS1302");
  add(f & ds1302::TF_COMPLETE, "complete");
  add(f & ds1302::TF_LATE_EDGE, "SCLK edge before ISR");
  add(f & ds1302::TF_CE_BOUNCE, "CE glitch rejected");
  add(f & ds1302::TF_TOO_SLOW, "over ISR budget");
  add(f & ds1302::TF_TIMEOUT, "timeout");
  add(f & ds1302::TF_LONG, "RAM burst not followed");
  add(f & ds1302::TF_INVALID, "invalid");
  return s;
}

String txnText(const ds1302::TxnRecord& r) {
  char d[48];
  ds1302::describeCmd(r.cmd, d, sizeof d);
  String s(d);
  const uint16_t whole = r.dataBits / 8;
  const uint16_t n = (r.flags & ds1302::TF_BURST) ? std::min<uint16_t>(whole, 8) : std::min<uint16_t>(whole, 1);
  if (n) {
    s += " =";
    for (uint16_t i = 0; i < n; ++i) { char h[4]; std::snprintf(h, sizeof h, " %02X", r.data[i]); s += h; }
  }
  return s;
}

void txnJson(Json& j, const char* key, const ds1302::TxnRecord& r, int64_t nowUs) {
  if (!(r.flags & ds1302::TF_CMD)) { j.null(key); return; }
  j.open(key)
      .str("cmd", txnText(r))
      .str("flags", flagText(r.flags))
      .unum("age_ms", (unsigned long)((nowUs - r.t0Us) / 1000))
      .unum("sclk_rises", r.rises)
      .unum("data_bits", r.dataBits)
      .unum("isr_us", r.durUs)
      .close();
}

void handleRoot() {
  if (!guard(false)) return;
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.sendHeader("Content-Encoding", "gzip");
  server.send_P(200, "text/html; charset=utf-8", (PGM_P)INDEX_HTML_GZ, sizeof(INDEX_HTML_GZ));
}

void handleStatus() {
  if (!guard(false)) return;

  BusSnapshot snap;
  bus::snapshot(snap);
  diag::Rates rates;
  diag::getRates(rates);
  spoof::Status st;
  spoof::getStatus(st);
  wifimgr::Stats ws;
  wifimgr::getStats(ws);
  const int64_t nowUs = esp_timer_get_time();

  // Reused across requests. A fresh 5 KB String per poll, at 1 Hz for hours, fragments the
  // heap even though total free never moves. Keep one buffer and refill it.
  static String out;
  out.remove(0);                 // drop contents, keep the allocation
  out.reserve(5120);             // no-op once the buffer is large enough
  Json j(out);
  j.open();
  j.str("version", FW_VERSION).unum("uptime_s", (unsigned long)(nowUs / 1000000));
  j.str("reset_reason", resetReasonName(g_resetReason))
      .unum("boot_count", (unsigned long)g_bootCount)
      .unum("heap_free", (unsigned long)ESP.getFreeHeap())
      .unum("heap_largest", (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT))
      .unum("heap_min", (unsigned long)ESP.getMinFreeHeap());
  j.unum("temp_raw", (unsigned long)tempout::raw())
      .boolean("temp_ok", tempout::attached())
      // Voltage at the clock's ADC node, after the series resistor divides against the
      // board's 2.7k leg - this is what a DC meter on the pad reads, not the pin voltage.
      .unum("temp_mv", (unsigned long)((uint32_t)tempout::raw()
                                       * (uint32_t)TEMP_NODE_MV_MAX / TEMP_RAW_MAX))
      .unum("temp_mv_max", (unsigned long)TEMP_NODE_MV_MAX)
      .real("temp_offset", tempout::offsetC(), 1)
      .unum("temp_raw_max", (unsigned long)TEMP_RAW_MAX)
      .unum("temp_pwm_hz", (unsigned long)tempout::freqHz())
      .unum("temp_pwm_hz_want", (unsigned long)TEMP_PWM_FREQ_HZ)
      .unum("temp_pwm_bits", (unsigned long)TEMP_PWM_BITS)
      .unum("temp_pin", (unsigned long)TEMP_OUT_PIN)
      .boolean("temp_march", tempout::marchRunning())
      .boolean("temp_march_raw", tempout::marchIsRaw())
      .num("temp_march_c", tempout::marchRunning() && !tempout::marchIsRaw()
             ? (long)tempout::marchStepC() : (long)0)
      .unum("temp_march_step", (unsigned long)tempout::marchStep() + 1)
      .unum("temp_march_steps", (unsigned long)tempout::marchSteps())
      .unum("temp_march_dwell_s", (unsigned long)tempout::marchDwellS())
      .unum("temp_hold_s", tempout::holdLeftS())
      .boolean("temp_neg", tempout::negActive())
      .boolean("temp_neg_blink", TEMP_NEG_BLINK ? true : false)
      .boolean("temp_target_valid", tempout::targetValid())
      .real("temp_target_c", tempout::targetC(), 1);

  // The outdoor feed, reported in enough detail to tell "API is broken" from "wiring is
  // broken": what was fetched, how old it was, what it maps to, and what the face should read.
  {
    wx::Status wxs;
    wx::get(wxs);
    j.open("wx").boolean("enabled", WX_ENABLE ? true : false)
        .boolean("https", WX_USE_HTTPS ? true : false)
        .str("station", g_settings.wxStation)
        .boolean("have_reading", wxs.haveReading)
        .real("temp_c", wxs.tempC, 1)
        .str("obs_time", wxs.obsTimeValid ? wxs.obsTime : "")
        .boolean("age_known", wxs.ageKnown)
        .unum("obs_age_s", wxs.obsAgeS)
        .unum("max_age_s", (unsigned long)WX_MAX_AGE_S)
        .unum("attempts", wxs.attempts).unum("accepted", wxs.accepted)
        .unum("http_fails", wxs.httpFails).unum("parse_fails", wxs.parseFails)
        .unum("stale_rejects", wxs.staleRejects).unum("heap_skips", wxs.heapSkips)
        .num("last_http_code", (long)wxs.lastHttpCode)
        .str("last_error", wxs.lastError)
        .boolean("ever_attempted", wx::everAttempted())
        .boolean("ever_ok", wx::everSucceeded())
        .unum("last_attempt_ago_s", wxs.lastAttemptAgoS)
        .unum("last_ok_ago_s", wxs.lastOkAgoS)
        .unum("next_in_s", wxs.nextInS)
        .boolean("pending", wxs.pending)
        .unum("heap_before", wxs.heapBefore).unum("heap_after", wxs.heapAfter)
        .unum("largest_before", wxs.largestBefore);
    // What this reading drives: the raw value after calibration, and the number the clock
    // face should settle on - the magnitude when the feed is below zero.
    if (wxs.haveReading) {
      const float t = wxs.tempC;
      j.unum("would_raw", (unsigned long)tempout::rawForC(t < 0.0f ? -t : t))
          .num("would_show_c", (long)lroundf(t < 0.0f ? -t : t))
          .boolean("would_be_neg", t < 0.0f);
    } else {
      j.null("would_raw").null("would_show_c").boolean("would_be_neg", false);
    }
    j.close();
  }

  const bool up = WiFi.isConnected();
  j.open("wifi").boolean("connected", up).str("ssid", WIFI_SSID).str("host", DEVICE_HOSTNAME)
      .unum("connects", ws.connects).unum("disconnects", ws.disconnects)
      .unum("retries", ws.retries).unum("radio_resets", ws.radioResets)
      .unum("last_reason", ws.lastReason).str("last_reason_text", wifimgr::reasonName(ws.lastReason))
      .unum("down_s", wifimgr::downSeconds()).unum("retry_ms", wifimgr::retryDelayMs())
      .num("tx_dbm", (long)g_settings.wifiTxDbm)
      .num("tx_dbm_actual", (long)wifimgr::actualTxDbm())
      .boolean("tx_applied", wifimgr::txApplied())
      .num("ps_mode", (long)g_settings.wifiPsMode)
      .str("ps_mode_text", g_settings.wifiPsMode == 2 ? "max modem sleep"
                         : g_settings.wifiPsMode == 1 ? "min modem sleep" : "no sleep");
  if (up) j.num("rssi", WiFi.RSSI()).str("ip", WiFi.localIP().toString());
  j.close();

  j.open("ntp").str("server", timekeeping::server()).boolean("synced", timekeeping::everSynced())
      .unum("sync_count", timekeeping::syncCount()).unum("interval_s", timekeeping::syncIntervalS());
  j.boolean("drift_valid", timekeeping::driftValid())
      .real("drift_ppm", timekeeping::driftPpm(), 2)
      .unum("drift_span_s", timekeeping::driftSpanS());
  const int64_t age = timekeeping::lastSyncAgeUs();
  if (age >= 0) {
    std::tm tm;
    timekeeping::toLocal(timekeeping::lastSyncEpoch(), tm);
    j.str("last_sync", fmtTm(tm)).unum("last_sync_age_s", (unsigned long)(age / 1000000));
  } else {
    j.null("last_sync");
  }
  j.close();

  std::tm now;
  const bool timeOk = timekeeping::localNow(now);
  j.open("time");
  if (timeOk) j.str("local", fmtTm(now)); else j.null("local");
  j.str("tz", g_settings.tz).str("tz_active", timekeeping::activeTz())
      .str("dst_mode", settings::dstModeName(g_settings.dstMode))
      .str("dst_mode_text", tzrule::modeName(g_settings.dstMode)).close();

  char buf[48];
  j.open("spoof").boolean("enabled", g_settings.spoofEnabled).boolean("armed", st.armed)
      .boolean("march", st.march).boolean("countdown", st.countdown)
      .boolean("boot_countdown", g_settings.bootCountdown)
      .str("reason", st.reason ? st.reason : "starting");
  if (st.haveTime) { ds1302::formatImage(st.reg, buf, sizeof buf); j.str("image", buf); } else { j.null("image"); }
  std::snprintf(buf, sizeof buf, "%02X %02X %02X %02X %02X %02X %02X", st.reg[0], st.reg[1], st.reg[2],
                st.reg[3], st.reg[4], st.reg[5], st.reg[6]);
  j.str("image_hex", buf)
      .str("hour_format", settings::hourFormatName(g_settings.hourFormat))
      .str("hour_mode", st.hour12 ? "12h" : "24h")
      .str("mcu_hour", snap.learnedHour < 0 ? "unknown" : (snap.learnedHour ? "12h" : "24h"))
      .unum("holdover_h", (unsigned long)NTP_HOLDOVER_HOURS)
      .close();

  j.open("bus").boolean("isr_installed", bus::isrInstalled()).boolean("isr_iram", bus::isrIramSafe())
      .real("ce_per_s", rates.cePerSec, 1)
      .real("sclk_hz", rates.sclkHz, 0)
      .str("poll_style", rates.pollStyle)
      .unum("poll_gap_ms", rates.pollGapMs);
  txnJson(j, "last", snap.last, nowUs);
  txnJson(j, "last_read", snap.lastRead, nowUs);
  txnJson(j, "last_write", snap.lastWrite, nowUs);
  if (snap.realMask == 0x7F) {
    ds1302::formatImage(snap.real, buf, sizeof buf);
    j.str("real_time", buf).unum("real_age_s", (unsigned long)((nowUs - snap.realUs) / 1000000));
  } else {
    j.null("real_time");
  }
  const BusCounters& c = snap.c;
  j.open("counters")
      .unum("ce_rises", c.ceRises).unum("single_reads", c.singleReads).unum("burst_reads", c.burstReads)
      .unum("writes", c.writes).unum("spoofed", c.spoofed).unum("captured", c.passReads)
      .unum("invalid", c.invalid).unum("late_sclk_high", c.misaligned).unum("late_sclk_edge", c.lateEdge)
      .unum("late_ce_low", c.glitches).unum("over_budget", c.tooSlow).unum("timeouts", c.timeouts)
      .unum("ram_burst", c.longTxn).unum("incomplete", c.incomplete)
      .unum("ce_bounces", c.ceBounces)
      .close();
  j.close();

  j.open("test").str("name", diag::testName()).boolean("running", diag::testRunning())
      .unum("elapsed_ms", diag::testElapsedMs()).unum("duration_ms", diag::testDurationMs())
      .str("report", diag::testReport())
      .close();
  j.close();
  ++statusServed_;
  sendJson(200, out);
}

// Lightweight liveness probe: no DS1302/spoof/rate reads, so it answers even if the
// status handler faults. If ping works but /api/status does not, the fault is in the
// status build (a hardware-state read), not the web stack or Wi-Fi.
void handlePing() {
  if (!guard(false)) return;
  String out;
  Json(out).open().boolean("ok", true).str("version", FW_VERSION)
      .unum("uptime_s", (unsigned long)(esp_timer_get_time() / 1000000))
      .unum("heap", (unsigned long)ESP.getFreeHeap())
      .unum("status_served", statusServed_).close();
  sendJson(200, out);
}

void handleTemp() {
  if (!guard(true)) return;
  if (server.hasArg("offset")) {
    const float c = server.arg("offset").toFloat();
    if (c < -20.0f || c > 20.0f) { sendError(400, "offset must be -20 to 20 C"); return; }
    tempout::setOffsetC(c);
  } else if (server.hasArg("hold")) {
    const float c = server.arg("hold").toFloat();
    if (c < TEMP_HOLD_MIN_C || c > TEMP_HOLD_MAX_C) { sendError(400, "hold out of range"); return; }
    tempout::holdC(c, TEMP_HOLD_MS);
  } else if (server.hasArg("target")) {
    const String t = server.arg("target");
    if (t == "off") {
      tempout::clearTarget();
    } else {
      const float c = t.toFloat();
      if (c < TEMP_HOLD_MIN_C || c > TEMP_HOLD_MAX_C) { sendError(400, "target out of range"); return; }
      tempout::setTargetC(c);
    }
  } else if (server.hasArg("march") || server.hasArg("sweep")) {
    uint32_t dwellMs = 0;
    if (server.hasArg("dwell")) {
      const long s = server.arg("dwell").toInt();
      if (s < (long)TEMP_DWELL_MIN_S || s > (long)TEMP_DWELL_MAX_S) {
        sendError(400, "dwell must be 5 to 120 seconds"); return;
      }
      dwellMs = (uint32_t)s * 1000UL;
    }
    if (!tempout::startMarch(server.hasArg("sweep"), dwellMs)) {
      sendError(409, "march already running"); return;
    }
  } else if (server.hasArg("raw")) {
    const long v = server.arg("raw").toInt();
    if (v < 0 || v > TEMP_RAW_MAX) { sendError(400, "raw out of range"); return; }
    tempout::setRaw((uint16_t)v);
  } else { sendError(400, "raw, offset, hold, target, march or sweep required"); return; }
  String out;
  Json(out).open().boolean("ok", true).unum("raw", (unsigned long)tempout::raw())
      .boolean("neg", tempout::negActive()).close();
  sendJson(200, out);
}

// The fetch itself never runs here: a blocking TLS session inside a request handler would
// hold the web server and need a second set of TLS buffers at the same time. Queue it and
// let loop() do it, then watch the wx block in /api/status.
void handleWx() {
  if (!guard(true)) return;
  if (server.hasArg("station")) {
    String s = server.arg("station");
    s.trim();
    s.toUpperCase();
    if (!settings::validWxStation(s.c_str())) {
      sendError(400, "station: 3-7 upper-case letters or digits, e.g. CWWB");
      return;
    }
    std::strncpy(g_settings.wxStation, s.c_str(), sizeof g_settings.wxStation - 1);
    g_settings.wxStation[sizeof g_settings.wxStation - 1] = '\0';
    if (!settings::save()) { sendError(500, "could not save station"); return; }
    wx::requestNow();                       // new station, so re-fetch immediately
  } else if (server.hasArg("fetch")) {
    wx::requestNow();
  } else { sendError(400, "station or fetch required"); return; }
  String out;
  Json(out).open().boolean("ok", true).str("station", g_settings.wxStation)
      .boolean("queued", true).close();
  sendJson(200, out);
}

void handleSettings() {
  if (!guard(true)) return;

  String err, ntp, tz;
  const bool haveNtp = server.hasArg("ntp");
  const bool haveTz = server.hasArg("tz");
  uint8_t hf = g_settings.hourFormat;
  uint8_t dst = g_settings.dstMode;
  uint32_t ival = g_settings.syncIntervalS;
  int spoofVal = -1, bootVal = -1;
  int txDbm = g_settings.wifiTxDbm, psMode = g_settings.wifiPsMode;

  if (server.hasArg("txdbm")) {
    const long v = server.arg("txdbm").toInt();
    if (v < 2 || v > 20) err = "Wi-Fi TX power: 2 to 20 dBm";
    else txDbm = settings::quantiseTxDbm((int)v);   // snapped down to a producible step
  }
  if (err.isEmpty() && server.hasArg("psmode")) {
    const long v = server.arg("psmode").toInt();
    if (v < 0 || v > 2) err = "Wi-Fi sleep mode: 0, 1 or 2";
    else psMode = (int)v;
  }

  if (haveNtp) {
    ntp = server.arg("ntp");
    ntp.trim();
    if (!settings::validNtpServer(ntp.c_str())) err = "NTP server: hostname or IPv4 address (letters, digits, dots, hyphens; max 63)";
  }
  if (err.isEmpty() && haveTz) {
    tz = server.arg("tz");
    tz.trim();
    if (!settings::validTz(tz.c_str())) err = "Time zone: POSIX TZ rule, for example EST5EDT,M3.2.0,M11.1.0";
  }
  if (err.isEmpty() && server.hasArg("ival")) {
    const long v = server.arg("ival").toInt();
    if (v < (long)NTP_MIN_INTERVAL_S || v > (long)NTP_MAX_INTERVAL_S) {
      err = "Poll interval: 15 to 86400 seconds";
    } else {
      ival = (uint32_t)v;
    }
  }
  if (err.isEmpty() && server.hasArg("dst") && !settings::parseDstMode(server.arg("dst").c_str(), dst))
    err = "Daylight saving: auto, standard or daylight";
  if (err.isEmpty() && server.hasArg("hrfmt") && !settings::parseHourFormat(server.arg("hrfmt").c_str(), hf))
    err = "Hour format: auto, 24h or 12h";
  if (err.isEmpty() && server.hasArg("spoof")) {
    const String v = server.arg("spoof");
    if (v == "1") spoofVal = 1; else if (v == "0") spoofVal = 0; else err = "spoof: 0 or 1";
  }
  if (err.isEmpty() && server.hasArg("bootcd")) {
    const String v = server.arg("bootcd");
    if (v == "1") bootVal = 1; else if (v == "0") bootVal = 0; else err = "bootcd: 0 or 1";
  }
  if (err.length()) { sendError(400, err.c_str()); return; }

  const bool ntpChanged = haveNtp && std::strcmp(ntp.c_str(), g_settings.ntpServer) != 0;
  const bool tzChanged = (haveTz && std::strcmp(tz.c_str(), g_settings.tz) != 0) || dst != g_settings.dstMode;
  const bool ivalChanged = ival != g_settings.syncIntervalS;

  if (haveNtp) strlcpy(g_settings.ntpServer, ntp.c_str(), sizeof g_settings.ntpServer);
  if (haveTz) strlcpy(g_settings.tz, tz.c_str(), sizeof g_settings.tz);
  g_settings.dstMode = dst;
  g_settings.hourFormat = hf;
  g_settings.syncIntervalS = ival;
  if (spoofVal >= 0) g_settings.spoofEnabled = (spoofVal == 1);
  if (bootVal >= 0) g_settings.bootCountdown = (bootVal == 1);

  const bool radioChanged = txDbm != g_settings.wifiTxDbm || psMode != (int)g_settings.wifiPsMode;
  g_settings.wifiTxDbm = (int8_t)txDbm;
  g_settings.wifiPsMode = (uint8_t)psMode;
  // Applied in place, no reconnect: dropping TX power must not drop the link answering this.
  if (radioChanged) wifimgr::applyRadioPower();

  if (tzChanged) {
    char active[64];
    settings::effectiveTz(active, sizeof active);
    timekeeping::setTz(active);
  }
  if (ntpChanged) timekeeping::setServer(g_settings.ntpServer);
  else if (ivalChanged) timekeeping::setSyncInterval(g_settings.syncIntervalS);

  const bool saved = settings::save();
  String out;
  Json j(out);
  j.open().boolean("ok", saved).boolean("ntp_restarted", ntpChanged || ivalChanged)
      .num("tx_dbm", (long)g_settings.wifiTxDbm)
      .num("tx_dbm_actual", (long)wifimgr::actualTxDbm());
  if (!saved) j.str("error", "NVS write failed: settings are active until reboot");
  j.close();
  sendJson(saved ? 200 : 500, out);
}

void handleTest(uint8_t which) {
  if (!guard(true)) return;
  bool ok = false;
  switch (which) {
    case 0: ok = diag::startPassiveCheck(); break;
    case 1: ok = diag::startMarch(); break;
    case 3: ok = diag::startPinScan(); break;
    case 4: ok = diag::startIsrSelfTest(); break;
    default:
      if (!diag::testRunning()) { spoof::requestCountdown(); ok = true; }
      break;
  }
  if (!ok) { sendError(409, "a test is already running"); return; }
  String out;
  Json(out).open().boolean("ok", true).close();
  sendJson(200, out);
}

void handleSyncNow() {
  if (!guard(true)) return;
  const bool ok = timekeeping::syncNow();
  String out;
  Json j(out);
  j.open().boolean("ok", ok);
  if (!ok) j.str("error", "SNTP not running yet");
  j.close();
  sendJson(ok ? 200 : 503, out);
}

}  // namespace impl

void begin() {
  impl::server.collectHeaders(impl::COLLECT_HEADERS, sizeof impl::COLLECT_HEADERS / sizeof impl::COLLECT_HEADERS[0]);
  impl::server.on("/", HTTP_GET, impl::handleRoot);
  impl::server.on("/api/status", HTTP_GET, impl::handleStatus);
  impl::server.on("/api/ping", HTTP_GET, impl::handlePing);
  impl::server.on("/api/settings", HTTP_POST, impl::handleSettings);
  impl::server.on("/api/sync", HTTP_POST, impl::handleSyncNow);
  impl::server.on("/api/temp", HTTP_POST, impl::handleTemp);
  impl::server.on("/api/wx", HTTP_POST, impl::handleWx);
  impl::server.on("/api/test/passive", HTTP_POST, [] { impl::handleTest(0); });
  impl::server.on("/api/test/march", HTTP_POST, [] { impl::handleTest(1); });
  impl::server.on("/api/test/countdown", HTTP_POST, [] { impl::handleTest(2); });
  impl::server.on("/api/test/pinscan", HTTP_POST, [] { impl::handleTest(3); });
  impl::server.on("/api/test/isrself", HTTP_POST, [] { impl::handleTest(4); });
  impl::server.onNotFound([] { impl::sendError(404, "not found"); });
  impl::server.begin();
}

void loop() { impl::server.handleClient(); }

uint32_t statusServed() { return impl::statusServed_; }

}  // namespace webui

// ================================================================ MAIN

namespace {

uint32_t g_lastNtpKickMs = 0;
bool     g_mdnsStarted = false;

// If SNTP goes quiet for three intervals (or five minutes, whichever is longer) while the
// link is up, kick it. Costs one extra request at most once a minute.
void ntpWatchdog() {
  if (!WiFi.isConnected()) return;
  const uint32_t now = millis();
  if (now - g_lastNtpKickMs < 60000UL) return;
  uint32_t limitS = timekeeping::syncIntervalS() * 3;
  if (limitS < 300) limitS = 300;
  const int64_t age = timekeeping::lastSyncAgeUs();
  if (age < 0 || age > (int64_t)limitS * 1000000LL) {
    timekeeping::syncNow();
    g_lastNtpKickMs = now;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("NtpClockMod " FW_VERSION " starting");
  g_resetReason = esp_reset_reason();
  // Count boots in NVS. "The display occasionally shows 03" is the boot countdown replaying,
  // i.e. the ESP32 restarted - this turns that from an impression into a number, and pairs it
  // with the reason. A rising count with a clean uptime means something is resetting us.
  {
    Preferences p;
    if (p.begin("ntpclock", false)) {
      g_bootCount = p.getULong("boots", 0) + 1;
      p.putULong("boots", g_bootCount);
      p.end();
    }
  }
  Serial.printf("Last reset: %s (code %d) | boot #%lu\n",
                resetReasonName(g_resetReason), (int)g_resetReason,
                (unsigned long)g_bootCount);

  // Bus tap first: I/O is high-Z from here on and the ISR starts listening immediately.
  if (!bus::begin())            Serial.println("ERROR: DS1302 tap: GPIO/ISR setup failed");
  else if (!bus::isrIramSafe()) Serial.println("WARN: GPIO ISR service pre-installed without IRAM flag");

  settings::load();
  char activeTz[64];
  settings::effectiveTz(activeTz, sizeof activeTz);
  Serial.printf("NTP %s every %lu s | TZ %s -> %s | spoof %s\n", g_settings.ntpServer,
                (unsigned long)g_settings.syncIntervalS, g_settings.tz, activeTz,
                g_settings.spoofEnabled ? "on" : "off");

  timekeeping::begin(g_settings.ntpServer, activeTz, g_settings.syncIntervalS);
  spoof::begin();
  tempout::begin();
  wx::begin();
  diag::begin();
  wifimgr::begin();
  webui::begin();
}

void loop() {
  wifimgr::loop();

  if (wifimgr::takeGotIp()) {
    timekeeping::restart();                 // immediate NTP request on every (re)connect
    g_lastNtpKickMs = millis();
    if (!g_mdnsStarted && MDNS.begin(DEVICE_HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      g_mdnsStarted = true;
    }
    Serial.printf("Wi-Fi up: http://%s/  (http://%s.local/)\n",
                  WiFi.localIP().toString().c_str(), DEVICE_HOSTNAME);
  }

  ntpWatchdog();
  webui::loop();
  tempout::loop();
  wx::loop();
  diag::loop();

  // Heartbeat: a repeating startup banner with "up" resetting means the ESP32 is rebooting;
  // "served" stuck at 0 while the web page is open means the status handler is faulting.
  static uint32_t hbMs = 0;
  if (millis() - hbMs >= 5000) {
    hbMs = millis();
    // largest = biggest contiguous block. Fragmentation shows here while "heap" stays flat.
    Serial.printf("[hb] up %lus heap %u largest %u min %u wifi %s served %lu\n",
                  (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap(),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                  (unsigned)ESP.getMinFreeHeap(),
                  WiFi.isConnected() ? "up" : "down", (unsigned long)webui::statusServed());
  }
  delay(2);
}
