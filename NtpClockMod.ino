// NtpClockMod - Andrew Frelas
// NTP-accurate time on the Temu 4-in-1 clock (YM-SZ010-L board) by answering the stock
// MCU's DS1302 reads. Single-file Arduino sketch. Reference: project doc ntp-clock-mod.md
//
// Target : ESP32-C3, Arduino IDE, Espressif "esp32" core 3.3.x, no external libraries
// Board  : "ESP32C3 Dev Module" (or the entry for your C3 board)
//          Tools > USB CDC On Boot: Enabled   (serial log over the C3's native USB)

// ============================================================================ CONFIGURATION

#define FW_VERSION              "01.05.00"

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
#define DEFAULT_NTP_SERVER2     "time.nrc.ca"       // empty string = slot unused
#define DEFAULT_NTP_SERVER3     ""
#define NTP_SERVER_SLOTS        3

// Failover policy. lwIP's own multi-server handling is round-robin and STICKY: one 15 s
// timeout moves to the next slot and it stays there for good, so a single blip permanently
// demotes the primary. So lwIP is given exactly ONE server at a time in slot 0 and the policy
// lives in ntpPolicy(), where it can be reasoned about. With no other slot populated lwIP's
// internal index cannot move at all.
//
// A server that has not answered within NTP_PROBE_S of being selected is passed over. One that
// has answered is judged on ongoing silence instead, over NTP_FAILOVER_AFTER_MULT poll
// intervals floored at NTP_FAILOVER_MIN_S. While running on a fallback the primary is retried
// every NTP_RETURN_PRIMARY_S and keeps the slot if it answers.
//
// NTP_FAILOVER_DEFAULT false gives strict single-server operation. Runtime-settable either way.
#define NTP_FAILOVER_DEFAULT    true
#define NTP_FAILOVER_AFTER_MULT 2
#define NTP_FAILOVER_MIN_S      90UL
#define NTP_RETURN_PRIMARY_S    1800UL
#define NTP_PROBE_S             60UL
#define DEFAULT_TZ              "EST5EDT,M3.2.0,M11.1.0"   // POSIX rule, America/Toronto
#define DEFAULT_SYNC_INTERVAL_S 60                  // NTP poll interval, seconds
#define DEFAULT_DST_MODE        0                   // 0 = follow zone rule, 1 = standard only, 2 = daylight only
#define DEFAULT_HOUR_FORMAT     0                   // 0 = follow the MCU, 1 = 24 hour, 2 = 12 hour

// Firmware-level hour format override. DEFAULT_HOUR_FORMAT above is only the FIRST-BOOT value:
// once anything is in NVS the compiled-in value is never read again, so it cannot pin the
// format on a unit that has already been configured. This one can - it outranks NVS and the
// web UI and is evaluated on every image build.
//   0 = no override, use the stored setting   1 = always 24 hour   2 = always 12 hour
// Worth knowing: "auto" learns from the MCU writing the hours register, and with the DS1302
// removed that only happens if the time is set with the clock's own buttons. On a fresh build
// there is usually nothing to learn from, so auto sits on 24 hour indefinitely.
#define HOUR_FORMAT_FORCE       0
#define DEFAULT_SPOOF_ENABLED   false               // off until the passive bus check passes
#define DEFAULT_BOOT_COUNTDOWN  true                // show 00:00, 03:00, 02:00, 01:00 on first arm

// Poll interval limits. 15 s is the floor enforced by SNTP (RFC 4330).
// Crystal drift. The estimator is two-point: NTP time elapsed against esp_timer elapsed from a
// baseline taken at the first sync. Its noise floor is 2 * jitter / span, so at a 300 s span
// with 10 ms of sync jitter the figure carries about +/- 67 ppm of uncertainty - larger than
// the tens of ppm it is measuring. The old gate declared it valid there, which is why a cold
// boot reported figures like 127 ppm. These are the spans at which it is worth quoting.
#define DRIFT_PPM_LIMIT         250.0f    // discard any estimate outside +/- this: bad data
#define DRIFT_VALID_SPAN_S      3600UL    // baseline before the figure is trusted
#define DRIFT_SYNC_JITTER_US    10000.0f  // assumed per-endpoint sync jitter, for the error bar
#define DRIFT_RECENT_MIN_SPAN_S 1800UL    // shortest window for the recent-rate diagnostic

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
// Whole-degree targeting. The face shows whole degrees, so aiming at a fractional value puts
// the target next to a boundary the MCU's own conversion decides, and a reading sitting at
// 21.9 can land either side of it depending on ripple and temperature. Aiming at a whole
// degree puts the target in the middle of the band that displays that number.
//   0 = send the exact value      1 = truncate toward zero: 21.9 -> 21, -3.7 -> -3
//   2 = round to nearest: 21.9 -> 22, -3.7 -> -4
// 1 and 2 are equally stable; the stability comes from landing on a whole number. 2 is never
// more than half a degree out where 1 can be a full degree, so if the face reads a degree low,
// try 2 before touching the offset. Applied BEFORE the calibration offset, so a sub-degree
// nudge is not thrown away by the rounding.
#define TEMP_WHOLE_DEGREES      1

// Sub-zero handling. The display has no minus sign and the MCU has no notion of a negative
// temperature, so a negative reading is sent as its MAGNITUDE and nothing else: -4 C shows as
// 4. A deliberate loss of information - the alternative was blinking between the magnitude and
// 0 C to hint at the sign, which encodes a meaning the hardware cannot express and reads as a
// fault rather than a minus sign.
//   0 = magnitude only, steady   1 = alternate between the magnitude and 0 C while below zero
#define TEMP_NEG_BLINK          0
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
// Station file naming is not uniform: automatic stations publish as <CODE>-AUTO-swob.xml and
// staffed ones as <CODE>-MAN-swob.xml. 99 of the 871 files in the directory are MAN, so it is
// not an edge case - Pearson is CYYZ-MAN and Hamilton is CYHM-MAN, while Burlington Pier,
// Toronto City and Billy Bishop are all AUTO.
//
// A bare code probes AUTO then MAN with a HEAD request and caches the winner, so the normal
// path stays a single GET. A code containing "-" is taken verbatim as a full stem and never
// probed. A 404 retries the other suffix and re-caches only on a 200, so a station that
// changes category fixes itself and a transient outage cannot corrupt the cache.
#define WX_PATH_SUFFIX          "-AUTO-swob.xml"
#define WX_PATH_SUFFIX_ALT      "-MAN-swob.xml"
#define WX_PROBE_TIMEOUT_MS     8000
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
    phase_ = PH_CMD; bits_ = 0; cmd_ = 0; nBytes_ = 0;
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
    if (!(in & p_.ce)) return endOfTransfer();
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
  // lwIP is handed these by POINTER and keeps them for as long as it runs, so they live here,
  // at file scope, and are never moved or freed.
  char     ntpServer[64];
  char     ntpServer2[64];    // empty = slot unused
  char     ntpServer3[64];
  bool     ntpFailover;       // false = slot 0 only, never try another server
  char     tz[64];            // POSIX rule as chosen in the UI, before the DST mode is applied
  uint32_t syncIntervalS;
  uint8_t  dstMode;
  uint8_t  hourFormat;
  bool     spoofEnabled;
  bool     bootCountdown;
  char     wxStation[16];     // SWOB-ML code (CWWB) or a full stem (CYYZ-MAN)
  char     wxSuffix[16];      // cached winning suffix for a bare code
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

// SWOB-ML station: a bare code (CWWB) or a full stem (CYYZ-MAN). The value is concatenated
// straight into a URL path, so the character set is deliberately ONE character wider than it
// was and no wider: '/', '.' and '%' are a path-traversal surface and stay out. Leading,
// trailing and doubled hyphens are rejected because none of them can name a real file.
bool validWxStation(const char* s) {
  const size_t n = std::strlen(s);
  if (n < 3 || n > 14) return false;
  if (s[0] == '-' || s[n - 1] == '-') return false;
  for (size_t i = 0; i < n; ++i) {
    const char c = s[i];
    if (c == '-') { if (s[i + 1] == '-') return false; continue; }
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) return false;
  }
  return true;
}

// True when the value already names a complete stem and must not be probed.
bool wxStationIsStem(const char* s) { return std::strchr(s, '-') != nullptr; }

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
  impl::copyStr(g_settings.ntpServer2, sizeof g_settings.ntpServer2, DEFAULT_NTP_SERVER2);
  impl::copyStr(g_settings.ntpServer3, sizeof g_settings.ntpServer3, DEFAULT_NTP_SERVER3);
  g_settings.ntpFailover = NTP_FAILOVER_DEFAULT;
  impl::copyStr(g_settings.wxSuffix, sizeof g_settings.wxSuffix, WX_PATH_SUFFIX);
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
  // getString returns 0 WITHOUT writing to buf on a read error, and buf is shared across every
  // read here, so a failure would otherwise leave the PREVIOUS key's value in place - the
  // primary's hostname, which validates fine and turns slot 1 into a duplicate of slot 0.
  // Failing over to the same dead host is worse than having no fallback. Clearing buf first
  // makes a read error indistinguishable from an empty value, which is the safe reading.
  if (p.isKey("ntp2")) {
    buf[0] = '\0'; p.getString("ntp2", buf, sizeof buf);
    if (buf[0] == '\0' || validNtpServer(buf))
      impl::copyStr(g_settings.ntpServer2, sizeof g_settings.ntpServer2, buf);
  }
  if (p.isKey("ntp3")) {
    buf[0] = '\0'; p.getString("ntp3", buf, sizeof buf);
    if (buf[0] == '\0' || validNtpServer(buf))
      impl::copyStr(g_settings.ntpServer3, sizeof g_settings.ntpServer3, buf);
  }
  g_settings.ntpFailover = p.getBool("ntpfo", NTP_FAILOVER_DEFAULT);
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
  if (p.isKey("wxsfx") && p.getString("wxsfx", buf, sizeof buf) > 0 &&
      (!std::strcmp(buf, WX_PATH_SUFFIX) || !std::strcmp(buf, WX_PATH_SUFFIX_ALT)))
    impl::copyStr(g_settings.wxSuffix, sizeof g_settings.wxSuffix, buf);
  g_settings.wifiTxDbm = quantiseTxDbm(p.getChar("wtx", WIFI_TX_DBM_DEFAULT));
  const uint8_t ps = p.getUChar("wps", WIFI_PS_MODE_DEFAULT);
  g_settings.wifiPsMode = ps <= 2 ? ps : (uint8_t)0;
  p.end();
}

bool save() {
  Preferences p;
  if (!p.begin(impl::NVS_NS, false)) return false;
  bool ok = p.putString("ntp", g_settings.ntpServer) > 0;
  // Preferences::putString returns size_t: strlen(value) on success and 0 on FAILURE, so an
  // empty value and a failed write return the same thing. The only sound test is against the
  // length that was asked for.
  ok = (p.putString("ntp2", g_settings.ntpServer2) == std::strlen(g_settings.ntpServer2)) && ok;
  ok = (p.putString("ntp3", g_settings.ntpServer3) == std::strlen(g_settings.ntpServer3)) && ok;
  ok = p.putBool("ntpfo", g_settings.ntpFailover) > 0 && ok;
  ok = p.putString("tz", g_settings.tz) > 0 && ok;
  ok = p.putULong("ival", g_settings.syncIntervalS) > 0 && ok;
  ok = p.putUChar("dst", g_settings.dstMode) > 0 && ok;
  ok = p.putUChar("hrfmt", g_settings.hourFormat) > 0 && ok;
  ok = p.putBool("spoof", g_settings.spoofEnabled) > 0 && ok;
  ok = p.putBool("bootcd", g_settings.bootCountdown) > 0 && ok;
  ok = p.putString("wxstn", g_settings.wxStation) > 0 && ok;
  ok = p.putString("wxsfx", g_settings.wxSuffix) > 0 && ok;
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
// Mid-point sample. The long baseline gives the low-noise figure; a second sample taken part
// way along gives a recent-window rate, which is what moves when the board's temperature does.
// Without it a rate shift takes as long to show as the whole baseline, because a two-point
// estimate from a fixed origin averages everything since boot.
int64_t           midNtpUs_ = 0;
int64_t           midEspUs_ = 0;
bool              haveMid_ = false;
float             recentPpm_ = 0.0f;
uint32_t          recentSpanS_ = 0;
uint8_t           activeSlot_ = 0;      // which configured server is currently installed
uint32_t          slotSinceMs_ = 0;     // millis() when it was installed
uint32_t          slotSyncCount_ = 0;   // syncCount_ then; proves THIS server replied
float             driftPpm_ = 0.0f;     // crystal fast(+) / slow(-) versus NTP
uint32_t          driftSpanS_ = 0;      // baseline length the figure is measured over
uint32_t          syncCount_ = 0;
uint32_t          intervalS_ = DEFAULT_SYNC_INTERVAL_S;

constexpr time_t PLAUSIBLE_EPOCH = 1735689600;   // 2025-01-01T00:00:00Z

// Crystal drift: esp_timer measures elapsed time on the local crystal, NTP measures the real
// elapsed time. The difference over a long baseline is the crystal error.
//
// Every division below is OUTSIDE the lock, deliberately. This chip has no FPU, so each double
// operation is a libgcc call in flash costing tens of microseconds, usually cold - and
// portENTER_CRITICAL raises the interrupt threshold above BUS_INTR_LEVEL, masking the DS1302
// CE ISR for the whole window. A CE rise landing in it loses SCLK edges. Only the stores are
// protected. Reading the baselines unlocked is safe: this callback is their only writer.
void onSync(struct timeval* tv) {                // runs in the lwIP task
  const int64_t now = esp_timer_get_time();
  const int64_t ntpUs = (int64_t)tv->tv_sec * 1000000LL + (int64_t)tv->tv_usec;

  bool  haveLong = false, haveRecent = false, rollMid = false, setBase = false;
  float longPpm = 0.0f, recentPpm = 0.0f;
  uint32_t longSpan = 0, recentSpan = 0;

  if (!haveBase_) {
    setBase = true;
  } else {
    const int64_t dNtp = ntpUs - baseNtpUs_;
    const int64_t dEsp = now - baseEspUs_;
    if (dNtp > 0) {
      const float ppm = (float)((double)(dEsp - dNtp) * 1000000.0 / (double)dNtp);
      // Outside the sanity limit means a bad packet or a stepped clock, not a real rate.
      if (ppm > -DRIFT_PPM_LIMIT && ppm < DRIFT_PPM_LIMIT) {
        longPpm = ppm; longSpan = (uint32_t)(dNtp / 1000000LL); haveLong = true;
      }
    } else {
      // The baseline is ahead of this reply, so it came from a bad packet - SNTP steps the
      // clock to whatever arrives and nothing upstream range-checks it. Left alone this is
      // permanent: dNtp stays negative for the life of the boot and no figure is ever produced
      // again. Re-anchor here instead.
      setBase = true;
    }
    if (haveMid_) {
      const int64_t rNtp = ntpUs - midNtpUs_;
      const int64_t rEsp = now - midEspUs_;
      if (rNtp >= (int64_t)DRIFT_RECENT_MIN_SPAN_S * 1000000LL) {
        const float ppm = (float)((double)(rEsp - rNtp) * 1000000.0 / (double)rNtp);
        if (ppm > -DRIFT_PPM_LIMIT && ppm < DRIFT_PPM_LIMIT) {
          recentPpm = ppm; recentSpan = (uint32_t)(rNtp / 1000000LL); haveRecent = true;
        }
        rollMid = true;
      } else if (rNtp < 0) {
        rollMid = true;
      }
    } else {
      rollMid = true;
    }
  }

  portENTER_CRITICAL(&mux_);
  lastSyncEpoch_ = tv->tv_sec;
  lastSyncUs_ = now;
  ++syncCount_;
  if (setBase)    { baseNtpUs_ = ntpUs; baseEspUs_ = now; haveBase_ = true; driftSpanS_ = 0; }
  if (haveLong)   { driftPpm_ = longPpm; driftSpanS_ = longSpan; }
  if (haveRecent) { recentPpm_ = recentPpm; recentSpanS_ = recentSpan; }
  if (rollMid)    { midNtpUs_ = ntpUs; midEspUs_ = now; haveMid_ = true; }
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
// The configured server for one of our slots, or an empty string if that slot is unused.
const char* slotServer(uint8_t i) {
  if (i == 1) return g_settings.ntpServer2;
  if (i == 2) return g_settings.ntpServer3;
  return g_settings.ntpServer;
}

bool slotUsable(uint8_t i) {
  if (i > 0 && !g_settings.ntpFailover) return false;   // failover off: only slot 0 exists
  return slotServer(i)[0] != '\0';
}

// (Re)start SNTP with exactly ONE server - whichever slot is active - in lwIP slot 0, every
// other slot cleared. lwIP's own selection is round-robin and sticky, so a single timeout on
// the primary would demote it permanently; with no other slot populated its index cannot move
// and the policy is entirely ours. The pointer lwIP keeps is always impl::server_, one buffer
// that outlives every restart, so a settings edit cannot move a string it may be resolving.
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

// Install one of our slots as the single lwIP server.
void selectSlot(uint8_t slot) {
  if (slot >= NTP_SERVER_SLOTS) slot = 0;
  const char* name = slotServer(slot);
  if (name[0] == '\0') { slot = 0; name = slotServer(0); }
  if (name[0] == '\0') return;                     // nothing configured at all
  if (esp_sntp_enabled()) esp_sntp_stop();
  std::strncpy(impl::server_, name, sizeof impl::server_ - 1);
  impl::server_[sizeof impl::server_ - 1] = '\0';
  impl::activeSlot_ = slot;
  impl::slotSinceMs_ = millis();
  portENTER_CRITICAL(&impl::mux_);
  impl::slotSyncCount_ = impl::syncCount_;
  portEXIT_CRITICAL(&impl::mux_);
  restart();
}

uint8_t  activeSlot()    { return impl::activeSlot_; }
uint32_t slotSinceMs()   { return impl::slotSinceMs_; }
uint32_t slotSyncCount() { return impl::slotSyncCount_; }

// The next usable slot after `from`, wrapping. Returns `from` when nothing else is configured,
// so a lone-server setup degenerates into a plain restart rather than a special case.
uint8_t nextUsableSlot(uint8_t from) {
  for (uint8_t step = 1; step <= NTP_SERVER_SLOTS; ++step) {
    const uint8_t cand = (uint8_t)((from + step) % NTP_SERVER_SLOTS);
    if (slotUsable(cand)) return cand;
  }
  return from;
}

// Editing a server is an explicit statement of intent, so it re-anchors on the primary rather
// than leaving the device on a fallback it was sent to before the change.
void setServer(const char* ntpServer) {
  if (esp_sntp_enabled()) esp_sntp_stop();         // stop before the buffer changes
  std::strncpy(impl::server_, ntpServer, sizeof impl::server_ - 1);
  impl::server_[sizeof impl::server_ - 1] = '\0';
  impl::activeSlot_ = 0;
  impl::slotSinceMs_ = millis();
  portENTER_CRITICAL(&impl::mux_);
  impl::slotSyncCount_ = impl::syncCount_;
  portEXIT_CRITICAL(&impl::mux_);
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
// Valid once the baseline is long enough for the figure to beat its own noise. The old gate
// was 300 s, where a two-point estimate carries about +/- 67 ppm of uncertainty - larger than
// the quantity it measures, which is why a cold boot used to report figures like 127 ppm.
bool     driftValid() { return impl::driftSpanS_ >= DRIFT_VALID_SPAN_S; }
float    driftRecentPpm()   { return impl::recentPpm_; }
uint32_t driftRecentSpanS() { return impl::recentSpanS_; }

// One-sigma uncertainty of the current estimate, in ppm. NEGATIVE when nothing has been
// measured at all: returning 0 there reads exactly like a perfectly measured zero.
float driftUncertaintyPpm() {
  const uint32_t span = impl::driftSpanS_;
  if (span == 0) return -1.0f;
  return (2.0f * DRIFT_SYNC_JITTER_US) / (float)span;
}
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
#if HOUR_FORMAT_FORCE == 1
  return ds1302::HourMode::H24;              // pinned in firmware, outranks NVS and the UI
#elif HOUR_FORMAT_FORCE == 2
  return ds1302::HourMode::H12;
#else
  if (g_settings.hourFormat == HOURFMT_12) return ds1302::HourMode::H12;
  if (g_settings.hourFormat == HOURFMT_24) return ds1302::HourMode::H24;
  return bus::learnedHour() == 1 ? ds1302::HourMode::H12 : ds1302::HourMode::H24;
#endif
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
  31,139,8,0,0,0,0,0,2,255,197,125,253,123,218,198,178,255,239,252,
  21,91,218,147,64,3,50,224,151,56,96,236,235,56,238,73,218,56,241,19,
  147,166,167,125,122,137,64,11,40,22,18,213,74,198,142,195,253,219,191,159,
  153,93,9,9,132,67,114,239,243,124,79,78,49,90,205,236,204,206,206,206,
  203,190,113,244,131,19,12,163,187,153,20,147,104,234,29,151,142,232,143,240,
  108,127,220,45,75,191,124,124,52,145,182,115,124,52,149,145,45,134,19,59,
  84,50,234,150,227,104,84,63,44,155,82,223,158,202,110,249,198,149,243,89,
  16,70,101,49,12,252,72,250,128,154,187,78,52,233,58,242,198,29,202,58,
  63,212,92,223,141,92,219,171,171,161,237,201,110,179,12,122,145,27,121,242,
  248,77,239,82,156,121,193,240,90,92,4,206,209,142,46,44,29,169,232,142,
  254,182,195,32,136,238,235,245,193,184,253,227,104,15,255,154,157,122,125,104,
  135,14,30,71,35,124,31,225,69,115,136,127,54,30,166,113,212,254,241,192,
  198,191,61,60,121,174,47,219,63,202,134,108,56,244,50,184,6,164,243,212,
  222,125,134,135,185,29,250,237,31,159,237,29,180,26,13,60,14,108,84,56,
  216,107,237,54,15,241,100,15,135,4,186,39,157,195,69,233,191,166,210,113,
  237,202,44,148,35,25,170,250,48,240,130,16,173,152,200,169,108,59,118,120,
  93,189,207,242,216,220,197,191,86,202,99,211,193,191,129,97,83,14,240,239,
  48,97,243,217,0,255,246,83,54,119,91,187,173,150,52,108,238,217,142,60,
  108,164,108,142,6,131,81,107,47,97,115,116,248,180,249,180,153,176,249,212,
  182,91,163,209,98,81,250,249,126,16,220,214,149,251,217,245,199,237,65,16,
  58,50,172,163,100,49,8,156,187,251,169,29,142,93,191,221,232,140,208,69,
  237,230,222,236,118,167,105,237,237,11,117,167,34,57,173,199,110,173,110,207,
  102,158,172,235,130,218,149,28,7,82,188,127,85,83,182,175,234,74,134,238,
  168,51,176,135,215,227,48,136,125,167,125,99,135,21,106,111,181,195,226,48,
  207,163,113,117,81,154,218,174,15,114,183,186,219,219,135,135,141,217,109,39,
  33,47,236,56,10,58,51,219,113,136,201,230,193,236,118,81,34,45,147,225,
  189,227,170,153,103,223,181,71,158,188,237,208,71,125,30,218,179,54,125,116,
  108,207,29,251,117,23,140,169,246,16,10,38,195,206,24,239,128,46,168,41,
  73,245,248,42,26,194,212,218,188,167,166,146,60,100,187,249,52,195,195,226,
  199,33,105,27,191,110,31,52,26,162,69,245,196,110,125,26,248,129,154,217,
  67,89,187,144,190,23,212,206,2,95,5,158,173,106,233,139,69,201,162,110,
  189,95,147,4,149,86,59,90,232,237,38,170,3,162,235,8,253,146,250,55,
  121,89,15,109,199,141,85,187,73,82,73,229,64,140,19,219,134,71,116,91,
  20,5,83,46,71,75,90,217,150,180,0,21,201,219,168,30,133,232,153,81,
  16,78,219,241,108,38,195,161,173,100,199,147,17,100,83,39,94,169,94,171,
  241,84,78,115,61,4,197,171,46,59,163,33,14,137,64,100,15,60,121,175,
  187,171,217,104,252,43,97,21,136,158,61,83,178,157,124,89,68,206,125,194,
  243,62,201,186,115,35,195,200,197,128,174,115,15,181,163,96,150,32,227,107,
  177,32,64,47,108,143,220,80,69,245,225,196,245,28,129,74,51,56,13,16,
  201,190,190,95,99,95,51,186,123,240,175,68,126,245,208,29,79,34,22,13,
  58,136,58,75,11,108,100,79,93,239,174,189,85,207,118,2,52,101,228,5,
  115,173,117,182,127,55,159,200,144,58,28,67,110,44,211,102,131,134,120,134,
  46,200,247,230,51,42,90,233,36,126,156,75,102,13,90,182,174,28,195,56,
  12,161,203,103,212,62,208,9,252,92,83,131,235,234,194,10,70,163,181,246,
  47,44,178,9,185,98,42,64,57,56,205,21,227,25,210,246,236,129,244,210,
  209,53,32,213,79,52,128,148,16,74,64,35,104,141,74,201,245,103,113,244,
  23,185,135,46,233,219,223,181,76,129,31,79,7,50,252,187,166,164,39,135,
  81,86,117,18,57,29,166,50,218,106,56,144,238,111,99,94,180,249,114,125,
  244,141,27,209,104,156,92,231,13,7,153,5,162,93,96,49,86,171,50,99,
  141,21,149,7,154,21,6,243,245,218,120,164,126,221,30,101,107,211,138,56,
  14,93,39,173,142,30,58,244,81,7,18,74,34,73,195,43,158,250,48,5,
  163,80,224,63,38,150,88,47,227,115,150,102,20,110,106,118,91,189,215,149,
  110,172,7,94,96,16,195,116,248,247,89,57,101,59,69,219,203,2,225,23,
  247,21,60,76,117,189,99,184,84,139,147,157,48,52,89,225,251,44,112,73,
  20,9,15,150,146,195,172,165,100,131,53,179,73,231,115,125,65,181,45,52,
  74,27,226,34,99,228,220,7,100,194,162,187,54,124,84,82,189,35,71,118,
  236,161,215,225,137,239,231,19,72,159,13,157,108,227,153,251,166,88,131,190,
  81,11,83,155,156,113,91,102,156,36,206,179,197,206,115,127,91,151,225,32,
  82,114,61,37,84,60,69,125,119,247,121,105,109,50,207,90,147,64,120,97,
  249,65,36,239,55,89,241,195,12,107,198,250,236,178,250,33,22,243,239,51,
  166,158,59,62,105,221,51,210,132,214,154,38,28,230,205,216,110,145,98,172,
  152,45,34,99,201,48,92,183,60,250,213,6,99,85,58,218,209,1,222,209,
  142,142,50,41,74,65,84,137,224,129,2,81,14,9,16,128,54,243,209,161,
  56,130,76,125,225,58,8,58,101,136,120,19,162,86,221,50,204,100,89,112,
  117,221,114,214,236,238,53,86,229,130,192,117,135,170,32,178,77,10,50,147,
  234,56,36,40,31,215,235,109,254,127,2,149,190,103,47,144,18,228,39,65,
  100,143,45,203,90,86,169,217,46,29,57,238,141,174,21,18,72,145,244,195,
  196,117,28,73,192,128,57,46,129,3,24,80,55,240,83,32,4,18,20,121,
  183,142,175,34,59,138,21,42,109,29,31,177,135,166,128,25,50,137,156,227,
  215,1,92,174,136,220,169,68,184,236,80,17,83,139,184,60,165,71,42,136,
  22,105,144,157,40,92,226,247,128,41,62,7,254,10,122,239,243,22,184,31,
  220,250,47,110,30,239,131,59,114,139,129,169,243,16,59,162,175,242,24,111,
  162,217,22,164,222,65,41,131,60,34,23,109,129,250,218,86,145,96,234,119,
  254,48,95,197,21,74,138,145,46,164,173,226,80,58,194,65,180,27,229,177,
  94,80,209,3,180,66,137,236,104,133,87,42,218,68,105,26,132,119,121,112,
  148,109,211,117,176,250,34,136,35,184,226,149,206,195,139,45,240,175,102,65,
  48,90,17,8,21,61,160,40,10,163,93,68,129,136,38,82,92,156,189,207,
  35,191,154,218,153,113,81,64,119,71,235,46,198,136,86,244,135,117,30,241,
  43,204,147,209,122,10,110,153,202,168,108,198,148,65,32,255,103,138,240,201,
  1,142,0,112,183,236,67,175,114,74,199,239,0,195,177,139,224,216,165,76,
  209,76,153,235,37,112,1,31,235,73,127,140,116,181,124,176,91,230,4,101,
  24,192,181,202,72,38,150,101,38,61,15,249,222,240,26,172,216,158,146,68,
  91,143,223,117,22,220,27,12,65,230,97,134,168,89,176,141,71,145,168,160,
  209,129,239,168,106,49,83,58,162,210,108,113,21,98,234,250,221,114,115,159,
  25,236,150,15,15,96,204,200,200,201,25,74,31,98,0,141,106,105,6,192,
  171,71,46,81,52,69,101,128,180,254,90,116,69,236,199,74,58,213,45,36,
  211,250,63,23,13,42,221,93,225,172,245,61,156,237,254,111,57,219,201,178,
  150,40,225,4,230,63,71,147,145,145,64,167,116,71,208,237,247,74,242,64,
  72,91,160,53,77,137,10,168,162,21,244,110,22,186,228,232,133,237,59,2,
  190,123,2,141,22,18,28,100,218,54,75,200,146,111,47,195,214,135,238,48,
  74,17,231,110,52,1,5,215,163,164,164,70,117,8,142,106,240,57,112,125,
  75,188,245,165,161,43,92,37,72,108,194,142,132,205,30,193,18,167,37,95,
  206,189,59,161,131,115,188,51,160,19,91,137,131,134,80,52,150,67,57,3,
  196,64,162,95,116,123,124,8,152,42,3,35,210,233,136,192,31,74,225,70,
  140,131,168,109,46,201,44,186,4,81,250,20,195,247,57,128,192,255,199,1,
  181,77,185,232,11,192,19,183,34,154,7,121,205,87,150,248,128,68,78,138,
  48,246,125,2,7,166,189,148,95,86,96,168,61,148,204,129,144,168,235,78,
  236,54,104,24,196,145,84,44,204,107,41,103,138,49,148,7,153,184,35,98,
  73,179,167,224,133,103,235,130,237,1,150,20,238,50,8,60,97,171,107,69,
  154,8,129,138,169,110,56,188,251,40,136,67,129,194,145,123,35,197,63,49,
  2,102,166,38,38,40,183,196,171,164,21,208,79,248,118,97,15,96,124,197,
  179,6,228,88,66,40,11,17,194,248,235,80,206,116,130,199,206,121,105,132,
  68,5,125,23,81,63,190,57,189,170,129,76,40,231,104,124,85,132,54,90,
  18,106,38,108,49,139,7,158,59,68,85,129,103,154,146,25,57,209,103,216,
  181,140,215,214,106,116,164,123,88,27,98,130,32,35,75,37,91,90,203,232,
  115,249,248,242,237,213,171,63,68,239,79,244,142,39,191,54,246,128,240,127,
  109,19,28,5,23,249,194,190,243,40,96,19,202,190,129,134,44,217,200,52,
  144,1,75,71,193,140,61,7,122,36,6,61,162,94,62,254,5,234,22,204,
  89,47,72,60,220,20,81,25,66,174,8,210,162,185,11,213,180,197,157,180,
  67,12,64,141,191,86,145,138,160,95,236,134,222,142,70,109,145,60,242,136,
  18,232,45,70,223,136,237,24,254,13,118,242,184,17,123,217,77,121,131,180,
  77,167,77,194,209,20,132,154,45,177,35,90,123,172,165,194,36,153,133,114,
  51,240,133,130,59,197,103,27,213,166,226,131,123,23,149,164,214,216,143,92,
  143,70,216,28,57,164,212,227,46,148,99,23,62,40,35,201,149,138,91,123,
  147,242,177,169,97,19,76,179,53,225,6,228,97,214,132,82,208,116,21,34,
  116,123,69,99,54,68,167,250,193,252,107,26,171,81,192,182,237,4,62,44,
  30,130,17,23,227,24,14,181,222,92,119,8,43,198,131,196,35,60,116,157,
  175,27,79,241,8,134,248,40,12,166,169,180,72,54,100,212,232,153,218,163,
  82,17,193,236,145,25,167,23,47,174,154,187,141,22,108,219,20,38,210,161,
  17,31,9,102,103,98,207,102,18,181,195,146,17,28,235,11,140,48,226,70,
  237,3,168,144,51,147,199,74,4,115,95,232,44,89,213,144,138,25,51,138,
  184,115,130,98,215,115,74,100,77,164,118,8,49,148,238,46,245,61,73,43,
  52,231,100,72,169,251,73,201,239,20,85,99,186,203,18,151,46,5,11,45,
  178,134,84,70,213,105,35,85,98,35,53,183,117,91,201,132,218,166,74,110,
  203,212,6,49,178,216,2,166,128,60,144,230,72,12,201,178,121,224,117,230,
  250,172,72,212,166,151,111,223,191,235,255,242,246,221,197,105,143,254,156,157,
  151,80,3,183,62,64,100,171,5,161,174,101,52,156,212,4,178,252,225,132,
  194,221,16,49,2,117,130,22,14,113,97,140,228,86,118,238,214,25,76,203,
  58,115,17,60,11,49,117,201,104,207,179,17,98,214,150,106,248,213,81,222,
  106,64,179,27,194,121,62,21,117,65,9,115,32,204,148,68,13,242,139,48,
  68,162,141,22,162,121,8,141,63,36,220,205,32,7,0,57,120,24,100,31,
  32,251,15,131,236,1,100,239,97,16,4,97,205,221,135,65,48,56,154,77,
  211,212,103,248,139,172,31,173,212,206,175,185,115,168,189,182,22,224,134,42,
  208,224,135,219,251,180,124,252,244,65,0,52,246,225,182,34,204,109,25,30,
  17,37,184,211,88,15,75,221,53,246,112,40,103,145,122,200,234,174,41,202,
  76,77,3,71,38,154,66,223,167,136,52,16,116,20,170,73,2,189,202,86,
  131,221,64,170,35,182,55,215,227,108,179,180,203,199,23,24,32,117,49,179,
  195,107,68,20,209,92,74,95,188,232,189,186,192,131,141,172,65,61,36,130,
  11,251,22,168,30,130,49,10,69,152,221,26,197,71,115,122,196,48,215,241,
  209,22,222,103,61,116,66,172,107,155,248,144,194,58,79,114,164,203,163,88,
  222,184,14,7,125,3,9,27,67,54,13,178,34,195,64,192,131,128,124,103,
  18,239,97,108,88,165,222,31,90,95,200,38,80,60,27,5,241,144,12,151,
  34,163,71,195,153,166,119,18,113,163,59,225,62,217,202,217,163,136,98,164,
  56,228,170,92,178,155,176,67,48,14,102,238,38,151,120,210,116,168,80,67,
  239,90,72,196,168,102,90,166,38,10,32,57,84,29,80,36,27,25,48,178,
  139,69,117,146,77,134,214,171,4,12,29,245,28,92,59,174,61,246,3,21,
  185,67,5,233,163,209,119,98,26,147,252,97,84,41,14,252,44,195,192,18,
  103,49,219,42,209,251,163,164,91,207,214,16,99,73,65,42,115,49,71,160,
  196,2,35,101,85,74,199,145,176,187,118,168,150,166,191,38,6,119,218,40,
  218,240,14,58,79,36,59,233,60,207,197,137,91,166,49,74,39,251,156,243,
  11,233,243,100,107,146,187,204,80,67,52,65,180,58,158,180,141,183,71,36,
  171,157,87,18,102,103,114,152,111,162,59,8,130,104,8,243,252,28,127,225,
  37,16,91,56,228,208,2,159,9,153,0,70,84,26,141,118,163,81,19,141,
  93,253,167,165,255,52,241,39,67,56,99,243,195,96,14,178,218,45,26,186,
  42,30,192,188,155,214,218,55,148,93,225,243,104,71,3,101,166,244,70,83,
  53,206,204,7,154,177,64,78,126,203,217,138,183,49,26,65,42,47,167,51,
  9,71,25,99,172,140,164,116,120,246,98,109,60,157,251,55,110,24,248,83,
  154,77,33,31,124,230,33,233,129,190,158,233,56,245,204,246,109,199,22,87,
  31,222,62,175,95,188,174,9,10,100,161,76,80,48,164,12,136,103,65,65,
  63,7,190,85,122,161,253,142,56,251,240,225,57,121,252,231,113,232,65,207,
  72,8,151,46,0,235,90,97,52,184,56,63,59,59,51,1,196,91,251,250,
  198,245,168,58,27,52,145,57,34,150,24,200,161,29,235,180,150,227,135,146,
  31,164,168,238,18,5,105,231,128,50,26,243,66,113,204,224,221,117,16,10,
  80,120,197,153,157,231,36,9,13,162,59,130,9,229,39,157,131,102,227,8,
  211,219,210,49,10,188,50,187,249,11,139,48,51,201,52,167,169,80,249,192,
  228,91,144,97,76,193,118,230,177,193,246,54,19,99,104,14,66,150,129,92,
  209,201,124,93,31,8,104,219,249,199,17,5,49,121,252,95,168,168,24,229,
  140,6,5,70,88,30,129,75,183,32,248,82,218,51,152,145,48,128,25,41,
  160,75,175,183,152,165,219,24,79,101,189,228,92,69,126,153,167,167,121,84,
  192,108,167,9,169,30,255,76,145,128,114,225,120,38,113,124,186,109,222,152,
  245,212,89,22,6,31,152,135,71,254,64,205,58,41,117,99,6,216,220,48,
  64,210,18,140,96,109,5,18,189,94,90,131,141,4,124,178,44,15,17,32,
  128,28,1,238,91,157,148,172,212,94,236,96,19,9,122,72,23,144,52,58,
  214,20,178,8,172,241,208,26,218,59,81,128,44,114,39,163,216,106,199,9,
  134,59,106,30,12,234,183,83,175,111,26,210,39,100,107,168,110,138,102,63,
  116,71,220,94,144,153,171,107,128,173,236,90,47,99,207,146,169,230,34,139,
  246,166,119,70,1,10,6,183,78,108,58,226,223,151,175,222,62,165,233,243,
  27,169,114,201,203,233,139,51,72,198,33,35,195,254,69,180,16,73,7,147,
  105,77,71,19,45,171,37,226,95,16,20,148,244,242,157,64,104,149,36,47,
  168,229,242,195,133,37,94,179,3,229,184,7,190,202,211,79,129,23,145,33,
  235,66,13,201,246,88,226,119,83,162,38,100,238,96,129,76,114,49,179,157,
  90,201,4,19,19,158,61,163,105,30,100,79,144,95,64,6,19,161,133,210,
  38,96,74,225,5,172,26,88,166,30,9,181,97,180,196,149,164,233,29,102,
  160,198,38,111,217,66,49,162,37,192,146,153,240,147,134,162,27,62,156,164,
  228,114,148,208,134,190,189,179,231,166,133,149,70,125,233,169,240,14,81,30,
  194,225,102,211,184,171,106,193,144,227,42,86,166,147,121,18,185,97,230,144,
  247,41,172,223,168,241,188,128,240,144,198,231,86,24,244,144,130,68,18,5,
  249,202,128,138,120,177,12,113,49,229,182,149,71,142,28,119,206,10,27,193,
  166,32,223,8,61,233,221,176,154,166,61,245,86,210,34,74,201,54,54,232,
  45,81,124,160,61,244,126,221,66,4,204,226,215,219,51,129,194,149,143,95,
  226,51,105,78,77,236,18,127,133,173,98,232,13,205,218,79,154,181,159,52,
  235,224,161,102,189,100,194,15,180,139,1,114,13,99,46,57,121,102,229,218,
  100,159,182,81,83,135,98,86,234,121,94,204,48,81,201,50,65,202,182,89,
  131,22,41,100,178,168,209,108,101,150,52,178,105,246,122,227,87,27,156,13,
  212,151,198,142,73,190,73,103,126,245,216,68,175,134,48,9,10,67,92,7,
  65,38,92,209,249,5,211,55,195,57,8,4,98,111,239,142,246,64,162,113,
  73,38,70,185,15,51,167,172,213,104,209,24,246,245,14,160,150,229,116,139,
  226,125,216,60,173,42,156,41,177,182,164,125,81,202,143,181,11,59,68,160,
  144,171,34,107,150,167,244,122,3,238,21,152,94,25,168,100,87,20,21,139,
  74,232,210,54,63,193,126,182,154,209,132,130,201,115,230,65,144,234,42,200,
  5,95,89,84,73,146,199,178,166,41,1,12,26,50,124,179,16,201,139,153,
  37,76,205,24,47,25,32,162,178,4,245,7,7,25,84,0,39,89,186,56,
  63,189,122,255,238,252,133,142,183,80,200,89,163,233,148,80,214,145,62,38,
  22,153,123,135,87,11,144,13,236,213,196,97,77,52,119,241,31,254,182,90,
  248,239,41,134,30,254,238,62,133,29,183,253,59,118,28,37,90,113,129,1,
  71,222,130,42,97,200,217,155,184,58,182,67,172,237,5,99,152,104,248,30,
  61,79,236,88,226,223,180,0,96,244,129,167,251,98,180,175,101,22,76,104,
  214,201,147,102,197,164,148,204,250,161,121,148,198,116,214,38,234,180,206,17,
  136,103,143,225,132,196,192,29,139,79,241,116,86,188,76,177,236,31,162,174,
  68,131,72,50,125,39,142,238,208,210,125,161,187,64,203,145,183,199,177,208,
  61,155,115,78,194,225,201,3,118,60,174,95,226,9,70,201,25,187,87,51,
  139,41,220,176,84,167,41,95,39,247,168,147,60,245,79,76,153,254,28,124,
  195,209,69,144,247,93,18,7,75,26,10,68,139,216,215,50,164,169,70,40,
  175,227,120,137,132,169,224,221,25,101,42,145,94,148,154,186,74,113,74,16,
  242,242,149,19,104,113,41,241,41,24,112,122,98,179,102,152,48,128,250,69,
  37,43,47,78,201,132,3,233,68,235,140,7,167,208,177,129,94,15,179,73,
  8,211,57,49,173,59,176,88,176,207,37,77,109,167,195,46,155,107,154,137,
  8,10,20,80,27,45,51,65,81,220,177,207,26,200,13,68,134,239,70,177,
  99,166,100,253,168,228,250,138,68,99,102,118,7,72,185,174,33,165,90,146,
  43,115,14,100,147,4,124,155,38,204,51,178,206,84,70,162,108,180,181,162,
  145,152,15,169,250,39,135,20,61,136,67,45,24,39,12,102,172,117,13,134,
  102,205,7,80,253,208,18,108,202,109,225,203,49,226,63,232,171,30,102,122,
  20,2,129,167,70,32,103,107,99,60,72,201,234,55,71,132,43,51,30,133,
  155,82,206,206,5,176,104,63,40,130,85,177,35,242,137,140,115,182,33,129,
  187,58,123,253,27,77,99,255,19,75,127,152,207,182,156,151,159,139,113,46,
  3,207,227,117,162,44,48,21,62,144,140,33,217,152,66,152,121,28,122,179,
  109,54,71,29,156,199,126,135,146,109,177,121,13,37,143,254,129,138,182,217,
  6,147,153,133,89,219,243,67,76,108,179,229,231,37,173,232,232,5,140,21,
  25,227,69,49,202,171,171,119,121,208,87,42,220,152,54,234,61,110,240,203,
  122,147,91,154,205,98,72,241,44,33,194,121,168,154,121,121,100,28,1,235,
  133,79,171,83,233,46,145,164,162,173,83,21,21,41,147,157,172,76,11,37,
  27,194,50,219,233,26,229,21,87,121,9,104,140,162,188,179,108,90,194,148,
  243,252,36,123,202,13,174,182,192,75,183,44,241,194,88,24,118,210,122,246,
  74,212,143,69,99,31,95,54,249,108,88,163,124,61,187,176,195,122,253,196,
  227,180,74,175,186,64,12,55,110,116,183,161,22,234,160,92,37,123,150,64,
  47,210,78,128,81,93,175,79,20,226,229,39,25,116,196,32,185,9,131,220,
  116,221,38,230,105,71,85,62,218,65,9,47,129,103,179,225,204,110,187,232,
  50,12,214,231,222,86,157,34,2,128,166,246,117,109,61,243,75,145,25,173,
  173,249,202,18,173,108,154,73,253,52,74,214,240,217,29,178,81,166,80,143,
  118,168,43,237,160,207,175,46,119,91,169,221,151,178,148,113,7,108,245,181,
  245,212,165,99,56,171,196,138,134,50,217,209,69,219,139,224,29,104,106,157,
  226,147,59,201,150,54,148,57,125,61,218,49,219,24,213,16,209,86,116,92,
  162,41,252,72,244,254,236,254,85,250,171,108,38,250,118,196,251,171,114,173,
  124,58,133,219,67,190,127,21,245,127,13,38,190,66,209,155,171,222,110,123,
  183,241,230,69,175,118,177,107,181,172,70,237,162,217,180,154,86,163,252,119,
  109,35,254,75,219,115,71,200,17,81,114,213,219,59,253,38,220,94,16,6,
  126,20,160,228,252,170,183,127,254,77,184,31,92,223,119,103,114,140,162,179,
  171,222,193,217,55,33,191,147,24,153,182,65,125,8,240,220,153,18,135,62,
  138,46,174,122,79,47,190,137,202,239,182,15,13,166,253,162,181,242,229,85,
  239,240,242,219,26,72,27,141,39,65,168,164,33,254,16,240,127,36,45,165,
  95,251,238,72,126,23,171,175,254,129,59,119,163,239,234,137,55,114,222,255,
  79,16,94,127,23,242,217,4,159,227,224,187,122,241,133,244,181,112,191,189,
  189,151,147,64,250,238,237,22,146,125,29,168,254,41,194,60,79,170,239,234,
  198,83,127,136,78,164,189,138,40,251,237,170,247,236,244,183,173,240,47,237,
  161,59,114,135,59,47,225,89,189,216,139,81,244,242,170,215,52,176,175,17,
  132,249,194,208,200,80,187,144,183,238,48,232,159,193,84,231,180,123,19,124,
  207,253,20,219,60,18,30,106,217,38,236,127,199,8,55,167,182,103,111,69,
  235,18,132,166,182,209,146,135,65,159,7,227,32,162,130,163,122,99,255,248,
  43,192,175,221,233,182,160,103,118,104,15,109,165,161,247,142,247,30,134,190,
  178,253,200,213,202,169,193,241,185,123,92,187,120,6,201,28,236,180,246,106,
  23,123,230,219,195,245,156,134,99,4,241,48,56,59,207,17,107,146,62,185,
  161,52,76,236,30,239,126,141,137,160,127,105,199,94,176,37,252,5,157,132,
  164,53,210,109,17,94,219,168,255,243,150,34,121,105,223,216,137,225,220,95,
  142,214,157,84,97,118,154,95,151,105,208,127,17,208,210,107,96,156,198,87,
  212,38,134,35,13,250,239,160,215,57,248,243,24,201,10,141,42,253,5,3,
  213,119,216,80,255,251,162,215,120,126,197,172,237,19,67,96,173,65,223,138,
  209,94,208,246,183,4,237,213,214,104,175,93,53,96,106,31,206,123,141,15,
  231,91,227,93,216,14,205,112,65,128,231,189,122,243,108,137,152,160,237,236,
  22,35,94,218,161,171,190,3,239,121,24,35,57,242,190,7,245,116,74,219,
  137,28,123,250,61,100,101,168,197,250,173,136,127,198,232,245,201,119,32,190,
  11,166,242,59,208,126,119,165,175,53,250,91,59,4,70,61,254,30,138,31,
  236,80,217,243,239,64,188,138,130,225,245,36,240,190,167,63,222,42,182,32,
  223,138,118,134,15,127,2,239,245,61,93,249,18,106,231,250,215,46,149,0,
  181,117,158,25,38,187,41,242,134,225,124,74,97,180,250,46,212,231,49,29,
  26,71,168,252,93,216,191,221,185,55,223,133,248,138,54,111,14,98,143,76,
  233,19,216,221,250,6,177,92,4,106,24,204,57,244,248,109,13,230,52,242,
  200,235,12,17,162,222,93,127,178,111,220,107,99,154,52,220,233,136,108,34,
  98,132,11,61,9,118,110,115,43,117,49,220,27,29,98,67,212,105,107,30,
  26,199,13,124,54,143,107,100,160,127,221,61,216,223,105,237,111,83,207,107,
  56,61,18,253,135,83,116,248,118,132,221,48,200,11,109,15,162,218,223,73,
  116,100,47,117,146,15,87,132,84,4,89,146,84,131,56,164,200,254,10,230,
  190,222,218,6,241,13,113,48,96,85,3,211,187,219,160,156,14,135,161,253,
  13,210,61,245,198,174,12,83,67,250,48,138,114,209,24,9,219,107,123,146,
  198,235,43,106,200,43,237,48,247,72,28,7,121,63,241,80,61,207,165,27,
  198,133,202,188,28,130,91,84,243,206,189,179,157,201,170,118,62,132,1,223,
  104,187,26,97,239,184,190,247,117,132,158,156,132,182,111,72,236,54,64,4,
  249,228,22,13,180,199,19,199,118,214,88,195,187,4,228,55,138,220,38,196,
  204,229,111,189,250,126,1,64,224,93,219,28,51,146,168,247,151,116,115,149,
  68,19,154,139,139,53,165,253,189,253,99,64,238,21,212,246,98,98,95,155,
  65,116,112,92,63,88,7,248,143,141,248,197,180,244,128,90,122,80,72,241,
  57,192,174,131,107,13,247,244,184,254,116,29,228,87,80,10,153,241,15,175,
  158,23,1,92,33,84,178,103,65,40,117,45,135,199,245,195,130,166,197,8,
  195,251,175,227,233,44,14,31,130,187,176,125,215,51,241,126,209,123,164,27,
  227,254,111,248,160,124,227,183,66,144,158,141,188,219,213,49,96,209,251,43,
  154,22,159,216,15,65,200,128,205,36,210,161,250,179,2,2,193,245,29,89,
  147,95,139,95,191,247,108,216,89,27,157,253,96,75,79,189,169,205,89,16,
  245,244,113,145,202,244,108,53,185,70,100,254,16,204,111,118,106,209,247,168,
  155,247,10,187,153,206,164,78,7,129,169,136,224,138,21,240,119,207,118,220,
  155,64,69,70,37,154,128,108,22,192,253,71,66,151,17,254,250,198,14,110,
  100,239,77,128,218,220,129,27,170,53,29,123,59,148,232,107,134,140,85,20,
  218,30,192,47,17,74,147,5,56,253,176,236,152,34,184,23,118,56,231,232,
  237,148,122,240,89,218,150,34,216,83,71,122,54,242,141,44,244,41,167,6,
  48,77,77,138,18,246,56,49,216,221,92,197,115,132,182,3,219,231,42,96,
  221,82,145,20,193,94,221,57,190,188,91,66,158,158,127,19,169,11,233,13,
  130,56,204,210,250,198,26,94,6,3,140,215,111,68,79,178,249,211,120,120,
  77,107,188,52,219,246,39,161,183,222,252,73,232,207,116,60,245,21,236,179,
  9,12,24,135,227,80,157,22,89,175,102,11,230,11,15,187,120,48,181,236,
  80,81,90,85,106,221,214,107,251,197,253,228,154,170,168,162,77,96,72,242,
  57,224,156,20,118,76,2,117,25,132,81,255,2,70,74,13,238,214,84,123,
  29,188,103,79,220,136,169,3,232,120,51,220,233,204,213,102,184,9,183,208,
  220,40,152,30,12,22,236,193,44,94,131,125,223,59,67,89,250,9,66,127,
  119,204,180,232,79,93,183,123,236,4,195,152,118,70,90,99,25,157,195,85,
  227,235,243,187,87,78,197,173,38,96,82,13,187,170,203,39,229,252,113,69,
  85,45,58,194,102,15,101,101,231,175,71,71,199,229,191,119,198,181,97,247,
  184,114,95,126,84,110,151,31,217,211,89,135,152,160,239,94,68,95,143,233,
  235,152,190,62,46,63,198,215,127,226,0,15,139,191,134,127,87,83,26,8,
  176,187,83,16,153,170,163,102,163,209,56,153,170,39,101,49,85,130,38,31,
  218,40,124,214,160,210,11,244,190,197,75,145,149,169,218,33,192,42,192,150,
  80,8,179,228,193,10,212,65,195,128,33,225,214,128,84,186,107,29,200,131,
  170,21,5,191,184,183,210,169,52,9,96,194,175,19,142,156,56,164,86,19,
  233,19,98,70,149,219,32,176,151,231,130,170,55,117,163,94,84,11,106,171,
  181,162,70,79,70,162,119,126,113,217,191,56,253,163,187,223,68,134,252,59,
  127,221,109,236,55,245,91,90,31,122,29,216,142,116,186,188,1,175,54,136,
  213,157,249,74,199,17,33,124,105,95,119,27,157,210,40,246,245,138,12,45,
  159,158,5,190,95,25,122,170,54,85,227,234,189,102,124,216,253,169,162,79,
  188,87,59,238,168,242,131,126,101,233,211,239,221,40,140,101,39,148,180,173,
  187,179,24,90,60,237,255,134,239,146,34,20,81,126,130,218,58,67,139,54,
  14,158,153,43,165,80,65,39,197,103,150,58,139,210,146,143,145,235,121,189,
  207,149,132,188,34,242,116,38,174,74,119,226,136,113,215,143,61,175,22,232,
  191,157,146,232,253,105,161,177,231,136,101,42,149,191,198,225,172,70,55,89,
  213,232,224,216,223,213,238,241,61,56,70,225,15,221,46,152,30,119,241,181,
  3,212,84,71,135,144,66,36,141,154,86,202,193,44,162,149,105,162,21,140,
  45,222,159,194,40,202,226,211,61,206,25,93,99,83,9,198,85,48,44,132,
  230,47,232,250,114,46,222,242,150,252,74,74,155,43,200,33,1,7,218,41,
  242,85,101,80,203,103,48,136,193,148,143,188,65,201,135,252,84,174,106,156,
  192,215,75,227,221,138,105,147,178,120,185,24,237,74,33,89,76,229,170,126,
  209,53,0,157,5,240,147,55,129,207,123,122,116,37,154,251,155,110,14,205,
  194,152,156,86,232,70,16,122,57,237,146,112,93,232,101,212,61,142,254,106,
  253,221,237,118,111,170,29,83,117,119,122,50,69,89,59,97,0,164,178,221,
  24,221,250,149,168,74,188,254,16,85,181,130,60,206,237,245,225,109,100,62,
  237,109,184,75,183,238,63,6,183,26,86,124,252,233,30,134,162,130,62,154,
  58,213,197,209,32,60,94,199,78,64,70,180,53,163,186,248,233,222,124,61,
  41,139,47,2,86,162,140,34,88,2,64,224,179,63,5,8,202,9,138,78,
  22,244,225,35,165,90,232,205,30,10,47,104,125,141,94,186,42,236,199,120,
  17,39,39,5,62,230,26,198,55,96,184,159,101,37,172,222,27,94,137,139,
  48,99,199,254,187,242,238,252,234,253,235,94,91,92,158,94,93,89,63,127,
  161,63,213,157,241,180,182,34,2,31,109,72,54,2,62,174,150,68,81,21,
  191,156,190,122,141,42,232,79,65,21,3,68,249,217,58,50,53,124,56,125,
  247,102,29,129,46,237,40,31,211,187,20,39,215,188,80,210,233,216,138,83,
  189,103,197,161,245,128,106,110,248,150,111,202,79,28,139,206,143,0,92,107,
  151,190,114,35,15,230,88,124,134,153,207,208,158,100,31,44,53,243,92,12,
  54,81,174,254,213,132,250,36,23,117,148,141,166,234,171,47,54,87,246,229,
  75,89,175,250,69,162,146,61,199,246,166,119,89,77,234,232,125,94,169,0,
  218,100,234,136,62,247,121,237,85,46,68,37,45,116,84,212,167,35,64,125,
  194,89,84,63,162,26,61,4,230,32,61,135,71,52,245,242,253,24,85,203,
  69,70,29,190,236,93,188,238,206,249,106,20,222,24,15,139,112,242,113,173,
  123,211,215,201,169,19,82,64,210,151,185,165,148,235,24,141,156,91,33,158,
  22,124,4,106,9,224,206,204,107,253,56,65,152,91,93,104,33,124,4,181,
  246,199,117,77,112,92,181,13,69,62,177,129,214,199,33,74,233,161,175,199,
  6,157,217,190,163,45,60,63,221,103,188,18,184,163,242,126,226,35,23,130,
  134,139,103,182,82,168,192,111,167,20,168,176,175,11,89,150,213,197,71,35,
  58,186,40,36,43,57,51,190,29,203,143,102,150,62,95,205,44,232,29,78,
  154,55,253,50,57,126,78,60,82,187,159,152,242,228,88,253,137,121,214,221,
  218,167,3,229,39,31,81,83,129,218,147,123,73,142,171,83,231,175,160,45,
  146,1,15,42,91,252,175,253,120,149,10,245,184,57,2,159,142,174,54,25,
  35,63,72,111,1,40,87,141,68,244,13,40,57,153,244,254,96,101,136,110,
  251,206,96,202,234,160,27,156,20,145,234,34,47,125,244,104,165,224,135,110,
  82,128,150,23,180,187,162,15,176,193,54,32,170,84,25,26,6,127,81,77,
  90,14,163,89,93,146,164,139,4,93,233,156,148,203,104,235,186,182,85,120,
  255,150,134,169,102,204,152,120,242,49,163,117,51,181,28,91,166,7,233,181,
  222,26,69,172,44,149,86,37,74,72,251,199,120,88,232,239,92,108,154,128,
  129,111,222,81,65,95,23,164,106,198,59,24,178,50,213,157,204,122,73,55,
  200,156,228,244,46,45,134,234,173,187,151,138,246,30,43,160,125,242,37,234,
  103,51,18,190,164,122,196,239,120,131,197,130,239,170,81,213,140,54,181,139,
  140,48,157,206,161,237,21,121,255,103,194,158,25,56,231,35,94,181,192,239,
  42,168,105,56,149,142,110,164,62,10,157,120,122,52,8,175,233,8,15,139,
  153,205,90,82,160,240,49,148,218,160,1,81,95,125,84,205,70,105,250,254,
  163,242,147,74,224,159,144,250,182,185,233,213,60,124,214,146,2,174,130,234,
  121,71,204,73,153,255,32,146,157,89,233,214,146,147,114,250,21,181,113,19,
  96,162,203,24,7,153,83,105,229,101,67,199,126,247,6,225,254,205,113,227,
  164,252,132,156,118,245,201,77,26,242,182,146,193,162,111,237,89,239,88,190,
  224,167,15,73,184,218,0,163,237,99,191,146,125,55,155,77,209,79,143,102,
  94,172,166,126,39,237,174,244,101,95,134,97,134,222,66,160,76,176,5,120,
  242,17,162,92,173,235,231,134,213,56,60,216,171,230,80,212,142,99,223,85,
  245,181,25,89,211,165,177,168,107,19,227,149,90,47,253,42,148,116,233,156,
  129,88,29,187,172,132,52,30,8,70,172,183,204,32,235,6,18,215,27,232,
  231,136,84,23,43,67,189,157,131,157,154,155,147,180,5,60,249,168,159,233,
  162,49,189,133,84,69,250,172,155,47,165,147,94,174,65,135,24,89,229,105,
  227,226,64,210,134,80,179,65,219,15,16,100,137,202,132,246,227,111,22,76,
  237,27,187,167,106,12,116,187,156,97,15,163,105,96,43,73,183,193,209,136,
  90,154,89,190,188,41,171,57,149,157,203,211,55,175,206,190,60,127,247,246,
  195,155,183,239,123,95,120,183,144,19,140,119,92,139,54,4,129,71,182,42,
  198,145,85,215,61,187,142,186,140,33,201,193,46,165,251,113,83,188,186,9,
  35,53,156,188,85,235,71,146,4,125,75,108,202,23,17,207,82,25,198,51,
  138,91,178,30,113,9,122,220,60,121,92,232,4,180,107,13,37,215,207,113,
  226,157,222,116,213,216,93,63,200,73,231,96,82,155,78,170,98,196,73,151,
  91,229,124,214,40,148,212,183,96,97,34,237,89,159,30,17,38,180,246,178,
  121,235,66,92,63,231,144,33,228,163,205,124,117,101,6,199,148,111,68,211,
  39,160,165,214,238,4,9,57,114,33,66,34,144,108,205,226,72,28,54,159,
  181,196,137,40,116,99,163,208,30,83,6,152,241,98,162,45,30,63,174,46,
  227,192,91,10,4,111,191,124,185,55,9,149,57,201,152,149,196,15,243,91,
  203,156,195,61,41,72,117,146,11,17,41,188,74,118,60,39,212,104,24,50,
  58,154,216,183,35,218,217,27,21,215,66,129,4,159,78,75,161,178,238,131,
  234,65,53,52,220,250,102,27,115,73,20,198,165,198,57,223,90,230,248,25,
  68,7,159,122,107,81,165,253,225,194,108,179,206,197,67,79,8,156,221,32,
  134,100,16,174,90,43,22,37,7,133,134,181,52,42,204,98,173,153,159,194,
  48,214,15,146,93,216,204,84,134,104,89,212,225,173,214,171,165,100,47,77,
  215,116,15,209,105,209,92,172,126,107,5,3,213,167,129,67,35,218,212,145,
  20,85,185,121,228,219,175,125,12,131,146,54,127,156,35,80,148,172,1,217,
  245,67,84,129,231,48,99,105,217,49,30,166,246,173,126,56,41,82,178,171,
  222,233,235,243,204,120,90,24,35,182,26,88,233,200,131,207,244,250,204,73,
  205,28,126,225,164,7,70,118,37,212,66,2,101,178,30,115,164,53,103,233,
  192,214,156,74,251,52,187,211,31,34,129,167,105,147,47,95,214,203,233,82,
  36,56,106,212,78,194,41,212,188,244,14,146,89,172,38,171,106,87,164,98,
  43,84,86,180,74,116,249,120,73,6,12,143,137,147,76,203,6,178,239,203,
  113,161,178,85,232,136,54,109,167,207,106,110,205,28,24,164,235,174,150,123,
  245,57,211,35,0,212,213,231,237,254,39,229,154,222,247,79,193,9,247,71,
  86,49,141,68,245,33,223,181,65,158,31,165,232,0,22,64,170,41,172,150,
  230,61,52,34,96,149,193,223,101,211,24,98,18,69,96,24,33,27,103,43,
  47,123,189,75,177,212,246,244,229,34,29,40,140,201,180,131,107,70,225,193,
  54,14,2,71,228,73,7,215,57,170,235,105,138,30,99,108,79,85,60,28,
  66,138,105,182,248,120,73,138,166,169,32,156,147,194,44,71,27,161,127,98,
  25,47,81,219,196,148,190,126,204,95,178,68,5,125,87,7,64,137,92,245,
  54,229,149,20,61,141,250,49,12,181,240,56,238,15,174,77,25,95,3,34,
  29,42,35,233,112,66,165,95,177,176,232,81,45,211,139,25,221,114,159,129,
  225,103,3,132,215,48,123,158,212,111,248,107,95,159,111,231,119,228,57,132,
  186,118,103,166,118,114,36,244,152,102,25,250,16,118,158,253,4,208,156,43,
  34,227,97,46,101,131,203,202,191,44,246,115,149,196,87,105,120,243,244,16,
  74,213,68,169,180,37,93,31,136,205,144,226,130,141,14,178,93,78,29,9,
  197,74,201,180,9,157,11,205,217,15,51,106,116,180,95,52,196,179,0,52,
  126,79,232,136,171,62,1,133,81,165,211,131,133,88,129,163,125,221,139,157,
  130,66,136,255,191,86,138,249,112,90,95,45,148,105,108,17,73,80,106,11,
  154,149,202,188,24,62,41,27,115,147,58,6,116,237,50,191,53,192,116,38,
  142,34,239,181,166,209,139,37,223,26,108,161,30,168,41,162,14,51,169,72,
  65,125,246,116,224,234,48,62,15,191,102,21,11,170,102,3,184,62,130,85,
  60,168,211,237,36,69,198,141,110,99,212,198,173,10,233,136,74,106,9,171,
  15,138,35,184,102,205,213,118,217,148,145,85,22,59,120,78,150,28,144,98,
  254,15,199,97,90,220,55,122,86,40,23,164,255,158,57,173,109,28,157,153,
  19,72,208,102,243,105,127,242,121,5,149,50,234,235,151,159,151,180,9,106,
  128,100,98,81,199,103,18,134,228,106,56,202,61,245,231,182,31,153,217,155,
  13,83,33,4,33,157,53,78,24,179,152,157,26,29,91,71,64,50,226,251,
  25,221,104,117,178,164,93,48,37,75,24,111,222,246,196,105,175,119,122,246,
  242,252,133,185,200,131,175,207,82,236,203,7,146,188,41,31,125,240,249,190,
  65,207,179,196,99,86,242,199,191,36,167,168,196,173,104,253,55,9,64,200,
  219,33,167,92,84,203,235,243,23,103,201,149,50,65,200,135,74,30,47,197,
  226,250,95,190,60,173,62,121,204,23,136,249,246,13,76,30,5,158,214,218,
  4,141,62,32,189,20,182,126,78,243,240,197,202,139,68,81,211,169,22,190,
  128,38,151,11,228,228,240,211,125,110,78,97,145,4,157,106,102,173,164,62,
  133,105,47,13,57,147,208,242,116,134,126,234,79,22,98,146,15,244,204,205,
  181,121,115,12,20,151,138,79,244,244,8,127,95,8,241,215,242,169,63,145,
  183,139,191,63,154,248,73,135,249,3,68,249,131,88,233,122,233,160,91,190,
  210,129,53,148,253,25,152,80,25,147,106,128,95,126,94,3,230,85,134,201,
  231,227,220,18,95,90,76,139,121,64,106,243,108,118,146,118,103,172,177,62,
  6,183,90,39,93,200,217,231,243,80,79,42,230,105,76,201,144,226,136,96,
  120,55,132,87,67,1,132,150,123,187,16,83,149,9,108,244,113,185,108,215,
  209,114,205,128,195,135,106,135,0,248,68,92,49,0,103,22,26,74,31,124,
  219,0,198,7,229,18,130,124,184,45,11,56,32,37,240,56,244,78,230,228,
  50,69,15,205,199,25,176,236,52,220,3,83,109,233,50,83,91,12,237,89,
  196,66,230,243,71,124,38,149,154,162,50,71,138,156,64,234,209,153,220,181,
  149,78,203,113,31,211,9,187,149,168,133,174,14,164,219,119,180,154,78,135,
  113,159,102,68,56,196,208,241,147,88,155,159,51,247,222,181,51,111,244,121,
  190,116,162,142,15,232,229,194,206,1,47,75,209,177,85,216,137,181,212,144,
  77,206,217,185,190,169,53,140,103,17,183,33,133,110,235,235,185,160,22,174,
  175,167,187,137,9,125,119,134,237,33,191,30,87,179,161,188,161,21,218,211,
  147,245,245,170,87,239,78,47,234,202,30,73,125,150,141,143,22,46,81,139,
  166,57,19,56,190,69,149,174,240,76,249,226,187,116,249,215,57,218,250,54,
  63,250,89,30,90,41,231,53,29,212,49,73,110,172,164,123,9,7,146,207,
  34,47,227,205,100,152,250,121,85,126,59,160,72,14,169,56,7,148,80,151,
  161,57,193,88,69,138,54,171,84,254,186,174,221,208,218,240,199,228,120,228,
  79,247,215,203,101,179,254,206,184,70,107,83,139,228,196,36,94,223,44,150,
  71,37,63,86,173,79,129,235,87,140,227,76,43,161,85,34,145,76,99,103,
  112,211,21,34,149,169,36,135,196,19,225,57,140,236,132,120,134,242,58,61,
  51,61,158,67,78,166,204,55,80,203,78,163,231,241,178,211,233,25,170,169,
  113,228,101,56,132,164,75,107,25,171,187,55,193,188,27,89,230,130,56,51,
  107,156,156,197,172,90,201,156,7,173,50,155,35,150,43,133,73,38,144,43,
  228,51,148,43,101,122,68,164,69,134,182,241,0,124,8,113,197,84,106,0,
  178,45,145,69,235,242,139,228,26,59,203,178,146,245,173,41,58,18,22,164,
  150,177,207,120,252,25,202,195,191,146,227,208,146,151,6,180,111,43,205,90,
  100,65,55,245,109,61,83,85,173,86,23,255,34,155,10,30,220,81,69,19,
  169,234,25,199,124,20,157,174,24,71,150,94,129,169,106,148,31,150,187,52,
  170,247,64,244,121,137,76,207,234,103,215,197,200,212,242,77,226,249,151,203,
  117,177,78,126,237,63,93,233,236,172,172,252,87,136,176,208,148,90,133,164,
  90,95,190,160,65,26,98,183,16,98,55,3,49,162,229,43,62,85,139,30,
  249,225,135,252,178,156,33,229,176,163,201,49,150,172,182,118,120,45,131,46,
  181,77,119,45,228,204,33,189,87,38,200,72,168,0,194,204,177,153,250,205,
  117,116,57,136,229,92,40,223,233,144,110,149,89,6,180,72,18,110,191,124,
  73,94,116,204,254,153,52,162,213,175,117,41,113,97,174,237,201,107,88,138,
  77,82,166,27,123,200,190,220,22,21,167,173,55,212,249,21,175,179,228,95,
  233,56,203,52,76,223,52,179,2,145,75,50,86,18,137,118,171,97,80,249,
  202,174,4,179,66,19,151,143,30,209,103,58,217,135,62,164,11,231,202,9,
  41,190,38,53,65,48,27,179,146,37,66,246,244,230,134,204,53,16,179,182,
  71,191,230,147,110,56,226,45,65,139,210,162,84,178,121,121,43,221,90,64,
  241,72,133,183,132,208,224,52,155,66,58,188,51,137,113,74,112,2,119,247,
  96,137,54,250,176,254,208,115,216,181,105,153,95,103,171,149,242,142,61,115,
  119,20,255,162,70,185,118,63,180,209,237,20,66,213,233,242,39,89,94,84,
  59,139,33,205,230,87,124,25,157,135,97,245,158,22,156,230,247,138,174,146,
  2,28,162,171,5,111,222,161,225,23,90,193,117,53,247,158,38,18,202,53,
  154,118,105,135,150,38,66,243,189,204,208,167,132,161,79,134,161,208,250,132,
  32,182,146,82,228,25,134,117,154,92,108,168,154,205,21,159,32,175,236,38,
  172,116,235,85,185,92,163,249,57,26,166,166,82,73,251,48,196,18,250,201,
  147,142,230,126,89,116,220,109,49,16,149,202,71,143,164,197,132,187,221,46,
  55,183,186,172,28,161,1,237,59,212,167,166,67,190,7,134,79,72,243,111,
  251,89,226,85,164,239,249,51,225,9,101,22,250,250,84,114,80,51,186,87,
  229,237,204,220,129,241,254,234,121,18,60,76,97,91,232,222,45,100,48,205,
  230,126,171,209,104,39,119,32,131,141,48,138,103,98,64,7,16,66,115,229,
  59,12,160,164,115,105,227,90,38,234,98,178,52,104,137,108,29,85,194,185,
  120,238,181,244,244,117,153,251,72,39,205,173,9,20,199,240,53,52,188,194,
  180,107,237,226,21,197,23,202,226,208,86,112,68,177,46,8,238,215,53,73,
  188,224,134,155,13,71,136,70,120,10,174,252,68,90,164,0,136,206,105,87,
  73,70,221,30,38,161,187,121,19,13,254,229,13,186,161,5,246,204,231,27,
  26,245,173,41,212,19,60,152,197,175,87,111,223,88,226,140,140,88,122,175,
  153,142,205,114,116,215,234,55,55,20,196,51,135,150,229,72,43,244,18,3,
  31,166,135,26,162,30,154,27,209,220,78,165,82,148,20,165,223,218,146,119,
  152,9,40,231,98,132,224,208,243,238,238,151,91,5,59,139,162,65,172,162,
  10,93,183,83,163,223,249,33,197,51,251,224,102,81,247,158,46,231,9,156,
  118,249,242,237,85,175,92,211,63,162,163,218,247,229,63,234,239,40,161,85,
  72,188,235,116,227,10,178,158,55,209,140,127,12,232,34,112,202,139,133,118,
  136,92,35,42,178,232,75,151,62,210,88,35,111,5,152,62,0,171,102,235,
  97,102,44,150,74,48,87,35,246,121,250,254,208,174,110,128,236,30,223,75,
  107,70,215,8,248,145,185,117,179,178,92,224,97,138,180,29,239,253,187,215,
  87,146,34,149,75,27,1,176,170,220,195,167,181,243,206,217,236,146,171,145,
  79,110,175,248,230,90,244,185,93,180,165,174,6,33,147,215,109,175,120,223,
  228,53,249,219,246,138,223,205,188,27,5,237,117,151,123,82,110,66,144,141,
  50,213,13,159,218,206,123,219,26,187,214,246,170,143,173,177,71,109,175,187,
  214,180,54,237,81,219,5,190,53,75,144,125,70,123,213,121,212,180,163,104,
  175,185,140,133,73,4,249,30,215,76,28,199,134,159,122,140,174,112,93,217,
  104,166,127,67,0,225,90,217,184,6,221,85,137,253,101,77,52,254,192,252,
  218,76,89,43,165,113,107,166,206,101,32,246,201,162,153,243,181,148,134,120,
  114,126,186,255,68,225,11,5,192,100,183,168,177,200,225,232,230,139,180,132,
  39,69,210,105,200,79,198,63,254,144,89,59,97,156,222,31,24,111,201,91,
  152,17,231,249,180,156,93,158,106,111,92,75,254,100,173,172,145,25,91,79,
  124,87,87,183,244,146,67,73,125,68,81,107,11,82,196,80,143,66,99,38,
  210,60,106,81,216,53,134,74,103,205,4,32,133,228,17,8,71,87,220,41,
  252,146,55,11,51,231,27,34,116,211,218,206,210,211,45,116,132,0,126,242,
  105,68,224,15,61,119,120,205,27,87,137,184,238,116,202,69,118,102,9,80,
  167,148,205,49,30,192,208,115,210,6,62,73,63,30,128,95,238,102,49,56,
  38,59,121,136,41,100,184,12,164,17,116,234,242,0,60,210,109,186,216,133,
  224,87,68,29,177,60,131,141,146,78,42,153,34,108,41,178,94,65,53,25,
  12,201,109,81,171,227,193,44,116,174,13,138,224,154,84,28,193,234,138,126,
  215,121,209,142,20,28,127,51,106,205,227,71,142,11,215,62,147,137,106,33,
  125,184,86,190,91,139,103,33,178,203,161,102,53,244,27,198,198,138,254,23,
  53,240,27,134,128,214,185,183,163,209,74,87,233,30,184,215,161,121,123,37,
  112,95,152,30,126,169,67,245,2,60,10,226,219,171,209,124,22,173,213,120,
  0,145,174,80,76,128,245,101,128,133,176,188,220,210,134,113,174,241,106,9,
  211,211,119,54,102,8,174,104,214,60,209,44,202,23,110,47,214,173,239,60,
  8,175,183,52,191,243,219,175,105,95,66,225,171,182,88,175,40,146,50,153,
  92,133,85,46,185,196,90,175,252,167,239,114,171,254,255,91,253,89,103,241,
  155,181,231,131,206,187,178,61,116,191,154,142,173,60,27,31,111,69,193,123,
  250,237,223,51,91,73,24,192,185,233,87,221,204,246,10,14,37,57,90,39,
  248,234,226,60,65,131,202,113,18,169,68,162,62,189,34,211,104,244,135,77,
  226,215,244,135,43,209,11,132,73,21,172,82,220,202,82,222,196,228,221,184,
  249,153,143,45,20,105,179,33,187,39,107,179,146,81,47,182,50,110,69,182,
  141,111,186,21,93,97,76,156,89,208,226,239,63,235,172,127,39,73,227,191,
  182,184,149,85,191,255,143,214,203,216,8,189,251,117,189,127,50,47,243,161,
  151,238,130,165,63,82,12,212,41,130,95,198,3,201,17,157,142,241,215,232,
  225,228,215,181,42,84,84,99,153,117,232,38,45,125,111,214,209,142,254,81,
  208,29,254,121,250,255,7,135,214,36,250,174,126,0,0
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
// Snap a requested temperature to what the face can actually show. Applied BEFORE the
// calibration offset, deliberately: the offset exists to nudge a reading that lands just the
// wrong side of a boundary, and quantising after it would discard every nudge smaller than a
// whole degree, which is all of them.
float quantiseC(float c) {
#if TEMP_WHOLE_DEGREES == 1
  return (float)(long)c;                      // toward zero: 21.9 -> 21, -3.7 -> -3
#elif TEMP_WHOLE_DEGREES == 2
  return (float)(long)(c >= 0.0f ? c + 0.5f : c - 0.5f);
#else
  return c;
#endif
}

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
// Quantised here rather than inside rawForC so the march and a manual hold can still ask for
// an exact value when a test wants one.
void driveC(float cIn) {
  const float c = quantiseC(cIn);
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
// What actually reaches the face, after whole-degree snapping. Reported next to the raw
// reading so the UI and the display cannot disagree.
float    quantisedTargetC() { return quantiseC(impl::targetC_); }

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

// Set when a fetch saw a 404 and a suffix probe is worth running once the session is closed.
bool wantProbe_ = false;
// One probe per station per suffix. Without this, a host answering HEAD 200 while answering
// GET 404 would flip the cached suffix back and forth at network round-trip rate, with a full
// settings write per cycle.
bool probeExhausted_ = false;

// Build the observation URL. A station containing '-' is already a complete stem (CYYZ-MAN)
// and gets only the extension; a bare code gets the suffix it is given.
void buildUrl(char* out, size_t n, const char* suffix) {
  if (settings::wxStationIsStem(g_settings.wxStation))
    std::snprintf(out, n, "%s://%s%s%s-swob.xml", WX_USE_HTTPS ? "https" : "http",
                  WX_HOST, WX_PATH_PREFIX, g_settings.wxStation);
  else
    std::snprintf(out, n, "%s://%s%s%s%s", WX_USE_HTTPS ? "https" : "http",
                  WX_HOST, WX_PATH_PREFIX, g_settings.wxStation, suffix);
}

// HEAD one URL and return the status code, or a negative HTTPClient error. Only used to find
// which suffix a bare code lives under, so the body is never wanted. Verified against the live
// host: it answers HEAD with the same 200 or 404 it answers a GET with.
int headCode(const char* url) {
#if WX_USE_HTTPS
  WiFiClientSecure client;
  client.setInsecure();
#else
  WiFiClient client;
#endif
  HTTPClient http;
  http.setConnectTimeout(WX_PROBE_TIMEOUT_MS);
  http.setTimeout(WX_PROBE_TIMEOUT_MS);
  http.setReuse(false);
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  if (!http.begin(client, url)) return -1;
  const int code = http.sendRequest("HEAD");
  http.end();
  return code;
}

// A 404 means either the wrong suffix or a wrong station code. Try the other suffix; adopt it
// only on a 200, so a station that is simply down, or a code that is wrong, cannot corrupt the
// cache. Returns true when the cache changed and the caller should try again.
bool reprobeSuffix() {
  if (settings::wxStationIsStem(g_settings.wxStation)) return false;
  const bool onAuto = (std::strcmp(g_settings.wxSuffix, WX_PATH_SUFFIX) == 0);
  const char* other = onAuto ? WX_PATH_SUFFIX_ALT : WX_PATH_SUFFIX;
  char url[192];
  buildUrl(url, sizeof url, other);
  if (headCode(url) != HTTP_CODE_OK) return false;
  std::strncpy(g_settings.wxSuffix, other, sizeof g_settings.wxSuffix - 1);
  g_settings.wxSuffix[sizeof g_settings.wxSuffix - 1] = '\0';
  settings::save();
  Serial.printf("[wx] %s is %s\n", g_settings.wxStation, other);
  return true;
}

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
  buildUrl(url, sizeof url, g_settings.wxSuffix);

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
        // Only NOTE that a probe is wanted. Running it here would build a SECOND TLS session
        // while this one is still open, on a heap only ever checked to hold one, and would
        // commit to NVS mid-session. It runs after this scope closes.
        wantProbe_ = (lastCode_ == 404);
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

  // The session is closed now, so a probe can have the heap to itself.
  if (wantProbe_ && !probeExhausted_) {
    wantProbe_ = false;
    probeExhausted_ = true;
    if (!WX_USE_HTTPS ||
        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) >= WX_MIN_LARGEST_BLOCK) {
      if (reprobeSuffix()) {
        char m[64];
        std::snprintf(m, sizeof m, "HTTP 404, retrying as %s", g_settings.wxSuffix);
        setError(m);
        pending_ = true;                    // retry now, with the corrected suffix
      }
    }
  }
  wantProbe_ = false;
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
void allowProbe() { impl::probeExhausted_ = false; }

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
      .real("temp_target_c", tempout::targetC(), 1)
      .real("temp_sent_c", tempout::quantisedTargetC(), 1)
      .num("temp_whole_degrees", (long)TEMP_WHOLE_DEGREES)
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
  j.str("server2", g_settings.ntpServer2).str("server3", g_settings.ntpServer3)
      .boolean("failover", g_settings.ntpFailover)
      .num("active_slot", (long)timekeeping::activeSlot())
      .str("active_role", timekeeping::activeSlot() == 0 ? "primary" : "fallback");
  // drift_ppm is the long baseline; drift_recent_ppm is the shorter window and is the one that
  // moves when the board warms up. drift_ppm_err is the estimator's own noise floor, so a
  // figure quoted from a short baseline is visibly worthless rather than merely wrong.
  j.boolean("drift_measured", timekeeping::driftSpanS() > 0)
      .real("drift_ppm_err", timekeeping::driftUncertaintyPpm(), 2)
      .real("drift_recent_ppm", timekeeping::driftRecentPpm(), 2)
      .unum("drift_recent_span_s", timekeeping::driftRecentSpanS());
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
      // Why the clock is in the mode it is in. On "auto" the format is learned from the MCU
      // writing the hours register, which with the DS1302 removed happens only if someone sets
      // the time with the clock's buttons - so on a fresh build there is usually nothing to
      // learn from and it sits on 24 hour. Stating the reason beats leaving it to be worked
      // out from three separate fields.
      .str("hour_source", g_settings.hourFormat != HOURFMT_AUTO ? "forced by setting"
                        : snap.learnedHour >= 0 ? "learned from the MCU"
                        : "default: the MCU has never written the hours register")
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
      sendError(400, "station: 3-14 upper-case letters, digits and '-', e.g. CWWB or CYYZ-MAN");
      return;
    }
    std::strncpy(g_settings.wxStation, s.c_str(), sizeof g_settings.wxStation - 1);
    g_settings.wxStation[sizeof g_settings.wxStation - 1] = '\0';
    std::strncpy(g_settings.wxSuffix, WX_PATH_SUFFIX, sizeof g_settings.wxSuffix - 1);
    g_settings.wxSuffix[sizeof g_settings.wxSuffix - 1] = '\0';
    wx::allowProbe();
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
  int spoofVal = -1, bootVal = -1, ntpfoVal = -1;
  String ntp2, ntp3;
  const bool haveNtp2 = server.hasArg("ntp2");
  const bool haveNtp3 = server.hasArg("ntp3");
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
  // An empty fallback is legal and clears that slot, so only a non-empty value is validated.
  if (err.isEmpty() && haveNtp2) {
    ntp2 = server.arg("ntp2"); ntp2.trim();
    if (ntp2.length() && !settings::validNtpServer(ntp2.c_str()))
      err = "NTP fallback 1: hostname or IPv4 address, or empty to disable";
  }
  if (err.isEmpty() && haveNtp3) {
    ntp3 = server.arg("ntp3"); ntp3.trim();
    if (ntp3.length() && !settings::validNtpServer(ntp3.c_str()))
      err = "NTP fallback 2: hostname or IPv4 address, or empty to disable";
  }
  if (err.isEmpty() && server.hasArg("ntpfo")) {
    const String v = server.arg("ntpfo");
    if (v == "1") ntpfoVal = 1; else if (v == "0") ntpfoVal = 0; else err = "ntpfo: 0 or 1";
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

  const bool ntpChanged = (haveNtp && std::strcmp(ntp.c_str(), g_settings.ntpServer) != 0) ||
                          (haveNtp2 && std::strcmp(ntp2.c_str(), g_settings.ntpServer2) != 0) ||
                          (haveNtp3 && std::strcmp(ntp3.c_str(), g_settings.ntpServer3) != 0);
  const bool tzChanged = (haveTz && std::strcmp(tz.c_str(), g_settings.tz) != 0) || dst != g_settings.dstMode;
  const bool ivalChanged = ival != g_settings.syncIntervalS;

  if (haveNtp) strlcpy(g_settings.ntpServer, ntp.c_str(), sizeof g_settings.ntpServer);
  if (haveNtp2) strlcpy(g_settings.ntpServer2, ntp2.c_str(), sizeof g_settings.ntpServer2);
  if (haveNtp3) strlcpy(g_settings.ntpServer3, ntp3.c_str(), sizeof g_settings.ntpServer3);
  // Turning failover off while parked on a fallback must not strand the device there.
  if (ntpfoVal >= 0) {
    const bool wasOn = g_settings.ntpFailover;
    g_settings.ntpFailover = (ntpfoVal == 1);
    if (wasOn && !g_settings.ntpFailover && timekeeping::activeSlot() != 0)
      timekeeping::selectSlot(0);
  }
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
  if (ivalChanged) timekeeping::setSyncInterval(g_settings.syncIntervalS);
  if (ntpChanged)  timekeeping::setServer(g_settings.ntpServer);

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

bool     g_mdnsStarted = false;

// If SNTP goes quiet for three intervals (or five minutes, whichever is longer) while the
// link is up, kick it. Costs one extra request at most once a minute.
// Strict primary with failover, replacing both the old kick-when-quiet watchdog and lwIP's
// round-robin. Exactly one server is installed at a time, so this is the whole selection
// policy. Silence is judged on the global last-sync age, which is sound here precisely because
// only one server is ever installed: any sync in the window came from the active one.
void ntpPolicy() {
  if (!WiFi.isConnected()) return;                 // a dead link is not the server's fault

  const uint32_t now = millis();
  uint32_t windowS = timekeeping::syncIntervalS() * NTP_FAILOVER_AFTER_MULT;
  if (windowS < NTP_FAILOVER_MIN_S) windowS = NTP_FAILOVER_MIN_S;

  const uint32_t sinceSwitch = now - timekeeping::slotSinceMs();
  const uint8_t  slot = timekeeping::activeSlot();

  // Has the CURRENT server answered at all? selectSlot restarts SNTP, which sends a request
  // immediately, so a reachable server answers within seconds. Counting syncs since the switch
  // is unambiguous where the global age is not: the age may still be small because the
  // PREVIOUS server answered a moment before the switch.
  const bool answered = timekeeping::syncCount() > timekeeping::slotSyncCount();

  if (!answered) {
    if (sinceSwitch < NTP_PROBE_S * 1000UL) return;            // still inside its grace period
    const uint8_t next = timekeeping::nextUsableSlot(slot);
    Serial.printf("[ntp] %s did not answer in %lus, %s\n", timekeeping::server(),
                  (unsigned long)NTP_PROBE_S,
                  next == slot ? "restarting the request" : "trying the next server");
    timekeeping::selectSlot(next);
    return;
  }

  const int64_t age = timekeeping::lastSyncAgeUs();
  const bool healthy = age >= 0 && age <= (int64_t)windowS * 1000000LL;

  if (healthy) {
    // On a fallback and the primary has had long enough to recover: give it the slot back. If
    // it is still down this costs one poll and the next pass demotes again.
    if (slot != 0 && g_settings.ntpFailover &&
        sinceSwitch >= NTP_RETURN_PRIMARY_S * 1000UL) {
      Serial.println("[ntp] returning to the primary server");
      timekeeping::selectSlot(0);
    }
    return;
  }

  const uint8_t next = timekeeping::nextUsableSlot(slot);
  Serial.printf("[ntp] %s silent for %lus, %s\n", timekeeping::server(),
                (unsigned long)windowS,
                next == slot ? "restarting the request" : "failing over");
  timekeeping::selectSlot(next);
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
    // Re-anchor on the primary: a reconnect is a new network situation, and whatever drove
    // the device onto a fallback may well have been the link itself.
    timekeeping::selectSlot(0);             // immediate NTP request on every (re)connect
    if (!g_mdnsStarted && MDNS.begin(DEVICE_HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      g_mdnsStarted = true;
    }
    Serial.printf("Wi-Fi up: http://%s/  (http://%s.local/)\n",
                  WiFi.localIP().toString().c_str(), DEVICE_HOSTNAME);
  }

  ntpPolicy();
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
