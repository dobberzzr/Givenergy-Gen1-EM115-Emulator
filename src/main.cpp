#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>
#include <math.h>
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "secrets.h"  // WIFI_SSID / WIFI_PASSWORD - see include/secrets.h.example
// 14th October 2026. DATOUBOSS CAN receiver added - decodes the real
// Pylontech-LV CAN protocol (confirmed via candump capture the same day,
// see Giv Battery Emulator/giv-docs/10-pylontech-can-reference.md) and
// drives HR20-HR25 / Block 2 SOC+capacity / Block 3 cell min-max from
// live battery data, with a fail-safe if the CAN link goes quiet after
// being established.
// 19th September 2026. First working battery emulator for Givenergy Gen1.
// 12th October 2026. V1.2R: ported the V1.3 robustness work (task watchdog,
// WiFi recovery, restart-reason diagnostics, /health JSON) onto this known-
// good source. The FC04 "candidate offset" mapping lab from the separate
// V1.3 file was NOT ported - this keeps the confirmed capacity/status-field
// editor below, which V1.3 had replaced with unrelated raw-byte editing.
// ============================================================
// GivEnergy Gen1 LV BMS diagnostic emulator
// LilyGo T-CAN485
//
// USB console : Serial  @ 115200
// BMS RS485   : Serial1 @ 9600 8N1
//
// KNOWN HARDWARE SETUP - DO NOT CHANGE:
//   RX           GPIO21
//   TX           GPIO22
//   GPIO16       HIGH
//   GPIO17       HIGH
//   GPIO19       HIGH
//
// Web diagnostics:
//   http://192.168.1.xx/
//
//
// ============================================================


// ============================================================
// WIFI / OTA / WEB
// ============================================================

// WIFI_SSID and WIFI_PASSWORD now come from secrets.h (gitignored, not
// committed). Copy include/secrets.h.example to include/secrets.h and fill
// in your real credentials there.

static const char *OTA_HOSTNAME =
    "giv-bms-emulator";

bool otaStarted = false;
bool wifiWasConnected = false;

WebServer server(80);


// ============================================================
// V1.3 ROBUSTNESS: WATCHDOG, WIFI RECOVERY, RESTART DIAGNOSTICS
// ============================================================

// The board dropped off the network overnight on 27 Sep and 12 Oct and
// stayed off until power-cycled. An ESP32 crash reboots by itself, so
// staying off means either a hang or WiFi that never came back.
// The watchdog covers the first, superviseWiFi() the second.

static constexpr uint32_t WDT_TIMEOUT_S = 10;                 // loop hang -> reboot
static constexpr uint32_t WIFI_RETRY_MS = 60UL * 1000;        // restart WiFi after 1 min down
static constexpr uint32_t WIFI_REBOOT_MS = 10UL * 60 * 1000;  // restart board after 10 min down

// Why we restarted ourselves. RTC_NOINIT survives a software restart,
// not a power cut, so it is only trusted when the reset reason is SW.
static constexpr uint32_t CAUSE_NONE = 0;
static constexpr uint32_t CAUSE_WIFI_LOST = 0x57494649;  // "WIFI"
RTC_NOINIT_ATTR uint32_t rtcRebootCause;

Preferences prefs;

uint32_t bootCount = 0;
esp_reset_reason_t resetReason = ESP_RST_UNKNOWN;
uint32_t previousCause = CAUSE_NONE;

// Last abnormal restart, kept in flash so a later power cycle
// doesn't wipe the evidence.
esp_reset_reason_t lastAbnormalReason = ESP_RST_UNKNOWN;
uint32_t lastAbnormalCause = CAUSE_NONE;
uint32_t lastAbnormalBoot = 0;

bool servicesRegistered = false;
uint32_t wifiDownSince = 0;  // millis() when WiFi was lost, 0 = connected
uint32_t lastWifiRetry = 0;
uint32_t wifiDropCount = 0;
volatile uint8_t lastWifiDisconnectReason = 0;


const char *resetReasonText(esp_reset_reason_t r)
{
  switch (r)
  {
    case ESP_RST_POWERON:   return "power on";
    case ESP_RST_EXT:       return "external reset";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "CRASH (panic)";
    case ESP_RST_INT_WDT:   return "HANG (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "HANG (task watchdog)";
    case ESP_RST_WDT:       return "HANG (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (supply dipped)";
    default:                return "unknown";
  }
}


String restartText(esp_reset_reason_t reason, uint32_t cause)
{
  if (reason == ESP_RST_SW && cause == CAUSE_WIFI_LOST)
    return "WiFi lost for 10 min - self restart";

  return resetReasonText(reason);
}


bool isAbnormal(esp_reset_reason_t reason, uint32_t cause)
{
  return reason == ESP_RST_PANIC ||
         reason == ESP_RST_INT_WDT ||
         reason == ESP_RST_TASK_WDT ||
         reason == ESP_RST_WDT ||
         reason == ESP_RST_BROWNOUT ||
         cause == CAUSE_WIFI_LOST;
}


void recordBoot()
{
  resetReason = esp_reset_reason();

  previousCause =
      (resetReason == ESP_RST_SW) ? rtcRebootCause : CAUSE_NONE;

  rtcRebootCause = CAUSE_NONE;

  prefs.begin("bmsemu", false);

  bootCount = prefs.getUInt("boots", 0) + 1;
  prefs.putUInt("boots", bootCount);

  if (isAbnormal(resetReason, previousCause))
  {
    prefs.putUInt("badReason", resetReason);
    prefs.putUInt("badCause", previousCause);
    prefs.putUInt("badBoot", bootCount);
  }

  lastAbnormalReason =
      static_cast<esp_reset_reason_t>(prefs.getUInt("badReason", ESP_RST_UNKNOWN));

  lastAbnormalCause = prefs.getUInt("badCause", CAUSE_NONE);
  lastAbnormalBoot = prefs.getUInt("badBoot", 0);

  prefs.end();
}


String lastAbnormalText()
{
  if (lastAbnormalBoot == 0)
    return "none recorded";

  return restartText(lastAbnormalReason, lastAbnormalCause) +
         " (boot #" + String(lastAbnormalBoot) + ")";
}


void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
    lastWifiDisconnectReason = info.wifi_sta_disconnected.reason;
}


void setupWatchdog()
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t config =
  {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };

  esp_task_wdt_reconfigure(&config);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif

  // Watch the loop task: if loop() stops coming round for
  // WDT_TIMEOUT_S, the chip resets and records TASK_WDT.
  esp_task_wdt_add(NULL);
}


// ============================================================
// HARDWARE
// ============================================================

static constexpr int RS485_RX_PIN = 21;
static constexpr int RS485_TX_PIN = 22;

static constexpr int PIN_16 = 16;
static constexpr int PIN_17 = 17;
static constexpr int PIN_19 = 19;

static constexpr uint32_t RS485_BAUD = 9600;
static constexpr uint32_t USB_BAUD = 115200;

static constexpr uint32_t FRAME_GAP_US = 4000;
static constexpr uint32_t RESPONSE_DELAY_MS = 50;

static constexpr size_t RX_MAX = 128;

uint8_t rxBuf[RX_MAX];
uint32_t rxTimes[RX_MAX];

size_t rxLen = 0;
uint32_t lastRxUs = 0;


// ============================================================
// COUNTERS
// ============================================================

uint32_t rxByteCount = 0;
uint32_t rxFrameCount = 0;
uint32_t goodFrameCount = 0;
uint32_t badCrcCount = 0;
uint32_t replyCount = 0;
uint32_t unsupportedCount = 0;

uint32_t fc03Count = 0;
uint32_t fc04Count = 0;
uint32_t fc06Count = 0;

uint32_t lastHeartbeat = 0;


// ============================================================
// LAST FRAME STORAGE FOR WEB PAGE
// ============================================================

uint8_t lastRxFrame[RX_MAX];
size_t lastRxFrameLen = 0;

uint8_t lastTxFrame[128];
size_t lastTxFrameLen = 0;

uint8_t lastDevice = 0;
uint8_t lastFunction = 0;
uint16_t lastRegister = 0;
uint16_t lastValueOrCount = 0;

uint32_t lastFrameMillis = 0;

// ============================================================
// RX HISTORY - LAST 16 CAPTURED CHUNKS
// ============================================================

static constexpr int RX_HISTORY_COUNT = 16;

struct RxHistoryEntry
{
  uint8_t data[RX_MAX];
  size_t len;
  uint32_t capturedMillis;
};

RxHistoryEntry rxHistory[RX_HISTORY_COUNT];

int rxHistoryHead = 0;
int rxHistoryUsed = 0;


void addRxHistory(const uint8_t *data, size_t len)
{
  RxHistoryEntry &entry = rxHistory[rxHistoryHead];

  entry.len = min(len, sizeof(entry.data));

  memcpy(
      entry.data,
      data,
      entry.len);

  entry.capturedMillis = millis();

  rxHistoryHead =
      (rxHistoryHead + 1) % RX_HISTORY_COUNT;

  if (rxHistoryUsed < RX_HISTORY_COUNT)
    rxHistoryUsed++;
}


// ============================================================
// EMULATED CELL DATA
// ============================================================

static constexpr int CELL_COUNT = 16;

uint16_t cellMv[CELL_COUNT] =
{
  3293,
  3294,
  3295,
  3296,
  3297,
  3298,
  3299,
  3300,
  3301,
  3302,
  3303,
  3304,
  3305,
  3306,
  3307,
  3308
};


// ============================================================
// MODBUS CRC16
// ============================================================

uint16_t modbusCRC(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;

  for (size_t pos = 0; pos < len; pos++)
  {
    crc ^= data[pos];

    for (uint8_t i = 0; i < 8; i++)
    {
      if (crc & 0x0001)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}


bool crcOK(const uint8_t *frame, size_t len)
{
  if (len < 4)
    return false;

  uint16_t calculated =
      modbusCRC(frame, len - 2);

  uint16_t received =
      static_cast<uint16_t>(frame[len - 2]) |
      (static_cast<uint16_t>(frame[len - 1]) << 8);

  return calculated == received;
}


// ============================================================
// DEBUG PRINTING
// ============================================================

void printHex(const uint8_t *data, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    if (data[i] < 0x10)
      Serial.print('0');

    Serial.print(data[i], HEX);

    if (i + 1 < len)
      Serial.print(' ');
  }
}


String frameToHex(const uint8_t *data, size_t len)
{
  String out;

  out.reserve(len * 3);

  char b[4];

  for (size_t i = 0; i < len; i++)
  {
    snprintf(
        b,
        sizeof(b),
        "%02X",
        data[i]);

    out += b;

    if (i + 1 < len)
      out += ' ';
  }

  return out;
}


void printCapturedBytes()
{
  Serial.println();

  Serial.printf(
      "RX FRAME #%lu  bytes=%u\n",
      static_cast<unsigned long>(rxFrameCount),
      static_cast<unsigned>(rxLen));

  Serial.print("  HEX: ");
  printHex(rxBuf, rxLen);
  Serial.println();
}


// ============================================================
// HOLDING REGISTERS HR0..HR27
// ============================================================

uint16_t holdingRegs[28];

// ============================================================
// V1.2 FC04 MAPPING LAB
// ============================================================

// Experimental only.
// Keep inverter charge/discharge disabled while mapping.
// Use conservative limits for functional tests.

uint16_t experimentHR25 = 9000;


void initialiseHoldingRegisters()
{
  for (int i = 0; i < 28; i++)
    holdingRegs[i] = 0;

  holdingRegs[0] =
      0x0065;

  holdingRegs[1] = 0xFFFF;
  holdingRegs[2] = 0xFFFF;
  holdingRegs[3] = 0xFFFF;
  holdingRegs[4] = 0xFFFF;

  // HR5..HR9 = 10 byte ASCII serial

  const char serial10[10] =
  {
    'B','G','1','2','3',
    '4','G','5','6','7'
  };

  for (int i = 0; i < 5; i++)
  {
    holdingRegs[5 + i] =
        (static_cast<uint16_t>(
            serial10[i * 2]) << 8) |
        static_cast<uint8_t>(
            serial10[i * 2 + 1]);
  }

  holdingRegs[10] = 0xFFFF;

  holdingRegs[11] = 100;
  holdingRegs[12] = 48;
  holdingRegs[13] = 3022;

  holdingRegs[14] = 0;
  holdingRegs[15] = 0;
  holdingRegs[16] = 0;

  holdingRegs[17] = 0x114B;
  holdingRegs[18] = 0x389D;

  holdingRegs[19] =
      0x00CE;

  holdingRegs[20] =
      0x0000;

  // ----------------------------------------------------------
  // DISTINCTIVE TEST BATTERY VALUES
  // ----------------------------------------------------------

  holdingRegs[21] =
      63;       // SOC = 63%

  holdingRegs[22] =
      5280;     // 52.80 V

  holdingRegs[23] =
      0;        // zero current

  holdingRegs[24] =
      25;       // 25 C

  holdingRegs[25] =
    experimentHR25;

  // Conservative initial limits

  holdingRegs[26] =
      0;

  holdingRegs[27] =
      0;
}


// ============================================================
// RS485 TRANSMIT
// ============================================================

void sendFrame(uint8_t *frame, size_t lenWithoutCRC)
{
  uint16_t crc =
      modbusCRC(frame, lenWithoutCRC);

  frame[lenWithoutCRC] =
      crc & 0xFF;

  frame[lenWithoutCRC + 1] =
      (crc >> 8) & 0xFF;

  size_t total =
      lenWithoutCRC + 2;

  // Save copy for web diagnostics

  lastTxFrameLen =
      min(total, sizeof(lastTxFrame));

  memcpy(
      lastTxFrame,
      frame,
      lastTxFrameLen);

  Serial.print("TX: ");
  printHex(frame, total);
  Serial.println();

  delay(RESPONSE_DELAY_MS);

  // GPIO16 / GPIO17 / GPIO19 remain HIGH

  Serial1.write(
      frame,
      total);

  Serial1.flush();

  replyCount++;
}


void sendException(
    uint8_t device,
    uint8_t fc,
    uint8_t exceptionCode)
{
  uint8_t response[5];

  response[0] = device;
  response[1] = fc | 0x80;
  response[2] = exceptionCode;

  sendFrame(
      response,
      3);
}


// ============================================================
// FC03
// ============================================================

void handleFC03(
    uint8_t device,
    uint16_t start,
    uint16_t count)
{
  fc03Count++;

  if (device != 1)
  {
    Serial.println(
        "FC03 ignored: HR polling expected on device 1");

    return;
  }

  if (count == 0 ||
      start >= 28 ||
      (start + count) > 28)
  {
    Serial.println(
        "FC03 invalid register range");

    sendException(
        device,
        0x03,
        0x02);

    return;
  }

  Serial.printf(
      "FC03 VALID: device=%u start=%u count=%u\n",
      device,
      start,
      count);

  uint8_t response[
      3 + (28 * 2) + 2];

  size_t p = 0;

  response[p++] = device;
  response[p++] = 0x03;

  response[p++] =
      static_cast<uint8_t>(
          count * 2);

  for (uint16_t i = 0;
       i < count;
       i++)
  {
    uint16_t value =
        holdingRegs[start + i];

    response[p++] =
        (value >> 8) & 0xFF;

    response[p++] =
        value & 0xFF;
  }

  sendFrame(
      response,
      p);
}


// ============================================================
// FC04 PAYLOAD BUILDERS
// ============================================================

void buildPresentBlock1(uint8_t *d)
{
  memset(d, 0, 42);

  const char serial[20] =
  {
    'E','M','2','0','2','6','0','0','0','1',
    ' ',' ',' ',' ',' ',' ',' ',' ',' ',' '
  };

  memcpy(
      d,
      serial,
      20);

  // Existing temperature/status test data

  for (int i = 0; i < 5; i++)
  {
    d[22 + i * 2] = 0x00;
    d[23 + i * 2] = 0xC8;
  }

  d[32] = 0x00;
  d[33] = 0x01;

  d[34] = 0x00;
  d[35] = 0x08;
}


// ============================================================
// V1.3 FC04 BLOCK 2 - CONFIRMED BMS MAPPING
// ============================================================
//
// Offsets below are zero-based within the 38-byte Block 2 PAYLOAD
// (after the existing 4-byte FC04 response header), matching the
// confirmed GivEnergy BMS mapping. Capacity fields are genuine
// 4-byte big-endian uint32 fields, units 0.01 Ah - NOT two separate
// 2-byte groups as the earlier experimental lab treated them.
// Values are volatile: reboot restores the known-good 100.00 Ah /
// 100.00 Ah / 50.00 Ah, SOC 50% defaults below.

static constexpr uint32_t BLOCK2_CAPACITY_RAW_MAX = 999999; // 9999.99 Ah practical cap (field itself is uint32)

uint32_t block2CalibratedCapacityRaw = 10000; // 100.00 Ah
uint32_t block2DesignCapacityRaw     = 10000; // 100.00 Ah
uint32_t block2RemainingCapacityRaw  = 5000;  // 50.00 Ah
uint8_t  block2Soc                   = 50;    // 50 %

// Offset 26 (1 byte, UNKNOWN) and status fields 27/29/31/33 (2 bytes each).
// Bit-level semantics beyond the documented observations are NOT known -
// these remain raw, directly-settable values, default 0x0000.
static const uint8_t  statusOffsets[] = {26, 27, 29, 31, 33};
static const uint8_t  statusWidths[]  = { 1,  2,  2,  2,  2};
static constexpr size_t STATUS_COUNT  = sizeof(statusOffsets);
uint16_t statusValues[STATUS_COUNT]   = {}; // all default 0

void writeU32BE(uint8_t *d, size_t offset, uint32_t value)
{
  d[offset]     = (value >> 24) & 0xFF;
  d[offset + 1] = (value >> 16) & 0xFF;
  d[offset + 2] = (value >> 8) & 0xFF;
  d[offset + 3] = value & 0xFF;
}

void buildPresentBlock2(uint8_t *d)
{
  const uint8_t templateData[38] =
  {
    0x10,
    0x00, 0x00,
    0x00, 0x00,
    0xCB, 0x20,
    0xCB, 0x20,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x27, 0x10,
    0x00, 0x00,
    0x27, 0x10,
    0x00, 0x00,
    0x13, 0x88,
    0x32,
    0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x0B, 0xCE,
    0x00
  };

  memcpy(
      d,
      templateData,
      sizeof(templateData));

  // Confirmed 4-byte big-endian uint32 capacity fields, units 0.01 Ah.
  writeU32BE(d, 13, block2CalibratedCapacityRaw);
  writeU32BE(d, 17, block2DesignCapacityRaw);
  writeU32BE(d, 21, block2RemainingCapacityRaw);

  // SOC: direct 0-100 percentage byte.
  d[25] = block2Soc;

  // Offset 26 (UNKNOWN) + status fields 27/29/31/33: raw, default 0.
  for (size_t i = 0; i < STATUS_COUNT; ++i)
  {
    const uint8_t offset = statusOffsets[i];
    if (statusWidths[i] == 2) {
      d[offset] = (statusValues[i] >> 8) & 0xFF;
      d[offset + 1] = statusValues[i] & 0xFF;
    } else {
      d[offset] = statusValues[i] & 0xFF;
    }
  }
}


void buildPresentBlock3(uint8_t *d)
{
  memset(
      d,
      0,
      40);

  // 16 deliberately distinctive cell voltages

  for (int i = 0; i < CELL_COUNT; i++)
  {
    d[i * 2] =
        (cellMv[i] >> 8) & 0xFF;

    d[i * 2 + 1] =
        cellMv[i] & 0xFF;
  }

  // Existing fields retained

  d[32] = 0x00;
  d[33] = 0xC8;

  d[34] = 0x00;
  d[35] = 0xC8;

  // Min / max cell

  uint16_t minCell =
      cellMv[0];

  uint16_t maxCell =
      cellMv[0];

  for (int i = 1; i < CELL_COUNT; i++)
  {
    if (cellMv[i] < minCell)
      minCell = cellMv[i];

    if (cellMv[i] > maxCell)
      maxCell = cellMv[i];
  }

  // CLAUDE.md / confirmed BMS mapping: offset 36 = HIGHEST cell, offset 38 =
  // LOWEST cell. V1.2 had these two swapped - fixed here (2026-10-13).

  d[36] =
      (maxCell >> 8) & 0xFF;

  d[37] =
      maxCell & 0xFF;

  d[38] =
      (minCell >> 8) & 0xFF;

  d[39] =
      minCell & 0xFF;
}


// ============================================================
// CAN BUS (TWAI) - DATOUBOSS / JBD UP16S015 PYLONTECH-LV PROTOCOL
// ============================================================
//
// Confirmed 2026-10-14 via a real candump capture on the Pi 4 bench rig -
// see Giv Battery Emulator/giv-docs/10-pylontech-can-reference.md for the
// full decode notes this is built against. Bus: 500 kbps, standard 11-bit
// IDs, each core frame repeating at roughly 1 Hz.
//
// IMPORTANT, confirmed empirically: the battery does not transmit AT ALL
// until it sees a 0x305 keepalive frame from "the inverter" side. This
// firmware sends that keepalive continuously once it's up, the same job
// the real GivEnergy inverter would do on the real CAN bus.

static constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_27;
static constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_26;
static constexpr int CAN_SPEED_SELECT_PIN = 23;  // LOW = high-speed mode, LilyGo T-CAN485 pinout

static constexpr uint32_t CAN_ID_LIMITS       = 0x351;
static constexpr uint32_t CAN_ID_SOC_SOH      = 0x355;
static constexpr uint32_t CAN_ID_VIT          = 0x356;
static constexpr uint32_t CAN_ID_ALARMS       = 0x359;
static constexpr uint32_t CAN_ID_ENABLE_FLAGS = 0x35C;
static constexpr uint32_t CAN_ID_MANUFACTURER = 0x35E;
static constexpr uint32_t CAN_ID_CELL_MINMAX  = 0x373;
static constexpr uint32_t CAN_ID_RATED_CAP    = 0x379;
static constexpr uint32_t CAN_ID_KEEPALIVE    = 0x305;  // sent BY us, to the battery

static constexpr uint32_t CAN_KEEPALIVE_INTERVAL_MS = 1000;

// A link is "live" once any core frame has been seen; it's treated as gone
// quiet (fail-safe below kicks in) if none of the core six arrive for this
// long - comfortably inside the ~30s the real BMS/inverter tolerates before
// giving up on a silent bus (see the reply-timing note in CLAUDE.md).
static constexpr uint32_t CAN_FAILSAFE_MS = 5000;

bool canDriverOk = false;
bool canLinkEverEstablished = false;
bool canFailsafeActive = false;
uint32_t canLastCoreFrameMillis = 0;
uint32_t canLastKeepaliveMillis = 0;
uint32_t canFramesSeen = 0;
uint32_t canFailsafeTrips = 0;

// Decoded live values, kept in the CAN wire's own units (0.1A / 0.1V /
// 0.1degC counts) - converted to HR/IR scale only in applyCanLiveData().
int16_t  canChargeVoltLimit_0p1V    = 0;
int16_t  canChargeCurrLimit_0p1A    = 0;
int16_t  canDischargeCurrLimit_0p1A = 0;
int16_t  canDischargeVoltLimit_0p1V = 0;

uint16_t canSoc_pct = 0;
uint16_t canSoh_pct = 0;

int16_t canPackVoltage_0p01V = 0;
int16_t canPackCurrent_0p1A  = 0;
int16_t canTemp_0p1C         = 0;

uint16_t canProtectionFlags = 0;
uint16_t canWarningFlags    = 0;

uint8_t canEnableFlagsRaw = 0;

uint16_t canMinCellMv = 0;
uint16_t canMaxCellMv = 0;
uint16_t canMinTempK  = 0;
uint16_t canMaxTempK  = 0;

uint16_t canRatedCapacityAh = 0;

// Latching alarm state for the thresholds below - a plain comparison would
// chatter right at the trip point, so each one latches and only clears at
// the battery's own documented RECOVERY point, not the trip point again.
bool alarmOverVoltageLatched     = false;
bool alarmUnderVoltageLatched    = false;
bool alarmOverCurrentLatched     = false;
bool alarmChargeUnderTempLatched = false;

static inline int16_t canLe16s(const uint8_t *d)
{
  return static_cast<int16_t>(
      static_cast<uint16_t>(d[0]) |
      (static_cast<uint16_t>(d[1]) << 8));
}

static inline uint16_t canLe16u(const uint8_t *d)
{
  return static_cast<uint16_t>(d[0]) |
         (static_cast<uint16_t>(d[1]) << 8);
}

void initCan()
{
  pinMode(CAN_SPEED_SELECT_PIN, OUTPUT);
  digitalWrite(CAN_SPEED_SELECT_PIN, LOW);  // high-speed mode - DO NOT CHANGE

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
  {
    Serial.println("CAN: driver install FAILED");
    return;
  }

  if (twai_start() != ESP_OK)
  {
    Serial.println("CAN: twai_start FAILED");
    return;
  }

  canDriverOk = true;

  Serial.println("CAN: TWAI started, 500 kbps, TX=GPIO27 RX=GPIO26");
}

void sendCanKeepalive()
{
  twai_message_t msg = {};

  msg.identifier = CAN_ID_KEEPALIVE;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;
  memset(msg.data, 0, 8);

  twai_transmit(&msg, pdMS_TO_TICKS(10));
}

void decodeCanFrame(const twai_message_t &msg)
{
  const uint8_t *d = msg.data;
  uint8_t len = msg.data_length_code;

  bool isCoreFrame = false;

  switch (msg.identifier)
  {
    case CAN_ID_LIMITS:
      if (len >= 8)
      {
        canChargeVoltLimit_0p1V    = canLe16u(d + 0);
        canChargeCurrLimit_0p1A    = canLe16s(d + 2);
        canDischargeCurrLimit_0p1A = canLe16s(d + 4);
        canDischargeVoltLimit_0p1V = canLe16u(d + 6);
        isCoreFrame = true;
      }
      break;

    case CAN_ID_SOC_SOH:
      if (len >= 4)
      {
        canSoc_pct = canLe16u(d + 0);
        canSoh_pct = canLe16u(d + 2);
        isCoreFrame = true;
      }
      break;

    case CAN_ID_VIT:
      if (len >= 6)
      {
        canPackVoltage_0p01V = canLe16s(d + 0);
        canPackCurrent_0p1A  = canLe16s(d + 2);
        canTemp_0p1C         = canLe16s(d + 4);
        isCoreFrame = true;
      }
      break;

    case CAN_ID_ALARMS:
      if (len >= 4)
      {
        canProtectionFlags = canLe16u(d + 0);
        canWarningFlags    = canLe16u(d + 2);
        isCoreFrame = true;
      }
      break;

    case CAN_ID_ENABLE_FLAGS:
      if (len >= 1)
      {
        canEnableFlagsRaw = d[0];
        isCoreFrame = true;
      }
      break;

    case CAN_ID_MANUFACTURER:
      // "JBD" string, confirmed - identity only, nothing to decode into registers.
      isCoreFrame = true;
      break;

    case CAN_ID_CELL_MINMAX:
      if (len >= 8)
      {
        canMinCellMv = canLe16u(d + 0);
        canMaxCellMv = canLe16u(d + 2);
        canMinTempK  = canLe16u(d + 4);
        canMaxTempK  = canLe16u(d + 6);
      }
      break;

    case CAN_ID_RATED_CAP:
      if (len >= 2)
      {
        canRatedCapacityAh = canLe16u(d + 0);
      }
      break;

    default:
      // Unidentified ID (0x35F, 0x370-0x372, 0x374-0x377, or anything else
      // seen on the bus) - not decoded yet, see the reference doc.
      break;
  }

  if (isCoreFrame)
  {
    canLastCoreFrameMillis = millis();
    canLinkEverEstablished = true;
  }
}

// ----------------------------------------------------------
// HR20 alarm bits
// ----------------------------------------------------------
// Bit numbering per the confirmed HR20 table in CLAUDE.md (0-indexed,
// GivTCP battery_fault_code names):
//   0 Over Current   1 Short Current   2 Over Voltage ("full")
//   3 Under Voltage ("empty")   4 Discharge over-temp   5 Charge over-temp
//   6 Discharge under-temp   7 Charge under-temp
//
// Only bits 2/3 get a confirmed DSP response on the GivEnergy side, so
// they get the most weight. Thresholds below are the Datouboss's own
// documented protection spec (CLAUDE.md hardware section) wherever we
// have one - not invented numbers - except the OC recovery point, which
// isn't documented and is a conservative estimate (flagged below). Bits
// 1/4/5/6 have no documented trip point at all yet and are deliberately
// left clear rather than guessed.

static constexpr float DATOUBOSS_OV_TRIP_V           = 58.4f;
static constexpr float DATOUBOSS_OV_RECOVER_V        = 54.0f;
static constexpr float DATOUBOSS_UV_TRIP_V           = 43.2f;
static constexpr float DATOUBOSS_UV_RECOVER_V        = 46.4f;
static constexpr float DATOUBOSS_OC_TRIP_A           = 157.0f;
static constexpr float DATOUBOSS_OC_RECOVER_A        = 140.0f;  // estimated - no documented recovery point
static constexpr float DATOUBOSS_CHARGE_UT_TRIP_C    = 0.0f;
static constexpr float DATOUBOSS_CHARGE_UT_RECOVER_C = 5.0f;

uint8_t computeHR20FromCan()
{
  float voltage = canPackVoltage_0p01V / 100.0f;
  float current = canPackCurrent_0p1A / 10.0f;  // sign not yet verified - see applyCanLiveData()
  float tempC   = canTemp_0p1C / 10.0f;

  if (voltage >= DATOUBOSS_OV_TRIP_V) alarmOverVoltageLatched = true;
  else if (voltage <= DATOUBOSS_OV_RECOVER_V) alarmOverVoltageLatched = false;

  if (voltage <= DATOUBOSS_UV_TRIP_V) alarmUnderVoltageLatched = true;
  else if (voltage >= DATOUBOSS_UV_RECOVER_V) alarmUnderVoltageLatched = false;

  float absCurrent = fabsf(current);
  if (absCurrent >= DATOUBOSS_OC_TRIP_A) alarmOverCurrentLatched = true;
  else if (absCurrent <= DATOUBOSS_OC_RECOVER_A) alarmOverCurrentLatched = false;

  if (tempC <= DATOUBOSS_CHARGE_UT_TRIP_C) alarmChargeUnderTempLatched = true;
  else if (tempC >= DATOUBOSS_CHARGE_UT_RECOVER_C) alarmChargeUnderTempLatched = false;

  uint8_t bits = 0;

  if (alarmOverCurrentLatched)     bits |= (1 << 0);  // Over Current
  if (alarmOverVoltageLatched)     bits |= (1 << 2);  // Over Voltage / "full"
  if (alarmUnderVoltageLatched)    bits |= (1 << 3);  // Under Voltage / "empty"
  if (alarmChargeUnderTempLatched) bits |= (1 << 7);  // Charge under-temp

  // Defense in depth: if the BMS itself is flagging ANY protection or
  // warning bit we haven't individually decoded yet, don't silently drop
  // it - fall back to the conservative "stop charging, limit discharge"
  // pair until the real 0x359 bit layout is confirmed.
  if (canProtectionFlags != 0 || canWarningFlags != 0)
    bits |= (1 << 2) | (1 << 3);

  return bits;
}

// ----------------------------------------------------------
// Push decoded CAN data into the registers the inverter actually reads
// ----------------------------------------------------------

void applyCanLiveData()
{
  // HR22: pack voltage, 0.01V/count - same units as the CAN field, direct copy.
  holdingRegs[22] = static_cast<uint16_t>(canPackVoltage_0p01V);

  // HR23: current, 0.01A/count, "+ = charging" (GivEnergy convention).
  // CAN 0x356's current sign is NOT yet verified against a real charge or
  // discharge - this capture was taken at 0A, idle. If HR23 reads backwards
  // once real current flows, negate canPackCurrent_0p1A here.
  int32_t current_0p01A = static_cast<int32_t>(canPackCurrent_0p1A) * 10;
  holdingRegs[23] = static_cast<uint16_t>(static_cast<int16_t>(current_0p01A));

  // HR24: temperature, whole degC.
  holdingRegs[24] = static_cast<uint16_t>(canTemp_0p1C / 10);

  // HR25: single current limit, 0.01A/count. Gen 1 can't represent charge
  // and discharge limits separately (decided 2026-10-14, CLAUDE.md) - take
  // whichever of the two CAN limits is lower, so HR25 never claims more
  // headroom than the battery allows in either direction.
  int32_t chargeLimitAbs    = abs(static_cast<int32_t>(canChargeCurrLimit_0p1A));
  int32_t dischargeLimitAbs = abs(static_cast<int32_t>(canDischargeCurrLimit_0p1A));
  int32_t lowerLimit_0p1A   =
      (chargeLimitAbs < dischargeLimitAbs) ? chargeLimitAbs : dischargeLimitAbs;
  int32_t limit_0p01A = lowerLimit_0p1A * 10;
  if (limit_0p01A > 65535) limit_0p01A = 65535;
  holdingRegs[25] = static_cast<uint16_t>(limit_0p01A);

  // HR21 / FC04 Block 2 SOC: direct pass-through for now. The inverter's
  // hard 4% discharge floor means this needs deliberate scaling against
  // the Datouboss's real usable range before go-live (see the SoC pitfall
  // note in CLAUDE.md) - flagged here, not solved here.
  uint16_t socClamped = (canSoc_pct > 100) ? 100 : canSoc_pct;
  holdingRegs[21] = socClamped;
  block2Soc = static_cast<uint8_t>(socClamped);

  // FC04 Block 2 capacities - only touch these once the battery has told
  // us its rated capacity (0x379); otherwise leave the existing lab
  // defaults alone rather than overwrite them with zero.
  if (canRatedCapacityAh > 0)
  {
    uint32_t ratedRaw = static_cast<uint32_t>(canRatedCapacityAh) * 100;  // Ah -> raw 0.01Ah
    block2CalibratedCapacityRaw = ratedRaw;
    block2DesignCapacityRaw     = ratedRaw;
    block2RemainingCapacityRaw  =
        static_cast<uint32_t>(
            (static_cast<uint64_t>(ratedRaw) * block2Soc) / 100);
  }

  // Cell voltages: the Pylontech-LV set only carries pack-level min/max
  // (0x373), not all 16 individually - confirmed on the 2026-10-14
  // capture. Put the real min at one end and the real max at the other so
  // buildPresentBlock3()'s own min/max scan reports the true extremes;
  // fill the rest with the midpoint since there's no per-cell data to put
  // there.
  if (canMinCellMv > 0 && canMaxCellMv > 0)
  {
    uint16_t mid = static_cast<uint16_t>((canMinCellMv + canMaxCellMv) / 2);

    for (int i = 0; i < CELL_COUNT; i++)
      cellMv[i] = mid;

    cellMv[0] = canMinCellMv;
    cellMv[CELL_COUNT - 1] = canMaxCellMv;
  }

  // HR20: alarm bits, computed from real decoded values against the
  // Datouboss's documented protection spec.
  holdingRegs[20] = computeHR20FromCan();
}

// ----------------------------------------------------------
// Fail-safe: the CAN link was live and has gone quiet
// ----------------------------------------------------------
// "Never repeat stale values" (CLAUDE.md) - once we've genuinely heard
// from the battery, silence longer than CAN_FAILSAFE_MS means something's
// wrong (cable, battery fault, the keepalive not landing), and the safe
// response is the same one a dead/disconnected battery should produce:
// zero current limit, "full" and "empty" both raised, so the inverter
// neither charges nor discharges against a battery it can no longer hear.

void applyCanFailsafe()
{
  holdingRegs[25] = 0;
  holdingRegs[20] |= (1 << 2) | (1 << 3);
}

void serviceCan()
{
  if (!canDriverOk)
    return;

  uint32_t now = millis();

  // Send the keepalive the battery needs to see before it'll transmit at
  // all (confirmed empirically 2026-10-14) - without this the bus stays
  // completely silent regardless of wiring or protocol setting.
  if (now - canLastKeepaliveMillis >= CAN_KEEPALIVE_INTERVAL_MS)
  {
    canLastKeepaliveMillis = now;
    sendCanKeepalive();
  }

  twai_message_t msg;

  while (twai_receive(&msg, 0) == ESP_OK)
  {
    canFramesSeen++;
    decodeCanFrame(msg);
  }

  // Before any live CAN data has ever arrived, deliberately leave
  // holdingRegs/cellMv/block2 alone - this keeps the existing FC04 mapping
  // lab's test values and web controls working for bench testing with
  // nothing on the CAN bus, rather than forcing a fail-safe state that was
  // only ever meant for "was live, then went quiet".
  if (!canLinkEverEstablished)
    return;

  if (now - canLastCoreFrameMillis < CAN_FAILSAFE_MS)
  {
    if (canFailsafeActive)
    {
      canFailsafeActive = false;
      Serial.println("CAN: link recovered - live data resumed");
    }

    applyCanLiveData();
  }
  else
  {
    if (!canFailsafeActive)
    {
      canFailsafeActive = true;
      canFailsafeTrips++;
      Serial.println("CAN: link gone quiet - fail-safe applied (HR25=0, HR20 full+empty)");
    }

    applyCanFailsafe();
  }
}


// ============================================================
// ABSENT DEVICE PAYLOADS
// ============================================================

void buildAbsentBlock1(uint8_t *d)
{
  memset(
      d,
      0,
      42);

  for (int i = 0; i < 5; i++)
  {
    d[22 + i * 2] = 0xF5;
    d[23 + i * 2] = 0x56;
  }
}


void buildAbsentBlock2(uint8_t *d)
{
  memset(
      d,
      0,
      38);
}


void buildAbsentBlock3(uint8_t *d)
{
  memset(
      d,
      0,
      40);

  d[32] = 0xF5;
  d[33] = 0x56;

  d[34] = 0xF5;
  d[35] = 0x56;
}


// ============================================================
// FC04
// GivEnergy NON-STANDARD:
// device FC start_hi start_lo data CRC
// ============================================================

void handleFC04(
    uint8_t device,
    uint16_t start,
    uint16_t count)
{
  fc04Count++;

  if (device < 1 ||
      device > 5)
  {
    Serial.printf(
        "FC04 ignored: unsupported device %u\n",
        device);

    return;
  }

  Serial.printf(
      "FC04 VALID: device=%u start=0x%04X count=%u\n",
      device,
      start,
      count);

  uint8_t payload[42];

  size_t payloadLen = 0;

  bool present =
      (device == 1);

  if (start == 0x0000 &&
      count == 21)
  {
    payloadLen = 42;

    if (present)
      buildPresentBlock1(payload);
    else
      buildAbsentBlock1(payload);
  }

  else if (start == 0x0015 &&
           count == 19)
  {
    payloadLen = 38;

    if (present)
      buildPresentBlock2(payload);
    else
      buildAbsentBlock2(payload);
  }

  else if (start == 0x0028 &&
           count == 20)
  {
    payloadLen = 40;

    if (present)
      buildPresentBlock3(payload);
    else
      buildAbsentBlock3(payload);
  }

  else
  {
    Serial.println(
        "FC04 unknown block");

    sendException(
        device,
        0x04,
        0x02);

    return;
  }

  uint8_t response[
      4 + 42 + 2];

  size_t p = 0;

  response[p++] = device;
  response[p++] = 0x04;

  response[p++] =
      (start >> 8) & 0xFF;

  response[p++] =
      start & 0xFF;

  memcpy(
      &response[p],
      payload,
      payloadLen);

  p += payloadLen;

  sendFrame(
      response,
      p);
}


// ============================================================
// FC06
// ============================================================

void handleFC06(
    const uint8_t *request,
    size_t len)
{
  fc06Count++;

  if (len != 8)
    return;

  Serial.println(
      "FC06 write received - echoing unchanged");

  uint8_t response[8];

  memcpy(
      response,
      request,
      8);

  // Save TX for web diagnostics

  lastTxFrameLen = 8;

  memcpy(
      lastTxFrame,
      response,
      8);

  Serial.print(
      "TX FC06 ECHO: ");

  printHex(
      response,
      8);

  Serial.println();

  delay(
      RESPONSE_DELAY_MS);

  Serial1.write(
      response,
      8);

  Serial1.flush();

  replyCount++;
}


// ============================================================
// WEB PAGE
// ============================================================

String uptimeString()
{
  uint32_t seconds =
      millis() / 1000;

  uint32_t days =
      seconds / 86400;

  seconds %= 86400;

  uint32_t hours =
      seconds / 3600;

  seconds %= 3600;

  uint32_t minutes =
      seconds / 60;

  seconds %= 60;

  char result[32];

  snprintf(
      result,
      sizeof(result),
      "%lud %02lu:%02lu:%02lu",
      static_cast<unsigned long>(days),
      static_cast<unsigned long>(hours),
      static_cast<unsigned long>(minutes),
      static_cast<unsigned long>(seconds));

  return String(result);
}

// Strict decimal input: reject blanks, signs, suffixes and overflow.
bool parseRawDecimal(const String &text, uint16_t maximum, uint16_t &result)
{
  if (text.length() == 0 || text.length() > 5) return false;
  uint32_t value = 0;
  for (size_t i = 0; i < text.length(); ++i) {
    if (text[i] < '0' || text[i] > '9') return false;
    value = value * 10 + (text[i] - '0');
    if (value > maximum) return false;
  }
  result = static_cast<uint16_t>(value);
  return true;
}

// Strict Ah decimal input (e.g. "100", "100.0", "314.00"): digits, optional
// '.', up to 2 fractional digits. Returns the raw 0.01 Ah integer encoding.
bool parseAhToRaw(const String &text, uint32_t maximum, uint32_t &result)
{
  if (text.length() == 0 || text.length() > 12) return false;
  int dot = text.indexOf('.');
  String intPart = dot < 0 ? text : text.substring(0, dot);
  String fracPart = dot < 0 ? "" : text.substring(dot + 1);
  if (intPart.length() == 0 || intPart.length() > 9) return false;
  if (fracPart.length() > 2) return false;
  for (size_t i = 0; i < intPart.length(); ++i)
    if (intPart[i] < '0' || intPart[i] > '9') return false;
  for (size_t i = 0; i < fracPart.length(); ++i)
    if (fracPart[i] < '0' || fracPart[i] > '9') return false;
  uint64_t value = 0;
  for (size_t i = 0; i < intPart.length(); ++i)
    value = value * 10 + (intPart[i] - '0');
  value *= 100;
  if (fracPart.length() == 1) value += (fracPart[0] - '0') * 10;
  else if (fracPart.length() == 2) value += (fracPart[0] - '0') * 10 + (fracPart[1] - '0');
  if (value > maximum) return false;
  result = static_cast<uint32_t>(value);
  return true;
}

// Formats a raw 0.01 Ah integer as a fixed two-decimal Ah string using
// integer arithmetic only (no float rounding).
String formatAh(uint32_t raw)
{
  char buf[20];
  snprintf(buf, sizeof(buf), "%lu.%02lu",
           static_cast<unsigned long>(raw / 100),
           static_cast<unsigned long>(raw % 100));
  return String(buf);
}

String toHex8(uint8_t v)
{
  char buf[4];
  snprintf(buf, sizeof(buf), "%02X", v);
  return String(buf);
}

String toHex16(uint16_t v)
{
  char buf[6];
  snprintf(buf, sizeof(buf), "%04X", v);
  return String(buf);
}

void handleLabStatus()
{
  uint8_t payload[38];
  buildPresentBlock2(payload);
  String json = "{\"hr25\":" + String(holdingRegs[25]);
  json += ",\"good\":" + String(goodFrameCount);
  json += ",\"badCRC\":" + String(badCrcCount);
  json += ",\"replies\":" + String(replyCount);
  json += ",\"payload\":\"" + frameToHex(payload, sizeof(payload)) + "\"";
  json += ",\"calibratedRaw\":" + String(block2CalibratedCapacityRaw);
  json += ",\"designRaw\":" + String(block2DesignCapacityRaw);
  json += ",\"remainingRaw\":" + String(block2RemainingCapacityRaw);
  json += ",\"soc\":" + String(block2Soc);
  json += ",\"status26\":" + String(statusValues[0]);
  json += ",\"status27\":" + String(statusValues[1]);
  json += ",\"status29\":" + String(statusValues[2]);
  json += ",\"status31\":" + String(statusValues[3]);
  json += ",\"status33\":" + String(statusValues[4]);
  json += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void finishLabWrite()
{
  server.sendHeader("Cache-Control", "no-store");
  if (server.arg("ajax") == "1") { handleLabStatus(); return; }
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "Applied; return to /");
}

void handleSetCapacity()
{
  String field = server.arg("field");
  uint32_t raw;
  if (!parseAhToRaw(server.arg("ah"), BLOCK2_CAPACITY_RAW_MAX, raw)) {
    server.send(400, "text/plain", "Invalid Ah value; no change applied"); return;
  }
  if (field == "calibrated") block2CalibratedCapacityRaw = raw;
  else if (field == "design") block2DesignCapacityRaw = raw;
  else if (field == "remaining") block2RemainingCapacityRaw = raw;
  else { server.send(400, "text/plain", "Unknown capacity field; no change applied"); return; }
  Serial.printf("FC04 Block2 %s capacity -> %s Ah (raw=%lu)\n",
                field.c_str(), formatAh(raw).c_str(), static_cast<unsigned long>(raw));
  finishLabWrite();
}

void handleSetSoc()
{
  uint16_t value;
  if (!parseRawDecimal(server.arg("value"), 100, value)) {
    server.send(400, "text/plain", "SOC must be decimal 0..100; no change applied"); return;
  }
  block2Soc = static_cast<uint8_t>(value);
  Serial.printf("FC04 Block2 SOC -> %u%%\n", block2Soc);
  finishLabWrite();
}

void handleSyncSoc()
{
  uint8_t computed = 0;
  if (block2CalibratedCapacityRaw > 0) {
    uint64_t scaled = static_cast<uint64_t>(block2RemainingCapacityRaw) * 100
        + block2CalibratedCapacityRaw / 2;
    uint64_t pct = scaled / block2CalibratedCapacityRaw;
    computed = static_cast<uint8_t>(pct > 100 ? 100 : pct);
  }
  block2Soc = computed;
  Serial.printf("FC04 Block2 SOC synced from capacities -> %u%%\n", block2Soc);
  finishLabWrite();
}

void handleSetStatus()
{
  uint16_t offset, value;
  if (!parseRawDecimal(server.arg("offset"), 37, offset)) {
    server.send(400, "text/plain", "Invalid status offset"); return;
  }
  for (size_t i = 0; i < STATUS_COUNT; ++i) {
    if (statusOffsets[i] != offset) continue;
    uint16_t maxVal = statusWidths[i] == 1 ? 255 : 65535;
    if (!parseRawDecimal(server.arg("value"), maxVal, value)) {
      server.send(400, "text/plain", "Invalid raw decimal value; no change applied"); return;
    }
    statusValues[i] = value;
    Serial.printf("FC04 Block2 status offset=%u width=%u raw=%u (0x%0*X)\n",
                  offset, statusWidths[i], value, statusWidths[i] * 2, value);
    finishLabWrite(); return;
  }
  server.send(400, "text/plain", "Offset is not an exposed status field; no change applied");
}

// V1.3: small JSON for a Home Assistant REST sensor, so uptime and heap
// can be trended and a reboot or slow leak spotted before it falls over.
void handleHealth()
{
  String json;
  json.reserve(512);

  json += "{\"uptime_s\":" + String(millis() / 1000);
  json += ",\"boot_count\":" + String(bootCount);
  json += ",\"last_restart\":\"" + restartText(resetReason, previousCause) + "\"";
  json += ",\"last_abnormal\":\"" + lastAbnormalText() + "\"";
  json += ",\"free_heap\":" + String(ESP.getFreeHeap());
  json += ",\"min_free_heap\":" + String(ESP.getMinFreeHeap());
  json += ",\"largest_block\":" + String(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  json += ",\"wifi_rssi\":" + String(WiFi.RSSI());
  json += ",\"wifi_drops\":" + String(wifiDropCount);
  json += ",\"wifi_last_disconnect_reason\":" + String(lastWifiDisconnectReason);
  json += ",\"rs485_frames\":" + String(rxFrameCount);
  json += ",\"rs485_good\":" + String(goodFrameCount);
  json += ",\"rs485_last_frame_age_s\":";
  json += lastFrameMillis ? String((millis() - lastFrameMillis) / 1000.0, 1) : String("null");

  json += ",\"can_ok\":" + String(canDriverOk ? "true" : "false");
  json += ",\"can_link\":" + String(canLinkEverEstablished ? "true" : "false");
  json += ",\"can_failsafe\":" + String(canFailsafeActive ? "true" : "false");
  json += ",\"can_failsafe_trips\":" + String(canFailsafeTrips);
  json += ",\"can_frames_seen\":" + String(canFramesSeen);
  json += ",\"can_soc_pct\":" + String(canSoc_pct);
  json += ",\"can_pack_voltage_v\":" + String(canPackVoltage_0p01V / 100.0, 2);
  json += ",\"can_pack_current_a\":" + String(canPackCurrent_0p1A / 10.0, 1);
  json += ",\"can_temp_c\":" + String(canTemp_0p1C / 10.0, 1);
  json += ",\"can_last_frame_age_s\":";
  json += canLinkEverEstablished ? String((millis() - canLastCoreFrameMillis) / 1000.0, 1) : String("null");
  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleWebRoot()
{
  uint16_t minCell = cellMv[0];
  uint16_t maxCell = cellMv[0];
  uint32_t cellTotal = 0;

  for (int i = 0; i < CELL_COUNT; i++)
  {
    if (cellMv[i] < minCell)
      minCell = cellMv[i];

    if (cellMv[i] > maxCell)
      maxCell = cellMv[i];

    cellTotal += cellMv[i];
  }

  uint16_t delta = maxCell - minCell;

  // ----------------------------------------------------------
  // Work out current RS485 activity state
  // ----------------------------------------------------------

  bool haveTraffic = (rxByteCount > 0);

  float lastFrameAge = 0.0;

  if (lastFrameMillis > 0)
  {
    lastFrameAge =
        (millis() - lastFrameMillis) / 1000.0;
  }

  // Treat traffic within the last 2 seconds as ACTIVE.
  bool rs485Active =
      (lastFrameMillis > 0 &&
       (millis() - lastFrameMillis) < 2000);

  // ----------------------------------------------------------
  // Work out status of the last captured frame
  // ----------------------------------------------------------

  bool lastLengthOK =
      (lastRxFrameLen == 8);

  bool lastCrcOK = false;

  if (lastRxFrameLen >= 4)
  {
    lastCrcOK =
        crcOK(lastRxFrame, lastRxFrameLen);
  }

  uint8_t displayDevice = 0;
  uint8_t displayFunction = 0;
  uint16_t displayRegister = 0;
  uint16_t displayValueCount = 0;

  // Decode whatever we can, even if the frame is not valid.
  if (lastRxFrameLen >= 1)
    displayDevice = lastRxFrame[0];

  if (lastRxFrameLen >= 2)
    displayFunction = lastRxFrame[1];

  if (lastRxFrameLen >= 4)
  {
    displayRegister =
        (static_cast<uint16_t>(
            lastRxFrame[2]) << 8) |
        lastRxFrame[3];
  }

  if (lastRxFrameLen >= 6)
  {
    displayValueCount =
        (static_cast<uint16_t>(
            lastRxFrame[4]) << 8) |
        lastRxFrame[5];
  }

  // ----------------------------------------------------------
  // Build page
  // ----------------------------------------------------------

  String html;

  // V1.3: the page is large. Reserve it in one go rather than regrowing it
  // piece by piece (heap fragmentation), and skip the page rather than
  // crash if the heap can't supply it.
  if (!html.reserve(24000))
  {
    server.send(
        503,
        "text/plain",
        "Low memory - page skipped. Try /health");

    return;
  }

  html +=
      "<!DOCTYPE html>"
      "<html>"
      "<head>"
      "<meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>GivEnergy BMS Emulator V1.2R FC04 MAPPING LAB</title>"

      "<style>"

      "*{box-sizing:border-box;}"

      "body{"
      "font-family:Arial,sans-serif;"
      "background:#111827;"
      "color:#e5e7eb;"
      "margin:0;"
      "padding:16px;"
      "}"

      ".wrap{"
      "max-width:1400px;"
      "margin:auto;"
      "}"

      "h1{"
      "margin:0 0 4px 0;"
      "color:#7dd3fc;"
      "font-size:34px;"
      "}"

      ".version{"
      "color:#9ca3af;"
      "margin-bottom:14px;"
      "font-size:14px;"
      "}"

      "h2{"
      "color:#a5f3fc;"
      "margin:0 0 12px 0;"
      "font-size:24px;"
      "}"

      ".grid{"
      "display:grid;"
      "grid-template-columns:repeat(auto-fit,minmax(280px,1fr));"
      "gap:14px;"
      "}"

      ".topgrid{"
      "display:grid;"
      "grid-template-columns:1.1fr 1fr 1fr;"
      "gap:14px;"
      "}"

      ".card{"
      "background:#1f2937;"
      "border-radius:12px;"
      "padding:16px;"
      "box-shadow:0 3px 12px rgba(0,0,0,.35);"
      "}"

      ".hero{"
      "grid-column:1/-1;"
      "}"

      ".row{"
      "display:flex;"
      "justify-content:space-between;"
      "gap:12px;"
      "padding:5px 0;"
      "border-bottom:1px solid #374151;"
      "}"

      ".value{"
      "font-weight:bold;"
      "color:#86efac;"
      "}"

      ".bad{"
      "font-weight:bold;"
      "color:#fca5a5;"
      "}"

      ".warn{"
      "font-weight:bold;"
      "color:#fde68a;"
      "}"

      ".activity{"
      "font-size:26px;"
      "font-weight:bold;"
      "padding:12px 16px;"
      "border-radius:9px;"
      "margin-bottom:12px;"
      "}"

      ".active{"
      "background:#064e3b;"
      "color:#86efac;"
      "border:1px solid #10b981;"
      "}"

      ".idle{"
      "background:#3f3f46;"
      "color:#d1d5db;"
      "border:1px solid #71717a;"
      "}"

      ".never{"
      "background:#3f1d1d;"
      "color:#fca5a5;"
      "border:1px solid #ef4444;"
      "}"

      ".hex{"
      "font-family:Consolas,'Courier New',monospace;"
      "font-size:21px;"
      "font-weight:bold;"
      "line-height:1.55;"
      "white-space:pre-wrap;"
      "word-break:break-all;"
      "background:#030712;"
      "padding:14px;"
      "border-radius:8px;"
      "color:#fde68a;"
      "margin:8px 0 12px 0;"
      "min-height:54px;"
      "}"

      ".txhex{"
      "font-family:Consolas,'Courier New',monospace;"
      "font-size:16px;"
      "line-height:1.45;"
      "white-space:pre-wrap;"
      "word-break:break-all;"
      "background:#030712;"
      "padding:10px;"
      "border-radius:8px;"
      "color:#93c5fd;"
      "}"

      ".decode{"
      "display:grid;"
      "grid-template-columns:repeat(auto-fit,minmax(140px,1fr));"
      "gap:8px;"
      "margin-top:10px;"
      "}"

      ".decodebox{"
      "background:#111827;"
      "padding:10px;"
      "border-radius:7px;"
      "text-align:center;"
      "}"

      ".decodebox small{"
      "display:block;"
      "color:#9ca3af;"
      "margin-bottom:4px;"
      "}"

      ".decodebox strong{"
      "font-size:18px;"
      "color:#86efac;"
      "}"

      ".cells{"
      "display:grid;"
      "grid-template-columns:repeat(auto-fit,minmax(105px,1fr));"
      "gap:6px;"
      "}"

      ".cell{"
      "background:#111827;"
      "padding:8px;"
      "border-radius:6px;"
      "text-align:center;"
      "}"

      ".footer{"
      "text-align:center;"
      "color:#9ca3af;"
      "font-size:13px;"
      "margin:14px;"
      "}"

      "@media(max-width:900px){"
      ".topgrid{grid-template-columns:1fr;}"
      "h1{font-size:27px;}"
      ".hex{font-size:17px;}"
      "}"

      "</style>"
      "</head>"

      "<body>"
      "<div class='wrap'>"

      "<h1>GivEnergy BMS Emulator V1.2R FC04 MAPPING LAB</h1>"
      "<div class='version'>"
      "V1.2R FC04 MAPPING LAB • Confirmed Block 2 mapping controls, RS485 diagnostics, watchdog + WiFi recovery"
      "</div>";

  html += "<div class='card'><h2>FC04 Block 2 - Confirmed BMS Mapping</h2>"
          "<p>Device 1 only. Calibrated/Design/Remaining Capacity (offsets 13/17/21) are genuine "
          "4-byte big-endian fields, units 0.01 Ah - entered here in Ah to two decimal places. "
          "SOC (offset 25) is a direct 0-100% byte. Reboot restores the known-good "
          "100.00/100.00/50.00 Ah, SOC 50% defaults.</p>";

  uint8_t labPayload[38];
  buildPresentBlock2(labPayload);

  html += "<form action='/setcapacity' method='get'><label>Calibrated Capacity (Ah): "
          "<input name='ah' type='text' inputmode='decimal' required value='" +
          formatAh(block2CalibratedCapacityRaw) + "'></label>"
          "<input type='hidden' name='field' value='calibrated'>"
          " <button type='submit'>SET</button> Applied: <span id='calibratedApplied'>" +
          formatAh(block2CalibratedCapacityRaw) + "</span> Ah (default 100.00 Ah)</form>";

  html += "<form action='/setcapacity' method='get'><label>Design Capacity (Ah): "
          "<input name='ah' type='text' inputmode='decimal' required value='" +
          formatAh(block2DesignCapacityRaw) + "'></label>"
          "<input type='hidden' name='field' value='design'>"
          " <button type='submit'>SET</button> Applied: <span id='designApplied'>" +
          formatAh(block2DesignCapacityRaw) + "</span> Ah (default 100.00 Ah)</form>";

  html += "<form action='/setcapacity' method='get'><label>Remaining Capacity (Ah): "
          "<input name='ah' type='text' inputmode='decimal' required value='" +
          formatAh(block2RemainingCapacityRaw) + "'></label>"
          "<input type='hidden' name='field' value='remaining'>"
          " <button type='submit'>SET</button> Applied: <span id='remainingApplied'>" +
          formatAh(block2RemainingCapacityRaw) + "</span> Ah (default 50.00 Ah)</form>";

  html += "<form action='/setsoc' method='get'><label>State of Charge (%): "
          "<input name='value' type='number' required min='0' max='100' value='" +
          String(block2Soc) + "'></label>"
          " <button type='submit'>SET</button> Applied: <span id='socApplied'>" +
          String(block2Soc) + "</span> % (default 50%)</form>";

  html += "<form action='/syncsoc' method='get' style='margin-top:4px'>"
          "<button type='submit'>Sync SOC = round(Remaining &divide; Calibrated &times; 100)</button>"
          " <span style='color:#9ca3af'>(does not run automatically - capacities and SOC stay independent otherwise)</span>"
          "</form>";

  html += "<p class='bad' style='margin-top:12px'>Offset 26 and status fields 27/29/31/33: detailed bit "
          "semantics are not yet fully established. Only the raw observations below are confirmed:</p>"
          "<p style='color:#9ca3af;font-size:13px'>"
          "Status 1 (offset 27): bit 2 changes at min/max SOC during calibration; bit 3 changes at min SOC; "
          "bit 4 appears related to charge direction.<br>"
          "Status 2 (offset 29): bit 5 goes high at min SOC during calibration.<br>"
          "Status 3 (offset 31): bits 1 and 2 briefly go high at max SOC during calibration.<br>"
          "Status 4 (offset 33): bit 9 briefly goes high at max SOC during calibration; "
          "bit 10 stays high for a prolonged period at min SOC during calibration."
          "</p>";

  {
    static const char *statusLabels[STATUS_COUNT] =
    {
      "UNKNOWN offset 26",
      "Status 1 - offset 27",
      "Status 2 - offset 29",
      "Status 3 - offset 31",
      "Status 4 - offset 33"
    };
    static const char *statusIds[STATUS_COUNT] =
    {
      "status26Applied",
      "status27Applied",
      "status29Applied",
      "status31Applied",
      "status33Applied"
    };

    for (size_t i = 0; i < STATUS_COUNT; ++i)
    {
      uint16_t maxVal = statusWidths[i] == 1 ? 255 : 65535;
      String hexNow = statusWidths[i] == 1 ? toHex8(statusValues[i]) : toHex16(statusValues[i]);

      html += "<form action='/setstatus' method='get'><label>" + String(statusLabels[i]) +
              ": <input name='value' type='number' required min='0' max='" + String(maxVal) +
              "' value='" + String(statusValues[i]) + "'></label>";
      html += "<input type='hidden' name='offset' value='" + String(statusOffsets[i]) + "'>";
      html += " <button type='submit'>SET</button> Applied: <span id='" + String(statusIds[i]) + "'>" +
              String(statusValues[i]) + " (0x" + hexNow + ")</span> (default 0)</form>";
    }
  }

  html += "<h3>Current Block 2 Payload (38 bytes)</h3>"
          "<div class='hex' id='block2PayloadHex'>" +
          frameToHex(labPayload, sizeof(labPayload)) +
          "</div>";

  html += "<p>Direct URL fallback: <code>/setcapacity?field=calibrated&amp;ah=100.00</code>, "
          "<code>/setsoc?value=50</code> or "
          "<code>/sethr25?value=9000</code>. Values are not saved across reboot.</p>"
          "<p id='labMessage' role='status'>Ready</p><h3>Live applied values / counters</h3>"
          "<pre id='labLive' style='white-space:pre-wrap'></pre></div>";

  // ==========================================================
  // FC04 BLOCK 2 - OPTIONAL READ-ONLY DECODE
  // ==========================================================

  {
    uint16_t cycles = (static_cast<uint16_t>(labPayload[1]) << 8) | labPayload[2];
    uint16_t packVoltageMv = (static_cast<uint16_t>(labPayload[7]) << 8) | labPayload[8];
    int32_t packCurrentMa =
        (static_cast<int32_t>(labPayload[9]) << 24) |
        (static_cast<int32_t>(labPayload[10]) << 16) |
        (static_cast<int32_t>(labPayload[11]) << 8) |
        static_cast<int32_t>(labPayload[12]);
    uint16_t fwVersion = (static_cast<uint16_t>(labPayload[35]) << 8) | labPayload[36];

    html += "<div class='card' style='margin-top:14px'>"
            "<h2>FC04 Block 2 - Decoded Fields</h2>"
            "<div class='decode'>";

    html += "<div class='decodebox'><small>Cell count</small><strong>" +
            String(labPayload[0]) + "</strong></div>";

    html += "<div class='decodebox'><small>Cycle count</small><strong>" +
            String(cycles) + "</strong></div>";

    html += "<div class='decodebox'><small>Pack voltage</small><strong>" +
            String(packVoltageMv / 1000.0, 2) + " V</strong></div>";

    html += "<div class='decodebox'><small>Pack current</small><strong>" +
            String(packCurrentMa / 1000.0, 2) + " A</strong></div>";

    html += "<div class='decodebox'><small>BMS firmware</small><strong>" +
            String(fwVersion) + "</strong></div>";

    html += "</div></div>";
  }

  // ==========================================================
  // RS485 ACTIVITY
  // ==========================================================

  html +=
      "<div class='card hero'>"
      "<h2>RS485 Live Activity</h2>";

  if (!haveTraffic)
  {
    html +=
        "<div class='activity never'>"
        "NO RS485 TRAFFIC SEEN"
        "</div>";
  }
  else if (rs485Active)
  {
    html +=
        "<div class='activity active'>"
        "● RS485 ACTIVE"
        "</div>";
  }
  else
  {
    html +=
        "<div class='activity idle'>"
        "● RS485 IDLE"
        "</div>";
  }

  html +=
      "<div class='row'><span>RX bytes</span><span class='value'>" +
      String(rxByteCount) +
      "</span></div>";

  html +=
      "<div class='row'><span>Frames</span><span class='value'>" +
      String(rxFrameCount) +
      "</span></div>";

  if (lastFrameMillis > 0)
  {
    html +=
        "<div class='row'><span>Last frame age</span><span class='value'>" +
        String(lastFrameAge, 2) +
        " s</span></div>";
  }
  else
  {
    html +=
        "<div class='row'><span>Last frame age</span><span class='bad'>NEVER</span></div>";
  }

  html += "</div>";


  // ==========================================================
  // LAST RX - BIG AND OBVIOUS
  // ==========================================================

  html +=
      "<div class='card hero' style='margin-top:14px'>"
      "<h2>LAST RX</h2>";

  html +=
      "<div class='hex'>";

  if (lastRxFrameLen > 0)
  {
    html +=
        frameToHex(
            lastRxFrame,
            lastRxFrameLen);
  }
  else
  {
    html += "No frame captured yet";
  }

  html +=
      "</div>";

  html +=
      "<div class='decode'>";

  html +=
      "<div class='decodebox'>"
      "<small>Length</small><strong>" +
      String(lastRxFrameLen) +
      " bytes</strong></div>";

  html +=
      "<div class='decodebox'>"
      "<small>Slave</small><strong>0x";

  char temp[16];

  snprintf(
      temp,
      sizeof(temp),
      "%02X",
      displayDevice);

  html += temp;

  html +=
      "</strong></div>";

  html +=
      "<div class='decodebox'>"
      "<small>Function</small><strong>0x";

  snprintf(
      temp,
      sizeof(temp),
      "%02X",
      displayFunction);

  html += temp;

  html +=
      "</strong></div>";

  html +=
      "<div class='decodebox'>"
      "<small>Address</small><strong>0x";

  snprintf(
      temp,
      sizeof(temp),
      "%04X",
      displayRegister);

  html += temp;

  html +=
      "</strong></div>";

  html +=
      "<div class='decodebox'>"
      "<small>Value / Count</small><strong>" +
      String(displayValueCount) +
      "</strong></div>";

  html +=
      "</div>";

  html +=
      "<div style='margin-top:12px'>";

  if (lastRxFrameLen == 0)
  {
    html +=
        "<span class='warn'>Waiting for first frame</span>";
  }
  else
  {
    html +=
        "Length: ";

    if (lastLengthOK)
      html += "<span class='value'>8 bytes OK</span>";
    else
      html += "<span class='bad'>" +
              String(lastRxFrameLen) +
              " bytes UNEXPECTED</span>";

    html +=
        "    CRC: ";

    if (lastCrcOK)
      html += "<span class='value'>GOOD</span>";
    else
      html += "<span class='bad'>BAD / INCOMPLETE</span>";
  }

  html +=
      "</div>"
      "</div>";

  // ==========================================================
  // RX HISTORY
  // ==========================================================

  html +=
      "<div class='card hero' style='margin-top:14px'>"
      "<h2>RX History - Last 16 Chunks</h2>";

  if (rxHistoryUsed == 0)
  {
    html +=
        "<div class='warn'>No RX history captured yet</div>";
  }
  else
  {
    for (int n = 0; n < rxHistoryUsed; n++)
    {
      int index =
          rxHistoryHead - 1 - n;

      if (index < 0)
        index += RX_HISTORY_COUNT;

      RxHistoryEntry &entry =
          rxHistory[index];

      uint32_t ageMs =
          millis() - entry.capturedMillis;

      bool historyCrcOK = false;

      if (entry.len >= 4)
      {
        historyCrcOK =
            crcOK(entry.data, entry.len);
      }

      html +=
          "<div style='"
          "background:#030712;"
          "padding:10px;"
          "margin:7px 0;"
          "border-radius:7px;"
          "font-family:Consolas,\"Courier New\",monospace;'>";

      html +=
          "<span style='color:#9ca3af'>";

      html +=
          String(ageMs);

      html +=
          " ms ago   |   ";

      html +=
          String(entry.len);

      html +=
          " bytes   |   ";

      if (entry.len == 8)
      {
        html +=
            "<span class='value'>8 BYTE FRAME</span>";
      }
      else
      {
        html +=
            "<span class='warn'>CHUNK</span>";
      }

      html +=
          "   |   CRC ";

      if (historyCrcOK)
      {
        html +=
            "<span class='value'>GOOD</span>";
      }
      else
      {
        html +=
            "<span class='bad'>BAD/INCOMPLETE</span>";
      }

      html +=
          "</span><br>";

      html +=
          "<span style='"
          "color:#fde68a;"
          "font-size:18px;'>";

      html +=
          frameToHex(
              entry.data,
              entry.len);

      html +=
          "</span></div>";
    }
  }

  html += "</div>";
  // ==========================================================
  // THREE MAIN STATUS CARDS
  // ==========================================================

  html +=
      "<div class='topgrid' style='margin-top:14px'>";


  // SYSTEM

  html +=
      "<div class='card'>"
      "<h2>System</h2>";

  html +=
      "<div class='row'><span>Uptime</span><span class='value'>" +
      uptimeString() +
      "</span></div>";

  html +=
      "<div class='row'><span>IP</span><span class='value'>" +
      WiFi.localIP().toString() +
      "</span></div>";

  html +=
      "<div class='row'><span>WiFi RSSI</span><span class='value'>" +
      String(WiFi.RSSI()) +
      " dBm</span></div>";

  html +=
      "<div class='row'><span>RS485</span><span class='value'>9600 8N1</span></div>";

  html +=
      "<div class='row'><span>Firmware</span><span class='value'>V1.2R FC04 MAPPING LAB + robustness</span></div>";

  html +=
      "<div class='row'><span>Boot count</span><span class='value'>" +
      String(bootCount) +
      "</span></div>";

  html +=
      "<div class='row'><span>Last restart</span><span class='" +
      String(isAbnormal(resetReason, previousCause) ? "bad" : "value") + "'>" +
      restartText(resetReason, previousCause) +
      "</span></div>";

  html +=
      "<div class='row'><span>Last abnormal restart</span><span class='" +
      String(lastAbnormalBoot ? "warn" : "value") + "'>" +
      lastAbnormalText() +
      "</span></div>";

  html +=
      "<div class='row'><span>Free heap / min / largest</span><span class='value'>" +
      String(ESP.getFreeHeap()) + " / " +
      String(ESP.getMinFreeHeap()) + " / " +
      String(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)) +
      "</span></div>";

  html +=
      "<div class='row'><span>WiFi drops (last reason)</span><span class='value'>" +
      String(wifiDropCount) + " (" + String(lastWifiDisconnectReason) + ")" +
      "</span></div>";

  html += "</div>";


  // TRAFFIC

  html +=
      "<div class='card'>"
      "<h2>RS485 Counters</h2>";

  html +=
      "<div class='row'><span>Good frames</span><span class='value'>" +
      String(goodFrameCount) +
      "</span></div>";

  html +=
      "<div class='row'><span>Bad CRC</span><span class='value'>" +
      String(badCrcCount) +
      "</span></div>";

  html +=
      "<div class='row'><span>Replies</span><span class='value'>" +
      String(replyCount) +
      "</span></div>";

  html +=
      "<div class='row'><span>Unsupported</span><span class='value'>" +
      String(unsupportedCount) +
      "</span></div>";

  html += "</div>";


  // FUNCTION CODES

  html +=
      "<div class='card'>"
      "<h2>Function Codes</h2>";

  html +=
      "<div class='row'><span>FC03</span><span class='value'>" +
      String(fc03Count) +
      "</span></div>";

  html +=
      "<div class='row'><span>FC04</span><span class='value'>" +
      String(fc04Count) +
      "</span></div>";

  html +=
      "<div class='row'><span>FC06</span><span class='value'>" +
      String(fc06Count) +
      "</span></div>";

  html += "</div>";

  html += "</div>";


  // ==========================================================
  // LAST TX
  // ==========================================================

  html +=
      "<div class='card' style='margin-top:14px'>"
      "<h2>Last TX</h2>"
      "<div class='txhex'>";

  if (lastTxFrameLen > 0)
  {
    html +=
        frameToHex(
            lastTxFrame,
            lastTxFrameLen);
  }
  else
  {
    html += "No reply transmitted yet";
  }

  html +=
      "</div>"
      "</div>";


  // ==========================================================
  // EMULATED BATTERY
  // ==========================================================

  html +=
      "<div class='grid' style='margin-top:14px'>";

  html +=
      "<div class='card'>"
      "<h2>Emulated Battery</h2>";

  html +=
      "<div class='row'><span>SOC</span><span class='value'>" +
      String(holdingRegs[21]) +
      " %</span></div>";

  html +=
      "<div class='row'><span>Pack voltage</span><span class='value'>" +
      String(
          holdingRegs[22] / 100.0,
          2) +
      " V</span></div>";

  html +=
      "<div class='row'><span>Temperature</span><span class='value'>" +
      String(holdingRegs[24]) +
      " C</span></div>";

  html +=
      "<div class='row'><span>HR25 raw</span><span class='value' id='hr25Applied'>" +
      String(holdingRegs[25]) +
      "</span></div>";
  html +=
    "<div style='margin-top:14px;"
    "padding:12px;"
    "background:#111827;"
    "border-radius:8px;'>"

    "<strong style='color:#fde68a'>"
    "V1.2 FC04 MAPPING LAB - HR25"
    "</strong>"

    "<form action='/sethr25' method='get' "
    "style='margin-top:10px'>"

    "<input type='number' "
    "name='value' "
    "min='0' "
    "max='65535' "
    "value='" +
    String(holdingRegs[25]) +
    "' "

    "style='width:120px;"
    "padding:7px;"
    "font-size:16px'>"

    "<button type='submit' "
    "style='margin-left:8px;"
    "padding:7px 12px;"
    "font-size:16px'>"
    "SET HR25"
    "</button>"

    "</form>"

    "<div style='margin-top:8px;"
    "color:#fca5a5'>"
    "Keep inverter charge/discharge disabled while mapping. Use conservative limits for functional tests."
    "</div>"

    "</div>";
  html +=
      "<div class='row'><span>HR26 charge raw</span><span class='value'>" +
      String(holdingRegs[26]) +
      "</span></div>";

  html +=
      "<div class='row'><span>HR27 discharge raw</span><span class='value'>" +
      String(holdingRegs[27]) +
      "</span></div>";

  html += "</div>";


  // ==========================================================
  // CELL SUMMARY
  // ==========================================================

  html +=
      "<div class='card'>"
      "<h2>Cell Summary</h2>";

  html +=
      "<div class='row'><span>Cell total</span><span class='value'>" +
      String(cellTotal / 1000.0, 3) +
      " V</span></div>";

  html +=
      "<div class='row'><span>Minimum</span><span class='value'>" +
      String(minCell / 1000.0, 3) +
      " V</span></div>";

  html +=
      "<div class='row'><span>Maximum</span><span class='value'>" +
      String(maxCell / 1000.0, 3) +
      " V</span></div>";

  html +=
      "<div class='row'><span>Delta</span><span class='value'>" +
      String(delta) +
      " mV</span></div>";

  html += "</div>";

  html += "</div>";


  // ==========================================================
  // ALL CELLS
  // ==========================================================

  html +=
      "<div class='card' style='margin-top:14px'>"
      "<h2>16 Cell Test Data</h2>"
      "<div class='cells'>";

  for (int i = 0; i < CELL_COUNT; i++)
  {
    html +=
        "<div class='cell'>Cell ";

    html += String(i + 1);

    html +=
        "<br><span class='value'>";

    html +=
        String(
            cellMv[i] / 1000.0,
            3);

    html +=
        " V</span></div>";
  }

  html +=
      "</div>"
      "</div>";


  // ==========================================================
  // FOOTER
  // ==========================================================

  html +=
      "<div class='footer'>"
      "V1.2 FC04 MAPPING LAB • Live panel updates every 2 seconds; other diagnostics are page-load snapshots • "
      "RS485 protocol engine unchanged"
      "</div>";

  html += R"LAB(<script>
let writePending = false;
let statusGeneration = 0;
function hex(v, width) { return Number(v).toString(16).toUpperCase().padStart(width, '0'); }
function showLab(d) {
  document.getElementById('hr25Applied').textContent = d.hr25;
  document.getElementById('calibratedApplied').textContent = (d.calibratedRaw / 100).toFixed(2);
  document.getElementById('designApplied').textContent = (d.designRaw / 100).toFixed(2);
  document.getElementById('remainingApplied').textContent = (d.remainingRaw / 100).toFixed(2);
  document.getElementById('socApplied').textContent = d.soc;
  document.getElementById('status26Applied').textContent = d.status26 + ' (0x' + hex(d.status26, 2) + ')';
  document.getElementById('status27Applied').textContent = d.status27 + ' (0x' + hex(d.status27, 4) + ')';
  document.getElementById('status29Applied').textContent = d.status29 + ' (0x' + hex(d.status29, 4) + ')';
  document.getElementById('status31Applied').textContent = d.status31 + ' (0x' + hex(d.status31, 4) + ')';
  document.getElementById('status33Applied').textContent = d.status33 + ' (0x' + hex(d.status33, 4) + ')';
  document.getElementById('block2PayloadHex').textContent = d.payload;
  document.getElementById('labLive').textContent = 'HR25: '+d.hr25+' | Good frames: '+d.good+
    ' | Bad CRC: '+d.badCRC+' | Replies: '+d.replies+'\nBlock 2: '+d.payload;
}
function syncFormInput(form, d) {
  const field = form.querySelector('[name=field]');
  const offset = form.querySelector('[name=offset]');
  const input = form.querySelector('[name=ah],[name=value]');
  if (!input) return;
  if (field) {
    const key = field.value === 'calibrated' ? 'calibratedRaw' : field.value === 'design' ? 'designRaw' : 'remainingRaw';
    input.value = (d[key] / 100).toFixed(2);
  } else if (offset) {
    input.value = d['status' + offset.value];
  } else if (form.action.endsWith('/setsoc')) {
    input.value = d.soc;
  } else if (form.action.endsWith('/sethr25')) {
    input.value = d.hr25;
  }
}
async function pollLab() {
  if (writePending) return;
  const generation = statusGeneration;
  try {
    const r = await fetch('/labstatus', {cache:'no-store'});
    if (!r.ok) throw Error('Status unavailable');
    const d = await r.json();
    if (!writePending && generation === statusGeneration) showLab(d);
  } catch(e) { document.getElementById('labMessage').textContent = 'Live status unavailable; values may be stale.'; }
}
document.querySelectorAll('form').forEach(form => form.addEventListener('submit', async e => {
  e.preventDefault();
  if (writePending) return;
  writePending = true;
  ++statusGeneration;
  const buttons = document.querySelectorAll('button[type=submit]');
  buttons.forEach(b => b.disabled = true);
  const params = new URLSearchParams(new FormData(form));
  params.set('ajax','1');
  const input = form.querySelector('[name=ah],[name=value]');
  const sentValue = input ? input.value : null;
  try {
    const r = await fetch(form.action+'?'+params.toString(), {cache:'no-store'});
    if (!r.ok) throw Error(await r.text());
    const d = await r.json();
    showLab(d);
    if (input && input.value === sentValue) syncFormInput(form, d);
    document.getElementById('labMessage').textContent = 'Applied successfully.';
  } catch(e) { document.getElementById('labMessage').textContent = 'SET not confirmed: '+e.message+' Check applied values before retrying.'; }
  finally { writePending = false; buttons.forEach(b => b.disabled = false); }
}));
pollLab();
setInterval(pollLab, 2000);
</script>)LAB";

  html +=
      "</div>"
      "</body>"
      "</html>";


  server.send(
      200,
      "text/html",
      html);
}


// ============================================================
// WIFI / OTA / WEB SETUP
// ============================================================

void setupWiFiOTA()
{
  Serial.println();
  Serial.println(
      "Starting WiFi for OTA + web diagnostics...");

  WiFi.mode(
      WIFI_STA);

  WiFi.setSleep(
      false);

  WiFi.setAutoReconnect(
      true);

  IPAddress local_IP(
      192, 168, 1, 83);

  IPAddress gateway(
      192, 168, 1, 1);

  IPAddress subnet(
      255, 255, 255, 0);

  IPAddress dns1(
      192, 168, 1, 1);

  WiFi.config(
      local_IP,
      gateway,
      subnet,
      dns1);

  WiFi.setHostname(
      OTA_HOSTNAME);

  WiFi.begin(
      WIFI_SSID,
      WIFI_PASSWORD);
}

void handleSetHR25()
{
  if (!server.hasArg("value"))
  {
    server.send(
        400,
        "text/plain",
        "Missing value");

    return;
  }

  uint16_t requested;
  if (!parseRawDecimal(server.arg("value"), 65535, requested)) {
    server.send(400, "text/plain", "Value must be decimal 0..65535; no change applied");
    return;
  }

  experimentHR25 =
      static_cast<uint16_t>(requested);

  holdingRegs[25] =
      experimentHR25;

  Serial.println();
  Serial.println(
      "==============================================");

  Serial.printf(
      " V1.2 FC04 MAPPING LAB: BMS HR25 -> %u (0x%04X)\n",
      experimentHR25,
      experimentHR25);

  Serial.println(
      "==============================================");

  finishLabWrite();

}

void startNetworkServices()
{
  if (otaStarted)
    return;

  Serial.println();
  Serial.println(
      "==============================================");

  Serial.println(
      " WIFI CONNECTED");

  Serial.println(
      "==============================================");

  Serial.print(
      "IP address : ");

  Serial.println(
      WiFi.localIP());

  Serial.print(
      "Web page   : http://");

  Serial.print(
      WiFi.localIP());

  Serial.println(
      "/");

  // V1.3: register OTA and the web routes once. V1.2 re-ran all of this on
  // every WiFi reconnect, stacking duplicate route handlers each time.
  if (!servicesRegistered)
  {

  ArduinoOTA.setHostname(
      OTA_HOSTNAME);

  ArduinoOTA.onStart([]()
  {
    esp_task_wdt_reset();

    Serial.println();
    Serial.println(
        "OTA UPDATE STARTING");
  });

  ArduinoOTA.onEnd([]()
  {
    Serial.println();
    Serial.println(
        "OTA UPDATE COMPLETE - REBOOTING");
  });

  ArduinoOTA.onProgress([](
      unsigned int progress,
      unsigned int total)
  {
    // An OTA upload blocks loop() for longer than the watchdog timeout.
    esp_task_wdt_reset();

    static unsigned int lastPercent = 999;

    unsigned int percent =
        (progress * 100U) / total;

    if (percent != lastPercent)
    {
      lastPercent = percent;

      Serial.printf(
          "OTA progress: %u%%\r",
          percent);
    }
  });

  ArduinoOTA.onError([](
      ota_error_t error)
  {
    Serial.printf(
        "\nOTA ERROR [%u]\n",
        error);
  });

  ArduinoOTA.begin();

  server.on(
      "/",
      HTTP_GET,
      handleWebRoot);
  server.on(
    "/sethr25",
    HTTP_GET,
    handleSetHR25);

  server.on("/setcapacity", HTTP_GET, handleSetCapacity);
  server.on("/setsoc", HTTP_GET, handleSetSoc);
  server.on("/syncsoc", HTTP_GET, handleSyncSoc);
  server.on("/setstatus", HTTP_GET, handleSetStatus);
  server.on("/labstatus", HTTP_GET, handleLabStatus);
  server.on("/health", HTTP_GET, handleHealth);

  servicesRegistered = true;
  }

  // Harmless if already listening.
  server.begin();

  otaStarted = true;

  Serial.println(
      "OTA READY");

  Serial.println(
      "WEB SERVER READY");
}


// V1.3: auto-reconnect alone has left the board off the network all night.
// Restart the WiFi connection after a minute down, and restart the board if
// it still hasn't come back after ten. RS485 keeps running throughout; a
// restart costs ~3 s of replies, well inside the inverter's ~30 s tolerance.
void superviseWiFi()
{
  const uint32_t now = millis();

  if (wifiDownSince == 0)
    wifiDownSince = now ? now : 1;

  if (now - wifiDownSince >= WIFI_REBOOT_MS &&
      rxLen == 0)
  {
    Serial.println(
        "WiFi down for 10 min - restarting board");

    rtcRebootCause = CAUSE_WIFI_LOST;

    delay(100);

    ESP.restart();
  }

  if (now - lastWifiRetry >= WIFI_RETRY_MS)
  {
    lastWifiRetry = now;

    Serial.printf(
        "WiFi still down (reason %u) - restarting connection\n",
        lastWifiDisconnectReason);

    WiFi.disconnect();

    setupWiFiOTA();
  }
}


void serviceNetwork()
{
  bool connected =
      (WiFi.status() == WL_CONNECTED);

  if (connected &&
      !wifiWasConnected)
  {
    wifiWasConnected = true;
    wifiDownSince = 0;

    startNetworkServices();
  }

  if (!connected &&
      wifiWasConnected)
  {
    wifiWasConnected = false;
    otaStarted = false;

    wifiDropCount++;
    wifiDownSince = millis();
    lastWifiRetry = wifiDownSince;

    Serial.printf(
        "WiFi disconnected (reason %u).\n",
        lastWifiDisconnectReason);
  }

  if (!connected)
    superviseWiFi();

  // IMPORTANT:
  // network gets serviced only when we're not
  // actively receiving an RS485 frame.

  if (connected &&
      otaStarted &&
      rxLen == 0)
  {
    ArduinoOTA.handle();
    server.handleClient();
  }
}


// ============================================================
// FRAME PROCESSOR
// ============================================================

void processFrame()
{
  rxFrameCount++;

// Record EVERY received chunk before analysing it.
// This lets us see exactly what the inverter is putting
// onto the RS485 line, including short/invalid chunks.
  addRxHistory(rxBuf, rxLen);


  // Preserve frame for web page

  lastRxFrameLen =
      min(rxLen, sizeof(lastRxFrame));

  memcpy(
      lastRxFrame,
      rxBuf,
      lastRxFrameLen);

  lastFrameMillis =
      millis();

  printCapturedBytes();

  if (rxLen != 8)
  {
    Serial.printf(
        "UNEXPECTED LENGTH: %u bytes\n",
        static_cast<unsigned>(rxLen));

    unsupportedCount++;

    return;
  }

  if (!crcOK(
          rxBuf,
          rxLen))
  {
    Serial.println(
        "*** BAD MODBUS CRC ***");

    badCrcCount++;

    return;
  }

  goodFrameCount++;

  uint8_t device =
      rxBuf[0];

  uint8_t fc =
      rxBuf[1];

  uint16_t reg =
      (static_cast<uint16_t>(
          rxBuf[2]) << 8) |
      rxBuf[3];

  uint16_t valueOrCount =
      (static_cast<uint16_t>(
          rxBuf[4]) << 8) |
      rxBuf[5];

  lastDevice =
      device;

  lastFunction =
      fc;

  lastRegister =
      reg;

  lastValueOrCount =
      valueOrCount;

  Serial.printf(
      "DECODED: slave=0x%02X FC=0x%02X addr=0x%04X value/count=%u\n",
      device,
      fc,
      reg,
      valueOrCount);

  switch (fc)
  {
    case 0x03:

      handleFC03(
          device,
          reg,
          valueOrCount);

      break;


    case 0x04:

      handleFC04(
          device,
          reg,
          valueOrCount);

      break;


    case 0x06:

      handleFC06(
          rxBuf,
          rxLen);

      break;


    default:

      Serial.printf(
          "UNSUPPORTED FUNCTION 0x%02X\n",
          fc);

      unsupportedCount++;

      if (device >= 1 &&
          device <= 5)
      {
        sendException(
            device,
            fc,
            0x01);
      }

      break;
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(
      USB_BAUD);

  delay(
      1500);

  Serial.println();
  Serial.println(
      "================================================");

  Serial.println(
      " GivEnergy Gen1 V1.2R FC04 MAPPING LAB + robustness");

  Serial.println(
      "================================================");

  recordBoot();

  Serial.printf(
      "Boot #%lu  last restart: %s  last abnormal: %s\n",
      static_cast<unsigned long>(bootCount),
      restartText(resetReason, previousCause).c_str(),
      lastAbnormalText().c_str());

  pinMode(
      PIN_16,
      OUTPUT);

  pinMode(
      PIN_17,
      OUTPUT);

  pinMode(
      PIN_19,
      OUTPUT);

  // KNOWN GOOD HARDWARE STATE - DO NOT CHANGE

  digitalWrite(
      PIN_16,
      HIGH);

  digitalWrite(
      PIN_17,
      HIGH);

  digitalWrite(
      PIN_19,
      HIGH);

  initialiseHoldingRegisters();

  Serial1.begin(
      RS485_BAUD,
      SERIAL_8N1,
      RS485_RX_PIN,
      RS485_TX_PIN);

  Serial1.setRxFIFOFull(
      1);

  Serial1.setRxTimeout(
      1);

  while (Serial1.available())
    Serial1.read();

  // RS485 is fully configured BEFORE WiFi starts.

  initCan();

  // Register once, before the first connect attempt. Registering this
  // inside setupWiFiOTA() instead would stack a duplicate handler on
  // every WiFi retry that function does later (see superviseWiFi()).
  WiFi.onEvent(
      onWiFiEvent);

  setupWiFiOTA();

  lastHeartbeat =
      millis();

  setupWatchdog();

  Serial.println(
      "LISTENING...");

  Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  esp_task_wdt_reset();

  // ----------------------------------------------------------
  // RS485 ALWAYS GETS FIRST PRIORITY
  // ----------------------------------------------------------

  while (Serial1.available())
  {
    uint8_t b =
        Serial1.read();

    uint32_t nowUs =
        micros();

    if (rxLen < RX_MAX)
    {
      rxBuf[rxLen] =
          b;

      rxTimes[rxLen] =
          nowUs;

      rxLen++;
    }

    rxByteCount++;

    lastRxUs =
        nowUs;
  }


  // Silence >= 4ms = end of Modbus frame

  if (rxLen > 0)
  {
    uint32_t nowUs =
        micros();

    if ((uint32_t)(
            nowUs -
            lastRxUs)
        >= FRAME_GAP_US)
    {
      processFrame();

      rxLen = 0;
    }
  }


  // ----------------------------------------------------------
  // CAN (DATOUBOSS) - AFTER RS485, BEFORE NETWORK
  // ----------------------------------------------------------

  serviceCan();


  // ----------------------------------------------------------
  // NETWORK ONLY AFTER RS485
  // ----------------------------------------------------------

  serviceNetwork();


  // ----------------------------------------------------------
  // HEARTBEAT
  // ----------------------------------------------------------

  if (millis() -
          lastHeartbeat
      >= 5000)
  {
    lastHeartbeat =
        millis();

    Serial.printf(
        "STATS bytes=%lu frames=%lu good=%lu badcrc=%lu replies=%lu "
        "FC03=%lu FC04=%lu FC06=%lu",
        static_cast<unsigned long>(rxByteCount),
        static_cast<unsigned long>(rxFrameCount),
        static_cast<unsigned long>(goodFrameCount),
        static_cast<unsigned long>(badCrcCount),
        static_cast<unsigned long>(replyCount),
        static_cast<unsigned long>(fc03Count),
        static_cast<unsigned long>(fc04Count),
        static_cast<unsigned long>(fc06Count));

    Serial.printf(
        " heap=%lu min=%lu block=%lu wifidrops=%lu",
        static_cast<unsigned long>(ESP.getFreeHeap()),
        static_cast<unsigned long>(ESP.getMinFreeHeap()),
        static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
        static_cast<unsigned long>(wifiDropCount));

    if (WiFi.status() ==
        WL_CONNECTED)
    {
      Serial.print(
          " wifi=");

      Serial.print(
          WiFi.localIP());

      Serial.print(
          " WEB=READY OTA=READY");
    }
    else
    {
      Serial.print(
          " wifi=CONNECTING");
    }

    if (!canDriverOk)
    {
      Serial.print(" CAN=DRIVER_FAIL");
    }
    else if (!canLinkEverEstablished)
    {
      Serial.print(" CAN=waiting-for-battery");
    }
    else if (canFailsafeActive)
    {
      Serial.print(" CAN=FAILSAFE(link quiet)");
    }
    else
    {
      Serial.printf(
          " CAN=live soc=%u%% v=%.2f i=%.1f t=%.1f frames=%lu",
          canSoc_pct,
          canPackVoltage_0p01V / 100.0,
          canPackCurrent_0p1A / 10.0,
          canTemp_0p1C / 10.0,
          static_cast<unsigned long>(canFramesSeen));
    }

    Serial.println();
  }
}
