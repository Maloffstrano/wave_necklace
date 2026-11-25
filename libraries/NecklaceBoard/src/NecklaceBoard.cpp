// Shared by every board: the QMI8658 motion sensor and its task, the fast
// frame path (DMA strips on the display's QSPI bus) and the buttons.
// Board-specific code is in board_<name>.cpp.
#include "board_internal.h"
#include <Wire.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"

namespace NecklaceBoard {

// ---- QMI8658 IMU (on Wire, SA0 low) ----
static constexpr uint8_t QMI_ADDR = 0x6B;
static constexpr uint8_t QMI_WHO_AM_I = 0x00;  // reads 0x05
static constexpr uint8_t QMI_CTRL1 = 0x02;     // interface: bit6 = register auto-increment
static constexpr uint8_t QMI_CTRL2 = 0x03;     // accel: scale bits 6:4, ODR bits 3:0
static constexpr uint8_t QMI_CTRL3 = 0x04;     // gyro:  scale bits 6:4, ODR bits 3:0
static constexpr uint8_t QMI_CTRL5 = 0x06;     // low-pass filters
static constexpr uint8_t QMI_CTRL7 = 0x08;     // sensor enable
static constexpr uint8_t QMI_AX_L = 0x35;      // accel X/Y/Z then gyro X/Y/Z, 12 bytes little-endian
static constexpr uint8_t QMI_ODR_250HZ = 0x05;
static constexpr uint8_t QMI_ACC_8G = 2 << 4;
static constexpr uint8_t QMI_GYR_512DPS = 5 << 4;
static constexpr float QMI_ACC_SCALE = 8.0f / 32768.0f;
static constexpr float QMI_GYR_SCALE = 512.0f / 32768.0f;
static bool imuOk = false;

// ---- Fast frame output ----
// A second SPI device on the bus Arduino_GFX already set up, with its own
// DMA transaction queue. CS is driven by hand, as Arduino_ESP32QSPI does.
static constexpr size_t STRIP_PIXELS = (size_t)WIDTH * STRIP_ROWS;
static spi_device_handle_t fastDev = nullptr;
static uint16_t *strips[2] = {nullptr, nullptr};
static spi_transaction_ext_t fastTrans[2];

namespace detail {

void serialNoWait() {
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  // Serial is the native USB port. By default a print blocks (up to a
  // timeout) when the USB buffer is full because nothing on the computer is
  // reading, which stalls the whole loop. Never wait: drop output instead.
  Serial.setTxTimeoutMs(0);
#endif
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

}  // namespace detail

static void qmiWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMI_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static bool qmiRead(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(QMI_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(QMI_ADDR, (uint8_t)len) != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void detail::imuInit() {
  uint8_t id = 0;
  if (!qmiRead(QMI_WHO_AM_I, &id, 1) || id != 0x05) {
    Serial.printf("NecklaceBoard: QMI8658 not found (WHO_AM_I = 0x%02X)\n", id);
    return;
  }
  qmiWrite(0x60, 0xB0);                                 // soft reset (clears any state left by earlier firmware)
  delay(20);
  qmiWrite(QMI_CTRL1, 0x40);                            // auto-increment for block reads
  qmiWrite(QMI_CTRL2, QMI_ACC_8G | QMI_ODR_250HZ);      // +-8 g: room for jolts and splashes
  qmiWrite(QMI_CTRL3, QMI_GYR_512DPS | QMI_ODR_250HZ);  // +-512 dps: a swinging pendant turns fast
  // Gyro low-pass filter on; accelerometer filter OFF. Its narrowest mode cuts
  // off at about 2.7% of the data rate (~7 Hz at 250 Hz), which flattens
  // shocks like a bang on the table to almost nothing. Smoothing for tilt is
  // done in software instead.
  qmiWrite(QMI_CTRL5, 0x10);
  qmiWrite(QMI_CTRL7, 0x03);                            // enable accel + gyro
  delay(30);                                            // gyro needs a moment to start
  imuOk = true;
  Serial.println("NecklaceBoard: QMI8658 ready");
}

bool detail::fastInit() {
  spi_device_interface_config_t dev = {};
  dev.command_bits = 8;
  dev.address_bits = 24;
  dev.mode = 0;
  dev.clock_source = SPI_CLK_SRC_DEFAULT;
  dev.clock_speed_hz = DISPLAY_SPI_HZ;
  dev.spics_io_num = -1;  // CS by hand: it must stay low across a whole frame
  dev.flags = SPI_DEVICE_HALFDUPLEX;
  dev.queue_size = 2;
  if (spi_bus_add_device(SPI2_HOST, &dev, &fastDev) != ESP_OK) {
    Serial.println("NecklaceBoard: fast path: spi_bus_add_device failed");
    fastDev = nullptr;
    return false;
  }
  for (auto &strip : strips) {
    strip = (uint16_t *)heap_caps_malloc(STRIP_PIXELS * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!strip) {
      Serial.println("NecklaceBoard: fast path: strip allocation failed");
      return false;
    }
  }
  return true;
}

static inline void csLow() { gpio_set_level((gpio_num_t)PIN_LCD_CS, 0); }
static inline void csHigh() { gpio_set_level((gpio_num_t)PIN_LCD_CS, 1); }

// One command frame, framed exactly like Arduino_ESP32QSPI (opcode 0x02).
static void fastCommand(uint8_t reg, const uint8_t *data = nullptr, size_t len = 0) {
  spi_transaction_t t = {};
  t.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
  if (len) {
    t.flags |= SPI_TRANS_USE_TXDATA;
    memcpy(t.tx_data, data, len);  // len <= 4
  }
  t.cmd = 0x02;
  t.addr = (uint32_t)reg << 8;
  t.length = len * 8;
  csLow();
  spi_device_polling_transmit(fastDev, &t);
  csHigh();
}

void drawRows(int16_t y0, int16_t y1, StripRenderer render, void *ctx) {
  if (!fastDev || !strips[1] || y0 >= y1) return;

  const int16_t c0 = COL_OFFSET, c1 = COL_OFFSET + WIDTH - 1;
  const int16_t r0 = ROW_OFFSET + y0, r1 = ROW_OFFSET + y1 - 1;
  const uint8_t caset[4] = {(uint8_t)(c0 >> 8), (uint8_t)c0, (uint8_t)(c1 >> 8), (uint8_t)c1};
  const uint8_t raset[4] = {(uint8_t)(r0 >> 8), (uint8_t)r0, (uint8_t)(r1 >> 8), (uint8_t)r1};
  fastCommand(0x2A, caset, 4);  // CASET: full width
  fastCommand(0x2B, raset, 4);  // RASET: rows y0 .. y1 - 1
  fastCommand(0x2C);            // RAMWR

  // One pixel frame: CS stays low; the first transaction carries the
  // "memory write continue" header, the rest are raw pixels.
  csLow();
  int inFlight = 0;
  int k = 0;
  for (int16_t y = y0; y < y1; y += STRIP_ROWS, k++) {
    int16_t rows = (y1 - y < STRIP_ROWS) ? (y1 - y) : STRIP_ROWS;
    uint16_t *buf = strips[k & 1];
    if (inFlight == 2) {  // this buffer's previous strip must be on the wire already
      spi_transaction_t *done;
      spi_device_get_trans_result(fastDev, &done, portMAX_DELAY);
      inFlight--;
    }
    render(buf, y, rows, ctx);

    spi_transaction_ext_t &t = fastTrans[k & 1];
    memset(&t, 0, sizeof(t));
    if (k == 0) {
      t.base.flags = SPI_TRANS_MODE_QIO;
      t.base.cmd = 0x32;
      t.base.addr = 0x003C00;
    } else {
      t.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR |
                     SPI_TRANS_VARIABLE_DUMMY;
    }
    t.base.tx_buffer = buf;
    t.base.length = (size_t)rows * WIDTH * 16;
    spi_device_queue_trans(fastDev, &t.base, portMAX_DELAY);
    inFlight++;
  }
  while (inFlight > 0) {
    spi_transaction_t *done;
    spi_device_get_trans_result(fastDev, &done, portMAX_DELAY);
    inFlight--;
  }
  csHigh();
}

static void fillStrip(uint16_t *pixels, int16_t, int16_t rows, void *ctx) {
  uint16_t c = *(uint16_t *)ctx;
  for (size_t i = 0, n = (size_t)rows * WIDTH; i < n; i++) pixels[i] = c;
}

void clearScreen(uint16_t rgb565) {
  uint16_t c = panelColor(rgb565);
  drawFrame(fillStrip, &c);
}

// ---- Motion ----

bool imuReady() { return imuOk; }

bool readMotion(Motion &m) {
  if (!imuOk) return false;
  uint8_t b[12];
  if (!qmiRead(QMI_AX_L, b, sizeof(b))) return false;
  auto s16 = [&](int i) { return (int16_t)(b[i] | (b[i + 1] << 8)); };
  m.ax = s16(0) * QMI_ACC_SCALE;
  m.ay = s16(2) * QMI_ACC_SCALE;
  m.az = s16(4) * QMI_ACC_SCALE;
  m.gx = s16(6) * QMI_GYR_SCALE;
  m.gy = s16(8) * QMI_GYR_SCALE;
  m.gz = s16(10) * QMI_GYR_SCALE;
  return true;
}

static portMUX_TYPE motionLock = portMUX_INITIALIZER_UNLOCKED;
static Motion motionLatest = {};
static float motionPeakJolt = 0;
static bool motionHave = false;

static void motionTask(void *) {
  const TickType_t period = pdMS_TO_TICKS(4);  // 250 Hz, the sensor's data rate
  TickType_t wake = xTaskGetTickCount();
  float avgX = 0, avgY = 0, avgZ = 0;
  bool first = true;
  for (;;) {
    Motion m;
    if (readMotion(m)) {
      if (first) {
        avgX = m.ax; avgY = m.ay; avgZ = m.az;
        first = false;
      }
      // Jolt: distance from a short running average (about 0.1 s).
      float dx = m.ax - avgX, dy = m.ay - avgY, dz = m.az - avgZ;
      float jolt = sqrtf(dx * dx + dy * dy + dz * dz);
      const float k = 0.04f;  // 4 ms / 100 ms
      avgX += dx * k; avgY += dy * k; avgZ += dz * k;

      portENTER_CRITICAL(&motionLock);
      motionLatest = m;
      if (jolt > motionPeakJolt) motionPeakJolt = jolt;
      motionHave = true;
      portEXIT_CRITICAL(&motionLock);
    }
    vTaskDelayUntil(&wake, period);
  }
}

bool startMotionTask() {
  if (!imuOk) return false;
  // Core 0: the Arduino loop runs on core 1, so this never steals render time.
  return xTaskCreatePinnedToCore(motionTask, "motion", 4096, nullptr, 2, nullptr, 0) == pdPASS;
}

bool latestMotion(Motion &m, float &peakJolt) {
  portENTER_CRITICAL(&motionLock);
  bool have = motionHave;
  m = motionLatest;
  peakJolt = motionPeakJolt;
  motionPeakJolt = 0;
  portEXIT_CRITICAL(&motionLock);
  return have;
}

// ---- Buttons ----

void Button::begin() {
  attachInterruptArg(digitalPinToInterrupt(pin_), onEdge, this, CHANGE);
}

void IRAM_ATTR Button::onEdge(void *arg) {
  // Contacts bounce on both press and release. Count a press only on a
  // falling edge after the line has been quiet for 50 ms, so bounces
  // (including the ones on release) never count as extra presses.
  Button *b = static_cast<Button *>(arg);
  uint32_t now = millis();
  bool quiet = now - b->lastEdgeMs_ >= 50;
  b->lastEdgeMs_ = now;
  if (quiet && digitalRead(b->pin_) == LOW) {
    b->pressedAtMs_ = now;
    b->pressCount_++;
  }
}

bool Button::pressed() {
  uint32_t count = pressCount_;
  if (count == seenCount_) return false;
  seenCount_ = count;
  return true;
}

bool Button::heldFor(uint32_t ms) {
  uint32_t count = pressCount_;
  if (digitalRead(pin_) != LOW || count == holdHandledCount_) return false;
  if (millis() - pressedAtMs_ < ms) return false;
  holdHandledCount_ = count;
  return true;
}

}  // namespace NecklaceBoard
