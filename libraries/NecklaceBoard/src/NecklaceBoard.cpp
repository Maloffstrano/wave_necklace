#include "NecklaceBoard.h"
#include <Wire.h>
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "st77916_waveshare_init.h"

namespace NecklaceBoard {

// ---- TCA9554 IO expander ----
static constexpr uint8_t TCA9554_ADDR = 0x20;
static constexpr uint8_t TCA9554_OUTPUT = 0x01;
static constexpr uint8_t TCA9554_CONFIG = 0x03;
static constexpr uint8_t EXIO_TP_RST = 1 << 0;   // EXIO1
static constexpr uint8_t EXIO_LCD_RST = 1 << 1;  // EXIO2

// ---- CST816 touch ----
static constexpr uint8_t CST816_ADDR = 0x15;
static constexpr uint8_t CST816_REG_FINGERS = 0x02;
static constexpr uint8_t CST816_REG_NOSLEEP = 0xFE;

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

static Arduino_DataBus *bus = nullptr;
static Arduino_GFX *gfx = nullptr;
static int rev = 0;

static void tcaWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// The LCD and touch reset lines live on the expander, not on GPIOs.
static void resetPanelAndTouch() {
  tcaWrite(TCA9554_CONFIG, (uint8_t)~(EXIO_TP_RST | EXIO_LCD_RST));  // both as outputs
  tcaWrite(TCA9554_OUTPUT, 0x00);                                     // assert reset
  delay(20);
  tcaWrite(TCA9554_OUTPUT, EXIO_TP_RST | EXIO_LCD_RST);              // release reset
  delay(120);
}

// The board ships with two panel revisions that need different init
// sequences. Like Waveshare's demo, read register 0x04 over QSPI to tell
// them apart. Returns 1 or 2 (1 if the read fails).
static int detectPanelRevision() {
  spi_bus_config_t buscfg = {};
  buscfg.data0_io_num = PIN_LCD_D0;
  buscfg.data1_io_num = PIN_LCD_D1;
  buscfg.sclk_io_num = PIN_LCD_SCK;
  buscfg.data2_io_num = PIN_LCD_D2;
  buscfg.data3_io_num = PIN_LCD_D3;
  buscfg.data4_io_num = buscfg.data5_io_num = buscfg.data6_io_num = buscfg.data7_io_num = -1;
  buscfg.max_transfer_sz = 2048;
  buscfg.flags = SPICOMMON_BUSFLAG_MASTER;
  if (spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) {
    Serial.println("NecklaceBoard: spi_bus_initialize failed");
    return 1;
  }

  esp_lcd_panel_io_spi_config_t iocfg = {};
  iocfg.cs_gpio_num = PIN_LCD_CS;
  iocfg.dc_gpio_num = -1;
  iocfg.spi_mode = 0;
  iocfg.pclk_hz = 5 * 1000 * 1000;
  iocfg.trans_queue_depth = 10;
  iocfg.lcd_cmd_bits = 32;
  iocfg.lcd_param_bits = 8;
  iocfg.flags.quad_mode = 1;

  int result = 1;
  esp_lcd_panel_io_handle_t io = nullptr;
  if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &iocfg, &io) == ESP_OK) {
    uint8_t data[4] = {0};
    int cmd = (0x0B << 24) | (0x04 << 8);  // QSPI read opcode + register 0x04
    if (esp_lcd_panel_io_rx_param(io, cmd, data, sizeof(data)) == ESP_OK) {
      Serial.printf("NecklaceBoard: reg 0x04 = %02X %02X %02X %02X\n", data[0], data[1], data[2], data[3]);
      if (data[0] == 0x00 && data[1] == 0x02 && data[2] == 0x7F && data[3] == 0x7F) result = 2;
    } else {
      Serial.println("NecklaceBoard: panel register read failed");
    }
    esp_lcd_panel_io_del(io);
  }
  spi_bus_free(SPI2_HOST);  // hand the bus over to Arduino_GFX
  return result;
}

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

static void imuInit() {
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
  qmiWrite(QMI_CTRL5, 0x11);                            // accel and gyro low-pass filters on
  qmiWrite(QMI_CTRL7, 0x03);                            // enable accel + gyro
  delay(30);                                            // gyro needs a moment to start
  imuOk = true;
  Serial.println("NecklaceBoard: QMI8658 ready");
}

static void touchInit() {
  Wire1.begin(PIN_TP_SDA, PIN_TP_SCL, 400000);
  Wire1.beginTransmission(CST816_ADDR);
  Wire1.write(CST816_REG_NOSLEEP);
  Wire1.write(0x01);  // non-zero disables auto-sleep, so polling always gets an answer
  Wire1.endTransmission();
}

Arduino_GFX *begin() {
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  // Serial is the native USB port. By default a print blocks (up to a
  // timeout) when the USB buffer is full because nothing on the computer is
  // reading, which stalls the whole loop. Never wait: drop output instead.
  Serial.setTxTimeoutMs(0);
#endif

  // Keep ourselves powered when running from the battery: the PWR button only
  // powers the board while held, unless this latch is driven HIGH.
  pinMode(PIN_PWR_LATCH, OUTPUT);
  digitalWrite(PIN_PWR_LATCH, HIGH);
  pinMode(PIN_BTN_PWR, INPUT);
  pinMode(PIN_BTN_BOOT, INPUT_PULLUP);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  resetPanelAndTouch();

  rev = detectPanelRevision();
  Serial.printf("NecklaceBoard: panel revision %d\n", rev);

  bus = new Arduino_ESP32QSPI(PIN_LCD_CS, PIN_LCD_SCK, PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3);
  gfx = new Arduino_ST77916(bus, GFX_NOT_DEFINED /* RST via expander */, 0 /* rotation */,
                            true /* IPS */, WIDTH, HEIGHT, 0, 0, 0, 0,
                            rev == 2 ? waveshare_185_init_rev2 : waveshare_185_init_rev1,
                            rev == 2 ? sizeof(waveshare_185_init_rev2) : sizeof(waveshare_185_init_rev1));
  if (!gfx->begin(DISPLAY_SPI_HZ)) {
    Serial.println("NecklaceBoard: gfx->begin() failed");
    return nullptr;
  }
  gfx->fillScreen(RGB565_BLACK);

  ledcAttach(PIN_LCD_BL, BACKLIGHT_FREQ, BACKLIGHT_BITS);
  setBacklight(BACKLIGHT_MAX);

  touchInit();
  imuInit();
  return gfx;
}

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

int panelRevision() { return rev; }

void setBacklight(uint16_t duty) {
  ledcWrite(PIN_LCD_BL, duty > BACKLIGHT_MAX ? BACKLIGHT_MAX : duty);
}

bool isTouched() {
  Wire1.beginTransmission(CST816_ADDR);
  Wire1.write(CST816_REG_FINGERS);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(CST816_ADDR, (uint8_t)1) != 1) return false;
  return Wire1.read() > 0;
}

void powerOff() {
  setBacklight(0);
  digitalWrite(PIN_PWR_LATCH, LOW);
}

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
