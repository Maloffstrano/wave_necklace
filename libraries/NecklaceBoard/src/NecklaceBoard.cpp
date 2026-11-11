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

static void touchInit() {
  Wire1.begin(PIN_TP_SDA, PIN_TP_SCL, 400000);
  Wire1.beginTransmission(CST816_ADDR);
  Wire1.write(CST816_REG_NOSLEEP);
  Wire1.write(0x01);  // non-zero disables auto-sleep, so polling always gets an answer
  Wire1.endTransmission();
}

Arduino_GFX *begin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  resetPanelAndTouch();

  rev = detectPanelRevision();
  Serial.printf("NecklaceBoard: panel revision %d\n", rev);

  bus = new Arduino_ESP32QSPI(PIN_LCD_CS, PIN_LCD_SCK, PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3);
  gfx = new Arduino_ST77916(bus, GFX_NOT_DEFINED /* RST via expander */, 0 /* rotation */,
                            true /* IPS */, WIDTH, HEIGHT, 0, 0, 0, 0,
                            rev == 2 ? waveshare_185_init_rev2 : waveshare_185_init_rev1,
                            rev == 2 ? sizeof(waveshare_185_init_rev2) : sizeof(waveshare_185_init_rev1));
  if (!gfx->begin()) {
    Serial.println("NecklaceBoard: gfx->begin() failed");
    return nullptr;
  }
  gfx->fillScreen(RGB565_BLACK);

  ledcAttach(PIN_LCD_BL, BACKLIGHT_FREQ, BACKLIGHT_BITS);
  setBacklight(BACKLIGHT_MAX);

  touchInit();
  return gfx;
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

}  // namespace NecklaceBoard
