

#include "cc1101.h"
#include "сс1101_api.h"

cc1101_fun_table fun_tab;

const uint8_t cc1101_cfg_from_smrf[] = {
    0x29, // IOCFG2        GDO2 Output Pin Configuration
    0x2E, // IOCFG1        GDO1 Output Pin Configuration
    0x06, // IOCFG0        GDO0 Output Pin Configuration
    0x47, // FIFOTHR       RX/TX FIFO Thresholds
    0xD3, // SYNC1         Sync Word High
    0x91, // SYNC0         Sync Word Low
    0xFF, // PKTLEN        Packet Length
    0x06, // PKTCTRL1      Packet Automation Control
    0x05, // PKTCTRL0      Packet Automation Control
    0x00, // ADDR          Device Address
    0x00, // CHANNR        Channel Number
    0x06, // FSCTRL1       Frequency Synthesizer Control
    0x00, // FSCTRL0       Frequency Synthesizer Control
    0x21, // FREQ2         Frequency Control Word High
    0x62, // FREQ1         Frequency Control Word Mid
    0x76, // FREQ0         Frequency Control Word Low
    0xF6, // MDMCFG4       Modem Configuration
    0x83, // MDMCFG3       Modem Configuration
    0x93, // MDMCFG2       Modem Configuration
    0x22, // MDMCFG1       Modem Configuration
    0xF8, // MDMCFG0       Modem Configuration
    0x15, // DEVIATN       Deviation Setting
    0x07, // MCSM2         Main State Machine Config
    0x30, // MCSM1         Main State Machine Config
    0x18, // MCSM0         Main State Machine Config
    0x16, // FOCCFG        Freq Offset Compensation
    0x6C, // BSCFG         Bit Sync Configuration
    0x03, // AGCCTRL2      AGC Control
    0x40, // AGCCTRL1      AGC Control
    0x91, // AGCCTRL0      AGC Control
    0x87, // WOREVT1       Event0 Timeout High
    0x6B, // WOREVT0       Event0 Timeout Low
    0xFB, // WORCTRL       Wake-On-Radio Control
    0x56, // FREND1        Front End RX Config
    0x10, // FREND0        Front End TX Config (PA_POWER bits here)
    0xE9, // FSCAL3        Synth Calibration
    0x2A, // FSCAL2        Synth Calibration
    0x00, // FSCAL1        Synth Calibration
    0x1F, // FSCAL0        Synth Calibration
    0x41, // RCCTRL1       RC Oscillator Config
    0x00, // RCCTRL0       RC Oscillator Config
    0x59, // FSTEST        Synth Test
    0x7F, // PTEST         Production Test
    0x3F, // AGCTEST       AGC Test
    0x81, // TEST2         Various Test Settings
    0x35, // TEST1         Various Test Settings
    0x09, // TEST0         Various Test Settings
};

int cc1101_driver_init(cc1101_fun_table *fun_table) {
  if (!fun_table || !fun_table->cs_set_level || !fun_table->spi_txrx ||
      !fun_table->spi_tx || !fun_table->delay_us) {
    return CC1101_EINVAL;
  }
  fun_tab = *fun_table;
  return CC1101_OK;
}

void cc1101_reset() {

  fun_tab.cs_set_level(fun_tab.user, GPIO_LOW_LEVEL);
  fun_tab.delay_us(fun_tab.user, 10);

  fun_tab.cs_set_level(fun_tab.user, GPIO_HIGH_LEVEL);
  fun_tab.delay_us(fun_tab.user, 40);

  cc1101_write_strobe(SRES);
  fun_tab.delay_us(fun_tab.user, 30);
}

bool cc1101_check() {

  bool status = true;
  uint8_t version;
  cc1101_read_register(VERSION, &version);
  if (version != 0x14)
    status = false;

  if (status) {
    printf("RF check: OK\n\r");
  } else {
    printf("RF check: No luck :(\n\r");
  }
  printf("VERSION=0x%02X\r\n", version);
  return status;
}

uint8_t cc1101_get_status_byte() {
  uint8_t status_byte;
  cc1101_write_strobe(SNOP);
}

void cc1101_flush_RX_buffer() { cc1101_write_strobe(SFRX); }

void cc1101_flush_TX_buffer() { cc1101_write_strobe(SFTX); }

uint8_t cc1101_get_state_machine_state() {
  uint8_t state;
  cc1101_read_register(MARCSTATE, &state);
  return state;
}

int __spi_write(uint8_t addr, const uint8_t *pData, uint16_t size,
                uint32_t timeout) {
  uint8_t status;
  fun_tab.cs_set_level(fun_tab.user, GPIO_LOW_LEVEL); // set Chip Select to Low
  if (fun_tab.spi_txrx(fun_tab.user, &addr, &status, 1, timeout) != 0) {
    fun_tab.cs_set_level(fun_tab.user, GPIO_HIGH_LEVEL);
    return CC1101_EIO;
  }
  if (fun_tab.spi_tx(fun_tab.user, pData, size, timeout) != 0) {
    fun_tab.cs_set_level(fun_tab.user, GPIO_HIGH_LEVEL);
    return CC1101_EIO;
  }

  fun_tab.cs_set_level(fun_tab.user,
                       GPIO_HIGH_LEVEL); // set Chip Select to High
  return CC1101_OK;
}

int __spi_read(uint8_t addr, uint8_t *pData, uint16_t size, uint32_t timeout) {

  uint8_t status;
  fun_tab.cs_set_level(fun_tab.user, GPIO_LOW_LEVEL); // set Chip Select to Low
  if (fun_tab.spi_txrx(fun_tab.user, &addr, &status, 1, timeout) != 0) {
    fun_tab.cs_set_level(fun_tab.user, GPIO_HIGH_LEVEL);
    return CC1101_EIO;
  }
  if (fun_tab.spi_rx(fun_tab.user, pData, size, timeout) != 0) {
    fun_tab.cs_set_level(fun_tab.user, GPIO_HIGH_LEVEL);
    return CC1101_EIO;
  }
  fun_tab.cs_set_level(fun_tab.user,
                       GPIO_HIGH_LEVEL); // set Chip Select to High

  return status;
}

int cc1101_write_strobe(uint8_t strobe) {

  strobe = WRITE(strobe);
  return __spi_write(strobe, NULL, 0, TIMEOUT);
}

int cc1101_read_register(uint8_t reg, uint8_t *value) {

  reg = READ(reg);
  return __spi_read(reg, value, 1, TIMEOUT);
}

int cc1101_write_register(uint8_t reg, uint8_t data) {

  reg = WRITE(reg);
  return __spi_write(reg, &data, 1, TIMEOUT);
}

int cc1101_read_data(uint8_t addr, uint8_t *data, uint8_t size) {

  if (size > 1) {
    addr = READ_BURST(addr);
  } else {
    addr = READ(addr);
  }
  return __spi_read(addr, data, size, TIMEOUT);
}

int cc1101_write_data(uint8_t addr, uint8_t *data, uint8_t size) {

  if (size > 1) {
    addr = WRITE_BURST(addr);
  } else {
    addr = WRITE(addr);
  }
  return __spi_write(addr, data, size, TIMEOUT);
}

int load_config(uint8_t config) {
  if (config == 1) {
    uint8_t status;
    status = cc1101_write_data(0x00, cc1101_cfg_from_smrf,
                               sizeof(cc1101_cfg_from_smrf));
    return status;
  }
  return CC1101_OK;
}