#include "app.h"

#include "main.h"
#include "stm32f4xx_hal.h"
#include "сс1101_api.h"

#include "dwt_stm32_delay.h"

typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  uint32_t cpu_hz;
  uint8_t dwt_ok;
} cc_stm_ctx_t;

static void cs_set_level(void *user, uint8_t level) {
  cc_stm_ctx_t *c = (cc_stm_ctx_t *)user;
  HAL_GPIO_WritePin(c->cs_port, c->cs_pin, level);
}

static int spi_tx(void *user, const uint8_t *address, uint16_t len,
                  uint32_t timeout_ms) {
  cc_stm_ctx_t *c = (cc_stm_ctx_t *)user;
  return (HAL_SPI_Transmit(c->hspi, (uint8_t *)address, len, timeout_ms) ==
          HAL_OK)
             ? 0
             : -1;
}

static int spi_rx(void *user, uint8_t *buffer, uint16_t len,
                  uint32_t timeout_ms) {
  cc_stm_ctx_t *c = (cc_stm_ctx_t *)user;
  return (HAL_SPI_Receive(c->hspi, (uint8_t *)buffer, len, timeout_ms) ==
          HAL_OK)
             ? 0
             : -1;
}

static int spi_txrx(void *user, const uint8_t *address, uint8_t *buffer,
                    uint16_t len, uint32_t timeout_ms) {
  cc_stm_ctx_t *c = (cc_stm_ctx_t *)user;
  return (HAL_SPI_TransmitReceive(c->hspi, (uint8_t *)address, buffer, len,
                                  timeout_ms) == HAL_OK)
             ? 0
             : -1;
}

static void delay_us(void *user, uint32_t us) { DWT_Delay_us(us); }

int cc1101_bind_stm32(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
                      uint16_t cs_pin) {
  static cc_stm_ctx_t ctx;
  memset(&ctx, 0, sizeof(ctx));
  DWT_Delay_Init();
  ctx.hspi = hspi;
  ctx.cs_port = cs_port;
  ctx.cs_pin = cs_pin;

  cc1101_fun_table table = {.cs_set_level = cs_set_level,
                            .spi_txrx = spi_txrx,
                            .spi_tx = spi_tx,
                            .spi_rx = spi_rx,
                            .delay_us = delay_us,
                            .user = &ctx};
  return cc1101_driver_init(&table);
}

void app() {
  int erno;
  erno = cc1101_bind_stm32(&hspi2, SLAVE_SELECT_GPIO_Port, SLAVE_SELECT_Pin);

  cc1101_reset();
  bool status = cc1101_check();
  if (status == false) {
    printf("error check driver ");
  }
  load_config(1);

  while (1) {

    HAL_Delay(500);
  }
}

int _write(int file, char *ptr, int len) {
  (void)file; // hack for fix warning - unused int file.
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 1000);
  return len;
}
