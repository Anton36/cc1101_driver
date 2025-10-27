
#ifndef __CC1101_API_H__
#define __CC1101_API_H__


#include <stdint.h>
#include <stddef.h>














enum {
    CC1101_OK = 0,
    CC1101_TIMEOUT = -1,
    CC1101_EINVAL = -2,
    CC1101_EIO = -3,

};

typedef struct {
  void (*cs_set_level)(void *user,uint8_t level);
  int (*spi_tx)(void *user,const uint8_t *address,uint16_t len,uint32_t timeout_ms);
  int (*spi_rx)(void *user, uint8_t *buffer,uint16_t len,uint32_t timeout_ms);
  int (*spi_txrx)(void *user,const uint8_t *address, uint8_t *buffer,uint16_t len,uint32_t timeout_ms);
  void (*delay_us)(void *user, uint32_t us);

  void *user;

} cc1101_fun_table;

int cc1101_driver_init(cc1101_fun_table *fun_table);
void cc1101_reset();
bool cc1101_check();
int load_config(uint8_t config);


#endif