/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== hello.c ========
 */

/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>

#include <ti/drivers/Board.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/dpl/ClockP.h>
#include <string.h>
#include <cc1101_api.h>
/*
 *  ======== main ========
 */

typedef struct
{
    SPI_Handle hspi;
    uint_least8_t cs_gpio;
} cc_cc1310_ctx_t;

static void cs_set_level(void *u, uint8_t level)
{
    cc_cc1310_ctx_t *c = user;
    GPIO_write(c->cs_gpio, level);
}

static int spi_txrx(void *user, const uint8_t *address, uint8_t *buffer,
                    uint16_t len, uint32_t timeout)
{
    cc_cc1310_ctx_t *c = user;
    SPI_Transaction t = { 0 };
    t.count = len;
    t.txBuf = (void*) address;
    t.rxBuf = buffer;
    return SPI_transfer(c->hspi, &t) ? 0 : -1;
}
static int spi_tx(void *user, const uint8_t *address, uint16_t len,
                  uint32_t timeout)
{
    cc_cc1310_ctx_t *c = user;
    SPI_Transaction t = { 0 };
    t.count = len;
    t.txBuf = (void*) address;
    t.rxBuf = NULL;
    return SPI_transfer(c->hspi, &t) ? 0 : -1;
}
static int spi_rx(void *user, uint8_t *buffer, uint16_t len, uint32_t timeout)
{
    cc_cc1310_ctx_t *c = user;
    SPI_Transaction t = { 0 };
    t.count = len;
    t.txBuf = NULL;
    t.rxBuf = buffer;
    return SPI_transfer(c->hspi, &t) ? 0 : -1;
}

int cc1101_bind_cc1310(SPI_Handle hspi, uint_least8_t cs_gpio)
{
    static cc_cc1310_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.hspi = hspi;
    ctx.cs_gpio = cs_gpio;

    cc1101_fun_table table =
            { .cs_set_level = cs_set_level, .spi_txrx = spi_txrx, .spi_tx =
                      spi_tx,
              .spi_rx = spi_rx, .user = &ctx };
    return cc1101_driver_init(&table);
}

int main()
{
    /* Call driver init functions */
    Board_init();
    BIOS_start();
    GPIO_init();
    SPI_init();
    SPI_Handle masterSpi;
    SPI_Params spiParams;
    SPI_Transaction transaction;
    uint32_t i;
    bool transferOK;
    int32_t status;

    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.bitRate = 4000000;
    spiParams.transferMode = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 500000;
    masterSpi = SPI_open(CC1310_LAUNCHXL_SPI0, &spiParams);
    if (masterSpi == NULL)
    {
        System_printf("Error initializing master SPI\n");
        while (1)
            ;
    }

    /*
     *  normal BIOS programs, would call BIOS_start() to enable interrupts
     *  and start the scheduler and kick BIOS into gear.  But, this program
     *  is a simple sanity test and calls BIOS_exit() instead.
     */
    BIOS_exit(0); /* terminates program and dumps SysMin output */
    return (0);
}
