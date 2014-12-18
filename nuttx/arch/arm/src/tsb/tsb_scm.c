/*
 * Copyright (C) 2014 Google, Inc.
 * Author: Benoit Cousson <bcousson@baylibre.com>
 *         Fabien Parent <fparent@baylibre.com>
 */

#include <stdint.h>
#include <debug.h>
#include <nuttx/config.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "tsb_scm.h"


#define TSB_SCM_SOFTRESET0              0x00000000
#define TSB_SCM_SOFTRESETRELEASE0       0x00000100
#define TSB_SCM_CLOCKGATING0            0x00000200
#define TSB_SCM_CLOCKENABLE0            0x00000300
#define TSB_SCM_PINSHARE                0x00000800


static uint32_t scm_read(uint32_t offset)
{
    return getreg32(SYSCTL_BASE + offset);
}

static void scm_write(uint32_t offset, uint32_t v)
{
    putreg32(v, SYSCTL_BASE + offset);
}

void tsb_clk_init(void)
{
    scm_write(TSB_SCM_CLOCKGATING0, INIT_PERIPHERAL_CLOCK_BITS);
}

void tsb_clk_enable(uint32_t clk)
{
    scm_write(TSB_SCM_CLOCKENABLE0 + CLK_OFFSET(clk), CLK_MASK(clk));
}

void tsb_clk_disable(uint32_t clk)
{
    scm_write(TSB_SCM_CLOCKGATING0 + CLK_OFFSET(clk), CLK_MASK(clk));
}

uint32_t tsb_clk_status(uint32_t clk)
{
    return scm_read(TSB_SCM_CLOCKENABLE0 + CLK_OFFSET(clk)) & CLK_MASK(clk);
}

void tsb_reset(uint32_t rst)
{
    scm_write(TSB_SCM_SOFTRESET0 + CLK_OFFSET(rst), CLK_MASK(rst));
    scm_write(TSB_SCM_SOFTRESETRELEASE0 + CLK_OFFSET(rst), CLK_MASK(rst));
}

void tsb_set_pinshare(uint32_t bits)
{
    uint32_t r = scm_read(TSB_SCM_PINSHARE);
    scm_write(TSB_SCM_PINSHARE, r | bits);
}

void tsb_clr_pinshare(uint32_t bits)
{
    uint32_t r = scm_read(TSB_SCM_PINSHARE);
    scm_write(TSB_SCM_PINSHARE, r & ~bits);
}

/* Debug code for command line tool usage */
struct clk_info {
    uint32_t    clk;
    const char  name[16];
};

struct clk_info clk_names[] = {
    { TSB_CLK_WORKRAM,      "workram" },
    { TSB_CLK_SDIOSYS,      "sdio_sys" },
    { TSB_CLK_SDIOSD,       "sdio_sd" },
    { TSB_CLK_BUFRAM,       "bufram" },
    { TSB_CLK_MBOX,         "mbox" },
    { TSB_CLK_GDMA,         "gdma" },
    { TSB_CLK_TMR,          "tmr" },
    { TSB_CLK_WDT,          "wdt" },
    { TSB_CLK_TIMER,        "timer" },
    { TSB_CLK_PWMODP,       "pwmod_p" },
    { TSB_CLK_PWMODS,       "pwmod_s" },
    { TSB_CLK_I2CP,         "i2c_p" },
    { TSB_CLK_I2CS,         "i2c_s" },
    { TSB_CLK_UARTP,        "uart_p" },
    { TSB_CLK_UARTS,        "uart_s" },
    { TSB_CLK_SPIP,         "spi_p" },
    { TSB_CLK_SPIS,         "spi_s" },
    { TSB_CLK_GPIO,         "gpio" },
    { TSB_CLK_I2SSYS,       "i2ssys" },
    { TSB_CLK_I2SBIT,       "i2sbit" },
    { TSB_CLK_UNIPROSYS,    "unipro_sys" },
    { TSB_CLK_UNIPROREF,    "unipro_ref" },
    { TSB_CLK_HSIC480,      "hsic480" },
    { TSB_CLK_HSICBUS,      "hsicbus" },
    { TSB_CLK_HSICREF,      "hsicref" },
    { TSB_CLK_CDSI0_TX_SYS, "cdsi0_tx_sys" },
    { TSB_CLK_CDSI0_TX_APB, "cdsi0_tx_apb" },
    { TSB_CLK_CDSI0_RX_SYS, "cdsi0_rx_sys" },
    { TSB_CLK_CDSI0_RX_APB, "cdsi0_rx_apb" },
    { TSB_CLK_CDSI0_REF,    "cdsi0_ref" },
};

/* TODO: Need to be moved in a common header */
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

void tsb_clk_dump(void)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(clk_names); i++) {
        dbg("%12s: %s\n", clk_names[i].name, (tsb_clk_status(clk_names[i].clk) ? "ON" : "OFF"));
    }
}
