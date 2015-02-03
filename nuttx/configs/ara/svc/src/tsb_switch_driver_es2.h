/**
 * Copyright (c) 2015 Google, Inc.
 * Google Confidential/Restricted
 * @author: Jean Pihet
 */

#ifndef  _TSB_SWITCH_DRIVER_ES2_H_
#define  _TSB_SWITCH_DRIVER_ES2_H_

#define SW_SPI_PORT         (1)
#define SW_SPI_ID           (0)
#define SWITCH_SPI_FREQUENCY 13000000

// Max NULL frames to wait for a reply from the switch
#define SWITCH_WAIT_REPLY_LEN   (16)
// Write status reply length
#define SWITCH_WRITE_STATUS_LEN (11)

struct tsb_switch_driver *tsb_switch_es2_init(unsigned int spi_bus);
void tsb_switch_es2_exit(void);

#endif
