/**
 * Copyright (c) 2014 Google, Inc.
 * Google Confidential/Restricted
 * @author Patrick Titiano
 * @brief TCA6408 GPIO Expander Driver
 */

#ifndef _TSB_TCA6408_H_
#define _TSB_TCA6408_H_

#include <stdint.h>

int tca6408_reset(uint8_t gpio, bool en);
int tca6408_set_direction_in(uint8_t bus, uint8_t addr, uint8_t which);
int tca6408_set_default_outputs(uint8_t bus, uint8_t addr, uint8_t dflt);
int tca6408_set_direction_out(uint8_t bus, uint8_t addr, uint8_t which);
int tca6408_get_direction(uint8_t bus, uint8_t addr, uint8_t which);
int tca6408_set_polarity_inverted(uint8_t bus, uint8_t addr,
                                  uint8_t which, uint8_t inverted);
int tca6408_get_polarity_inverted(uint8_t bus, uint8_t addr, uint8_t which);
int tca6408_set(uint8_t bus, uint8_t addr, uint8_t which, uint8_t val);
int tca6408_get(uint8_t bus, uint8_t addr, uint8_t which);

#endif
