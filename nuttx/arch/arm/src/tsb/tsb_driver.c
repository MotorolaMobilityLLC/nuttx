/**
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Mark Greer
 */

#include <nuttx/device.h>

#ifdef CONFIG_ARCH_CHIP_DEVICE_GDMAC
extern struct device_driver tsb_dma_driver;
#endif
extern struct device_driver tsb_usb_hcd_driver;
extern struct device_driver tsb_usb_pcd_driver;
extern struct device_driver tsb_pll_driver;
extern struct device_driver tsb_i2s_driver;
extern struct device_driver tsb_pwm_driver;
extern struct device_driver tsb_spi_driver;
extern struct device_driver tsb_uart_driver;

void tsb_driver_register(void)
{
#ifdef CONFIG_ARCH_CHIP_DEVICE_GDMAC
    device_register_driver(&tsb_dma_driver);
#endif

#ifdef CONFIG_ARCH_CHIP_USB_HCD
    device_register_driver(&tsb_usb_hcd_driver);
#endif

#ifdef CONFIG_ARCH_CHIP_USB_PCD
    device_register_driver(&tsb_usb_pcd_driver);
#endif

#ifdef CONFIG_ARCH_CHIP_TSB_PLL
    device_register_driver(&tsb_pll_driver);
#endif

#ifdef CONFIG_ARCH_CHIP_TSB_I2S
    device_register_driver(&tsb_i2s_driver);
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_PWM
    device_register_driver(&tsb_pwm_driver);
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_SPI
    device_register_driver(&tsb_spi_driver);
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_UART
    device_register_driver(&tsb_uart_driver);
#endif
}
