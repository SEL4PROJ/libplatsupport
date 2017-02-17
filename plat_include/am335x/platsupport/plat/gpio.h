/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef _AM335X_GPIO_H
#define _AM335X_GPIO_H

#include <platsupport/gpio.h>

enum gpio_port {
    GPIO_BANK0,
    GPIO_BANK1,
    GPIO_BANK2,
    GPIO_BANK3,
    GPIO_NBANKS
};

int am335x_gpio_sys_init(void* bank0, void* bank1, void* bank2, void* bank3, gpio_sys_t* gpio_sys);

#endif /* _AM335X_GPIO_H */
