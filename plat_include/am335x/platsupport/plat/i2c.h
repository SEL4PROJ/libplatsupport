
/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef _PLATSUPPORT_PLAT_I2C_H_
#define _PLATSUPPORT_PLAT_I2C_H_

/* I2C Physical Addresses, 4KB each */
#define   I2C0_PADDR   (0x44E0B000)
#define   I2C1_PADDR   (0x4802A000)
#define   I2C2_PADDR   (0x4819C000)

#define I2C0_INTERRUPT  (70)
#define I2C1_INTERRUPT  (71)
#define I2C2_INTERRUPT  (30)

enum i2c_id {
    I2C0,
    I2C1,
    I2C2,
    NI2C
};

#endif /* _PLATSUPPORT_PLAT_I2C_H_ */
