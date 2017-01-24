/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <stdint.h>
#include <platsupport/gpio.h>
#include <platsupport/mux.h>
#include "../../services.h"

#define GPIO_DEBUG
#ifdef GPIO_DEBUG
#define DGPIO(...) printf("GPIO: " __VA_ARGS__)
#else
#define DGPIO(...) do{}while(0)
#endif

#define OMAP4_GPIO_REVISION             0x0000
#define OMAP4_GPIO_EOI                  0x0020
#define OMAP4_GPIO_IRQSTATUSRAW0        0x0024
#define OMAP4_GPIO_IRQSTATUSRAW1        0x0028
#define OMAP4_GPIO_IRQSTATUS0           0x002c
#define OMAP4_GPIO_IRQSTATUS1           0x0030
#define OMAP4_GPIO_IRQSTATUSSET0        0x0034
#define OMAP4_GPIO_IRQSTATUSSET1        0x0038
#define OMAP4_GPIO_IRQSTATUSCLR0        0x003c
#define OMAP4_GPIO_IRQSTATUSCLR1        0x0040
#define OMAP4_GPIO_IRQWAKEN0            0x0044
#define OMAP4_GPIO_IRQWAKEN1            0x0048
#define OMAP4_GPIO_IRQENABLE1           0x011c
#define OMAP4_GPIO_WAKE_EN              0x0120
#define OMAP4_GPIO_IRQSTATUS2           0x0128
#define OMAP4_GPIO_IRQENABLE2           0x012c
#define OMAP4_GPIO_CTRL                 0x0130
#define OMAP4_GPIO_OE                   0x0134
#define OMAP4_GPIO_DATAIN               0x0138
#define OMAP4_GPIO_DATAOUT              0x013c
#define OMAP4_GPIO_LEVELDETECT0         0x0140
#define OMAP4_GPIO_LEVELDETECT1         0x0144
#define OMAP4_GPIO_RISINGDETECT         0x0148
#define OMAP4_GPIO_FALLINGDETECT        0x014c
#define OMAP4_GPIO_DEBOUNCENABLE        0x0150
#define OMAP4_GPIO_DEBOUNCINGTIME       0x0154
#define OMAP4_GPIO_CLEARIRQENABLE1      0x0160
#define OMAP4_GPIO_SETIRQENABLE1        0x0164
#define OMAP4_GPIO_CLEARWKUENA          0x0180
#define OMAP4_GPIO_SETWKUENA            0x0184
#define OMAP4_GPIO_CLEARDATAOUT         0x0190
#define OMAP4_GPIO_SETDATAOUT           0x0194

struct am335x_gpio_reg_offs {
        uint16_t revision;
        uint16_t direction;
        uint16_t datain;
        uint16_t dataout;
        uint16_t set_dataout;
        uint16_t clr_dataout;
        uint16_t irqstatus;
        uint16_t irqstatus2;
        uint16_t irqstatus_raw0;
        uint16_t irqstatus_raw1;
        uint16_t irqenable;
        uint16_t irqenable2;
        uint16_t set_irqenable;
        uint16_t clr_irqenable;
        uint16_t debounce;
        uint16_t debounce_en;
        uint16_t ctrl;
        uint16_t wkup_en;
        uint16_t leveldetect0;
        uint16_t leveldetect1;
        uint16_t risingdetect;
        uint16_t fallingdetect;
        uint16_t irqctrl;
        uint16_t edgectrl1;
        uint16_t edgectrl2;
        uint16_t pinctrl;
};

/*
The am335x is compatible with TI omap4, in the future the two drivers should be merged.
Perhaps a similar feature to linux device trees should be introduced to sel4.

The following struct mirrors the linux driver implementation of omap.
In the future this could be used to support other omap versions.
*/
static struct am335x_gpio_reg_offs omap4_gpio_regs = {
        .revision =             OMAP4_GPIO_REVISION,
        .direction =            OMAP4_GPIO_OE,
        .datain =               OMAP4_GPIO_DATAIN,
        .dataout =              OMAP4_GPIO_DATAOUT,
        .set_dataout =          OMAP4_GPIO_SETDATAOUT,
        .clr_dataout =          OMAP4_GPIO_CLEARDATAOUT,
        .irqstatus =            OMAP4_GPIO_IRQSTATUS0,
        .irqstatus2 =           OMAP4_GPIO_IRQSTATUS1,
        .irqenable =            OMAP4_GPIO_IRQSTATUSSET0,
        .irqenable2 =           OMAP4_GPIO_IRQSTATUSSET1,
        .set_irqenable =        OMAP4_GPIO_IRQSTATUSSET0,
        .clr_irqenable =        OMAP4_GPIO_IRQSTATUSCLR0,
        .debounce =             OMAP4_GPIO_DEBOUNCINGTIME,
        .debounce_en =          OMAP4_GPIO_DEBOUNCENABLE,
        .ctrl =                 OMAP4_GPIO_CTRL,
        .wkup_en =              OMAP4_GPIO_IRQWAKEN0,
        .leveldetect0 =         OMAP4_GPIO_LEVELDETECT0,
        .leveldetect1 =         OMAP4_GPIO_LEVELDETECT1,
        .risingdetect =         OMAP4_GPIO_RISINGDETECT,
        .fallingdetect =        OMAP4_GPIO_FALLINGDETECT,
};


struct am335x_gpio_bank {
    volatile void* base;
    struct am335x_gpio_reg_offs* regs;
};


static struct am335x_gpio_prv {
    //mux_sys_t* mux;
    struct am335x_gpio_bank bank[GPIO_NBANKS];
} _gpio;


/**
 * Read a gpio register
 * @param[in] bank      the bank to read from
 * @param[in] reg       the register offset
 * @return              value stored in the register
 */
inline volatile static uint32_t am335x_gpio_read_reg(struct am335x_gpio_bank* bank, uint16_t reg)
{
    return (*((volatile uint32_t *)(bank->base + reg)));
}

/**
 * Write to a register
 * @param[in] bank     the bank to read from
 * @param[in] reg      the register offset
 * @param[in] v        the value to set
 */
inline static void am335x_gpio_write_reg(struct am335x_gpio_bank* bank, uint16_t reg, uint32_t v)
{
    (*((volatile uint32_t *)(bank->base + reg))) = v;
}

/**
 * Read, Mask, then Write a register
 * @param[in] bank     the bank to read from
 * @param[in] reg      the register offset
 * @param[in] v        the mask to apply
 * @param[in] set      should the mask bits be set or cleared
 */
inline static void am335x_gpio_rmw(struct am335x_gpio_bank* bank, uint16_t reg, uint32_t mask, bool set)
{
    int l = am335x_gpio_read_reg(bank, reg);

    if (set)
        l |= mask;
    else
        l &= ~mask;

    am335x_gpio_write_reg(bank, reg, l);
}

/**
 * Based on a struct representing a single pin, get the DMA bank of registers it belongs to.
 * @param[in] gpio     the pin to lookup.
 * @return             DMA bank struct pointer
 */
static struct am335x_gpio_bank* am335x_gpio_get_bank(gpio_t* gpio) {
    struct am335x_gpio_prv* gpio_priv;
    int port;
    assert(gpio);
    assert(gpio->gpio_sys);
    assert(gpio->gpio_sys->priv);
    gpio_priv = (struct am335x_gpio_prv*)gpio->gpio_sys->priv;
    port = GPIOID_PORT(gpio->id);
    assert(port < GPIO_NBANKS);
    assert(port >= 0);
    return &gpio_priv->bank[port];
}

/**
 * Set the input/output direction of a pin.
 * @param[in] gpio  the pin to configure
 * @param[in] dir   the i/o setting
 */
static void am335x_gpio_set_direction(gpio_t* gpio, enum gpio_dir dir)
{
    struct am335x_gpio_bank* bank = am335x_gpio_get_bank(gpio);

    am335x_gpio_rmw(bank, bank->regs->direction, BIT(GPIOID_PIN(gpio->id)), dir == GPIO_DIR_IN);
}

/**
 * Initialize a pin.
 * @param[in] gpio_sys  The driver data to use
 * @param[in] id        The am335x pin number (e.g. 49 = bank 1, pin 17)
 * @param[in] dir       The i/o direction to set
 * @param[out] gpio     A struct to configure
 * @return              0 on success
 */
static int am335x_gpio_init(gpio_sys_t* gpio_sys, int id, enum gpio_dir dir, gpio_t* gpio)
{
    struct am335x_gpio_prv *gpio_priv;
    uint32_t pin;

    assert(gpio);
    assert(gpio_sys);
    gpio_priv = (struct am335x_gpio_prv*)gpio_sys->priv;
    assert(gpio_priv);
    pin = GPIOID_PIN(id);
    assert(pin < 32);
    assert(pin >= 0);

    gpio->id = id;
    gpio->gpio_sys = gpio_sys;
    gpio->next = NULL;

    DGPIO("Configuring GPIO on port %d pin %d\n",
          GPIOID_PORT(id), GPIOID_PIN(id));

    /* MUX the GPIO
    TODO: IN THE FURTURE THIS NEEDS TO BE IMPLEMENTED.
    It is working without speficially setting the mux right now. */

    // if (am335x_mux_enable_gpio(gpio_priv->mux, id)) {
    //     DGPIO("Invalid GPIO\n");
    //     return -1;
    // }

    am335x_gpio_set_direction(gpio, dir);
    return 0;
}

/**
 * Set a list of pins high or low.
 * @param[in] gpio     Linked list of pins to set
 * @param[in] data     Parallel array of chars to the linked list of pins. "1" causes a write high.
 * @param[in] len      The length of the list and array
 * @return             The number of successful writes completed
 */
static int am335x_gpio_write(gpio_t* gpio, const char* data, int len)
{
    int i;
    for (i = 0; i < len && gpio; i++) {
        struct am335x_gpio_bank* bank = am335x_gpio_get_bank(gpio);
        uint32_t pin = GPIOID_PIN(gpio->id);

        if (*data++) {
            am335x_gpio_write_reg(bank, bank->regs->set_dataout, (1U << pin));
        } else {
            am335x_gpio_write_reg(bank, bank->regs->clr_dataout, (1U << pin));
        }

        gpio = gpio->next;
    }
    return i;
}

/**
 * Read the value on a list of pins.
 * @param[in] gpio      Linked list of pins to read
 * @param[out] data     Parallel array of chars to the linked list of pins.
 * @param[in] len       The length of the list and array
 * @return              The number of successful reads completed
 */
static int am335x_gpio_read(gpio_t* gpio, char* data, int len)
{
    int i;
    for (i = 0; i < len && gpio; i++) {
        struct am335x_gpio_bank* bank = am335x_gpio_get_bank(gpio);
        uint32_t pin = GPIOID_PIN(gpio->id);
        uint32_t v = am335x_gpio_read_reg(bank, bank->regs->datain) & (1U << pin);

        if (v) {
            *data++ = 1;
        } else {
            *data++ = 0;
        }

        gpio = gpio->next;
    }
    return i;
}

/**
 * Initialize a generic gpio driver with platform specific functions.
 * @param[in] mux           A mux data data structure to use when configuring pins
 * @param[out] gpio_sys     A driver to configure
 * @return                  0 on success
 */
int am335x_gpio_init_common(mux_sys_t* mux, gpio_sys_t* gpio_sys)
{
    //_gpio.mux = mux;
    gpio_sys->priv = (void*)&_gpio;
    gpio_sys->read = &am335x_gpio_read;
    gpio_sys->write = &am335x_gpio_write;
    gpio_sys->init = &am335x_gpio_init;
    return 0;
}

/**
 * Configure all of the GPIO banks, and setup the generic gpio driver
 * @param[in] bank0         pointer to am335x GPIO bank 0, should be mapped to physical 0x44e07000
 * @param[in] bank1         pointer to am335x GPIO bank 1, should be mapped to physical 0x4804c000
 * @param[in] bank2         pointer to am335x GPIO bank 2, should be mapped to physical 0x481ac000
 * @param[in] bank3         pointer to am335x GPIO bank 3, should be mapped to physical 0x481ae000
 * @param[out] gpio_sys     A driver to configure
 * @return                  0 on success
 */
int am335x_gpio_sys_init(void* bank0, void* bank1, void* bank2, void* bank3, gpio_sys_t* gpio_sys)
{
    struct am335x_gpio_bank* bank;

    if (bank0 != NULL) {
        bank = &_gpio.bank[GPIO_BANK0];
        bank->base = bank0;
        bank->regs = &omap4_gpio_regs;

        am335x_gpio_write_reg(bank, bank->regs->ctrl, 0);
        DGPIO("Started GPIO Bank 0. Rev: %i\n", am335x_gpio_read_reg(bank, bank->regs->revision));
    }

    if (bank1 != NULL) {
        bank = &_gpio.bank[GPIO_BANK1];
        bank->base = bank1;
        bank->regs = &omap4_gpio_regs;

        am335x_gpio_write_reg(bank, bank->regs->ctrl, 0);
        DGPIO("Started GPIO Bank 1. Rev: %i\n", am335x_gpio_read_reg(bank, bank->regs->revision));
    }

    if (bank2 != NULL) {
        bank = &_gpio.bank[GPIO_BANK2];
        bank->base = bank2;
        bank->regs = &omap4_gpio_regs;

        am335x_gpio_write_reg(bank, bank->regs->ctrl, 0);
        DGPIO("Started GPIO Bank 2. Rev: %i\n", am335x_gpio_read_reg(bank, bank->regs->revision));
    }

    if (bank3 != NULL) {
        bank = &_gpio.bank[GPIO_BANK3];
        bank->base = bank3;
        bank->regs = &omap4_gpio_regs;

        am335x_gpio_write_reg(bank, bank->regs->ctrl, 0);
        DGPIO("Started GPIO Bank 3. Rev: %i\n", am335x_gpio_read_reg(bank, bank->regs->revision));
    }

    return am335x_gpio_init_common(NULL, gpio_sys);
}

/**
 * System initialization
 * @param[out] io_ops
 * @param[out] gpio_sys
 * @return                  0 on success
 */
int gpio_sys_init(ps_io_ops_t* io_ops, gpio_sys_t* gpio_sys)
{
    // MAP_IF_NULL(io_ops, OMAP4_GPIO0, _gpio.bank[0]->base);
    // MAP_IF_NULL(io_ops, OMAP4_GPIO1, _gpio.bank[1]->base);
    // MAP_IF_NULL(io_ops, OMAP4_GPIO2, _gpio.bank[2]->base);
    // MAP_IF_NULL(io_ops, OMAP4_GPIO3, _gpio.bank[3]->base);
    return am335x_gpio_init_common(&io_ops->mux_sys, gpio_sys);
}
