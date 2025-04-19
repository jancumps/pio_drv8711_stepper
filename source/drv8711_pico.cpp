/**
 * DRV8711 Pico and project specific code
 * SPI, nSleep and reset pins + configs are set up here.
 */

module;

#include "hardware/gpio.h"
#include "hardware/spi.h"

import drv8711;
import drv8711_config;

export module drv8711_pico;
export namespace drv8711_pico {

class driver_pico : public drv8711::driver {
public:    
    driver_pico(spi_inst_t *spi, 
        uint csn, uint rx, uint tx, uint sck,
        uint n_sleep, uint reset) : drv8711::driver(),
        spi_(spi), 
        csn_(csn), rx_(rx), tx_(tx), sck_(sck),
        n_sleep_(n_sleep), reset_(reset) {}
    
    virtual void write(uint16_t data) override {
        cs_drive(true); // drv8711 has CS active high
        spi_write16_blocking(spi_, &data, 1);
        cs_drive(false);
    }
    
    virtual void enable(bool enable) override {
        gpio_put(n_sleep_, enable ? 1 : 0);
    }

    void init_registers() {
        write(drv8711::reg_ctrl);
        write(drv8711::reg_torque);
        write(drv8711::reg_off);
        write(drv8711::reg_blank);
        write(drv8711::reg_decay);
        write(drv8711::reg_stall);
        write(drv8711::reg_drive);
        write(drv8711::reg_status);
    }

    void init_spi() {
        // Enable SPI 0 at 1 MHz and connect to GPIOs
        spi_init(spi_, 1000 * 1000);
        spi_set_format(spi_, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST); // 16 bit registers
        gpio_set_function(rx_, GPIO_FUNC_SPI);
        gpio_set_pulls(rx_, true, false); // drv8711 outputs are open drain
        gpio_set_function(sck_, GPIO_FUNC_SPI);
        gpio_set_function(tx_, GPIO_FUNC_SPI);
        // Chip select is active-high, so we'll initialise it to a driven-low state
        gpio_init(csn_);
        gpio_put(csn_, 0);
        gpio_set_dir(csn_, GPIO_OUT);
    }

    void init_gpio() {
        // nsleep as output
        gpio_init(n_sleep_);
        gpio_put(n_sleep_, 0);
        gpio_set_dir(n_sleep_, GPIO_OUT);
        // reset as output
        gpio_init(reset_);
        gpio_put(reset_, 0);
        gpio_set_dir(reset_, GPIO_OUT);
    }

private:
    inline void cs_drive( bool high) {
        asm volatile("nop \n nop \n nop"); // FIXME
        gpio_put(csn_, high? 1 : 0);
        asm volatile("nop \n nop \n nop"); // FIXME
    }
    spi_inst_t * spi_;
    uint csn_;
    uint rx_;
    uint tx_;
    uint sck_;
    uint n_sleep_;
    uint reset_;
};

} // namespace drv8711_pico