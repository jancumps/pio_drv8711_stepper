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

const uint nsleep = 14U;
const uint reset = 15U;

inline void enable(bool enable) {
    gpio_put(nsleep, enable ? 1 : 0);
}

inline void cs_drive(bool high) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, high? 1 : 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

void spi_write(const uint16_t &data) {
    cs_drive(true); // drv8711 has CS active high
    spi_write16_blocking(spi_default, &data, 1);
    cs_drive(false);
}

drv8711::drv_8711 driver1(drv8711_pico::spi_write, drv8711_pico::enable);

void init_drv8711_settings() {
    spi_write(drv8711::reg_ctrl);
    spi_write(drv8711::reg_torque);
    spi_write(drv8711::reg_off);
    spi_write(drv8711::reg_blank);
    spi_write(drv8711::reg_decay);
    spi_write(drv8711::reg_stall);
    spi_write(drv8711::reg_drive);
    spi_write(drv8711::reg_status);
}

void init_drv8711_spi_hw() {
    // Enable SPI 0 at 1 MHz and connect to GPIOs
    spi_init(spi_default, 1000 * 1000);
    spi_set_format(spi_default, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST); // 16 bit registers
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_pulls(PICO_DEFAULT_SPI_RX_PIN, true, false); // drv8711 outputs are open drain
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    // Chip select is active-high, so we'll initialise it to a driven-low state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
}

void init_drv8711_gpio_hw() {
    // nsleep as output
    gpio_init(nsleep);
    gpio_put(nsleep, 0);
    gpio_set_dir(nsleep, GPIO_OUT);
    // reset as output
    gpio_init(reset);
    gpio_put(reset, 0);
    gpio_set_dir(reset, GPIO_OUT);
}

} // namespace drv8711_pico