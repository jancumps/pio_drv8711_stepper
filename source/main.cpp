#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "hardware/pio.h"
#include "stepper.pio.h"

#include <array>
#include <iterator>
#include <span>

import drv8711;
// pre-configured registers:
import drv8711_config;

#define MICROSTEP_8
// #undef MICROSTEP_8

const uint nsleep = 14U;
const uint reset = 15U;
const uint dir = 4U;
const uint step = 5U;
const auto piostep = pio1;
const uint sm = 0U;

#ifdef MICROSTEP_8
const float clock_divider = 2; // works well for 8 microsteps
const uint microstep_multiplier = 8;
#else
const float clock_divider = 100; // works well for no microsteps
const uint microstep_multiplier = 1;
#endif


class wakeup_drv8711 { // driver out of sleep as long as object in scope
public:    
    inline wakeup_drv8711() { gpio_put(nsleep, 1); }
    inline ~wakeup_drv8711() { gpio_put(nsleep, 0); }
};

static inline void cs_drive(bool high) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, high? 1 : 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static void spi_write(const uint16_t &data) {
    cs_drive(true); // drv8711 has CS active high
    spi_write16_blocking(spi_default, &data, 1);
    cs_drive(false);
}

void init_drv8711_settings() {
    // override any default settings
    #ifdef MICROSTEP_8
    drv8711::reg_ctrl.mode = 0x0003; // MODE 8 microsteps
    //drv8711::reg_torque.torque = 0x00ff;
    #endif
    // and config over SPI
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

void init_pio() {
    // todo get free sm
    uint offset = pio_add_program(piostep, &stepper_program);
    printf("Loaded program at %d\n", offset);

    stepper_program_init(piostep, sm, offset, dir, clock_divider);
    pio_sm_set_enabled(piostep, sm, true);
}

void init_everything() {
    stdio_init_all();
    init_drv8711_gpio_hw();
    init_drv8711_spi_hw();
    init_drv8711_settings();
    init_pio();
}

// Write `steps` to TX FIFO. State machine will copy this into X.
// max steps taken is 2147483647 (highest number that fits in 31 bits)
void pio_stepper_set_steps(PIO pio, uint sm, uint32_t steps, bool reverse) {
    if (steps > (UINT32_MAX >> 1)) {
        printf("%d is more than max steps (%d)\n", steps, UINT32_MAX >> 1);
        return;
    }
    pio_sm_put_blocking(pio, sm, steps << 1 | (reverse ? 0 : 1));
}

// call when the PIO is free. It interferes with activities
void pio_pwm_set_delay(PIO pio, uint sm, uint32_t delay) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put(pio, sm, delay);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}


struct command {
    uint32_t steps;
    bool reverse;
};

void demo_with_delay(const std::span<command> & cmd, uint32_t delay) {
    pio_pwm_set_delay(piostep, sm, delay);
    for(auto c : cmd) {
        // printf("Steps = %d\n", c.steps);
        pio_stepper_set_steps(piostep, sm, c.steps, c.reverse);
    }
}

int main() {
    init_everything();
    std::array<command, 6> cmd{{
        {200 * microstep_multiplier, true}, 
        {200 * microstep_multiplier, false},
        {200 * microstep_multiplier, false},
        {400 * microstep_multiplier, false},
        {250 * microstep_multiplier, true},
        {350 * microstep_multiplier, true}}
    };

    {
        wakeup_drv8711 w; // wake up the stepper driver
        sleep_ms(1); // see datasheet

        demo_with_delay(cmd, 4300);
        sleep_ms(2000); // give enough time to complete the action
        demo_with_delay(cmd, 7000);
        sleep_ms(3000); // give enough time to complete the action
        demo_with_delay(cmd, 9000);
        sleep_ms(4500); // give enough time to complete the action
    }
    return 0;
}